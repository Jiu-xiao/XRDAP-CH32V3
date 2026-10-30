// SPDX-License-Identifier: Apache-2.0
#include "sx1281.hpp"

#include <initializer_list>

#include XRDAP_BOARD_HEADER
#include "ch32_gpio.hpp"
#include "ch32_spi.hpp"
#include "fast_gpio.hpp"
#include "libxr.hpp"
#include "thread.hpp"
#include "timebase.hpp"

namespace XrDap
{
namespace
{
// SX1281 opcodes (section 11, table 11-1).
constexpr uint8_t kSetStandby = 0x80, kSetFs = 0xC1, kSetTx = 0x83, kSetRx = 0x82;
constexpr uint8_t kSetAutoFs = 0x9E;
constexpr uint8_t kSetPacketType = 0x8A, kSetRfFrequency = 0x86, kSetBufferBase = 0x8F;
constexpr uint8_t kSetModulation = 0x8B, kSetPacketParams = 0x8C, kSetTxParams = 0x8E;
constexpr uint8_t kSetDioIrq = 0x8D, kClearIrq = 0x97, kGetIrqStatus = 0x15;
constexpr uint8_t kGetRxBufferStatus = 0x17, kGetPacketStatus = 0x1D;
constexpr uint8_t kWriteBuffer = 0x1A, kReadBuffer = 0x1B, kWriteRegister = 0x18;
constexpr uint8_t kStandbyXosc = 0x01;

// Packet parameters (14.1.1 GFSK, 14.3.1 FLRC): 16-bit preamble, the minimum for these
// bit rates; variable length; 2-byte CRC.
constexpr uint8_t kPreamble = 0x30, kVariableLength = 0x20;
struct ModemParams
{
  uint8_t packet_type;
  uint8_t modulation[3];
  uint8_t sync_length, sync_match, crc, whitening;
  bool sync_used;
};
// GFSK 2 Mb/s, 2.4 MHz bandwidth, modulation index 0.5, BT 0.5 (tables 14-1..14-3);
// 4-byte sync word 1 (0x09CF..0x09D2 of SyncWord1 at 0x09CE), whitening on.
constexpr ModemParams kGfsk{0x00, {0x04, 0x01, 0x20}, 0x06, 0x10, 0x20, 0x00, true};
// FLRC 1.3 Mb/s, 1.2 MHz bandwidth, BT 1.0 (tables 14-31..14-33), coding rate 1 (no FEC,
// 0x04); whitening off (table 14-41); 32-bit sync word 1 at 0x09CF. Without the sync word
// the receiver takes noise for packets. Errata 16.4 lists the sync words to avoid at
// coding rates 1/2 and 3/4 ("XRDP" is allowed); for coding rate 1 none are published.
constexpr ModemParams kFlrc{0x03, {0x45, 0x04, 0x10}, 0x04, 0x10, 0x10, 0x08, true};
constexpr uint8_t kFlrcSyncWord[4] = {'X', 'R', 'D', 'P'};
// CRC seed (tables 14-12 and 14-40, reset value 0xFFFF).
constexpr uint16_t kCrcSeedReg = 0x09C8;

// IRQ bits (table 11-73), all on DIO1: TxDone while transmitting, RxDone and CrcError
// while receiving.
constexpr uint16_t kIrqTxDone = 0x0001, kIrqRxDone = 0x0002, kIrqCrcError = 0x0040;
constexpr uint16_t kIrqMask = kIrqTxDone | kIrqRxDone | kIrqCrcError;
// Latched in the IRQ status only, for the stuck-preamble check (see sx1281.hpp).
constexpr uint16_t kIrqSyncValid = 0x0004, kIrqPreamble = 0x8000;
constexpr uint16_t kIrqLatched = kIrqMask | kIrqSyncValid | kIrqPreamble;
// Error byte of GetPacketStatus (table 11-68): LengthError, CrcError, AbortError.
constexpr uint8_t kPacketErrors = 0x20 | 0x10 | 0x08;

// Both directions use the whole 256-byte buffer from address 0 (section 8).
constexpr uint8_t kTxBase = 0x00, kRxBase = 0x00;

constexpr int8_t kPowerDbm = 0;  // the module's PA follows
// PA ramp time code of SetTxParams (table 11-49): 8 us.
constexpr uint8_t kRamp = 0x60;
// Every SPI transfer goes byte by byte through the data register (PolledTransfer). A
// flag that does not come within this many polls (a few ms at 144 MHz; a byte takes
// 0.44 us at 18 MHz) fails the transfer.
constexpr uint32_t kSpiSpinLimit = 100'000;
constexpr uint32_t kBusyTimeoutUs = 20'000;
constexpr uint32_t kTxTimeoutUs = 5'000;  // the longest packet takes about 1.3 ms
constexpr uint16_t kMaxTransfer = 3 + Sx1281::kMaxPayload;
// CH32SPI moves larger transfers by DMA; none reach it (see PolledTransfer).
constexpr uint16_t kSpiDmaMin = kMaxTransfer;
// Silence recovery: enter receive again after kRearmUs without a packet, reset the radio
// after kRestartUs.
constexpr uint32_t kRearmUs = 100'000;
constexpr uint32_t kRestartUs = 2'000'000;
// Stuck preamble: look every kStuckCheckUs, enter receive again after kStuckUs.
constexpr uint32_t kStuckCheckUs = 1'000;
constexpr uint32_t kStuckUs = 2'000;

alignas(4) uint8_t dma_rx[kMaxTransfer];
alignas(4) uint8_t dma_tx[kMaxTransfer];

struct Hardware
{
  // SPI1 on PA5/PA6/PA7 (APB2 = core clock), mode 0 (9.3): 144 MHz / 8 = 18 MHz, the
  // SX1281's limit; at 216 MHz / 16 = 13.5 MHz. CH32SPI sets up the pins, the clock and
  // the mode; transfers go through PolledTransfer, not through CH32SPI.
  LibXR::CH32SPI spi{CH32_SPI1,
                     {dma_rx, sizeof(dma_rx)},
                     {dma_tx, sizeof(dma_tx)},
                     Board::kRadioSck.Port(),
                     Board::kRadioSck.Mask(),
                     Board::kRadioMiso.Port(),
                     Board::kRadioMiso.Mask(),
                     Board::kRadioMosi.Port(),
                     Board::kRadioMosi.Mask(),
                     0,
                     true,
                     true,
                     XRDAP_APP_SYSCLK_HZ > 144'000'000 ? SPI_BaudRatePrescaler_16
                                                       : SPI_BaudRatePrescaler_8,
                     kSpiDmaMin};
  LibXR::CH32GPIO nss{Board::kRadioNss.Port(), Board::kRadioNss.Mask()};
  LibXR::CH32GPIO nreset{Board::kRadioNreset.Port(), Board::kRadioNreset.Mask()};
  LibXR::CH32GPIO txen{Board::kRadioTxEn.Port(), Board::kRadioTxEn.Mask()};
  LibXR::CH32GPIO rxen{Board::kRadioRxEn.Port(), Board::kRadioRxEn.Mask()};
  LibXR::CH32GPIO busy{Board::kRadioBusy.Port(), Board::kRadioBusy.Mask(),
                       LibXR::GPIO::Direction::INPUT};
  LibXR::CH32GPIO dio1{Board::kRadioDio1.Port(), Board::kRadioDio1.Mask(),
                       LibXR::GPIO::Direction::INPUT};
};

Hardware* hw = nullptr;

uint32_t NowUs() { return static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds()); }

// NSS, BUSY and DIO1 are touched for every SPI transfer and packet: register access.
using FastNss = FastGpioFor<Board::kRadioNss>;
using FastBusy = FastGpioFor<Board::kRadioBusy>;
using FastDio1 = FastGpioFor<Board::kRadioDio1>;
FastNss fast_nss;
FastBusy fast_busy;
FastDio1 fast_dio1;

// Transfers bypass CH32SPI's buffers and DMA: one byte at a time through the data
// register (RM SPI chapter, TXE/RXNE). Only one byte is in flight, so an interrupt in
// the middle only stretches the transfer and cannot overrun the receive register. The
// whole transfer blocks either way, so DMA saved no time; and measured on a wdap remote
// sampling at 100 us, a CH32SPI DMA transfer now and then stopped part way (TXE set, the
// TX channel not refilling it), timed out and left both channels armed: the RX channel
// then took the bytes of the following transfers, the radio received garbled commands
// and went deaf, or a wait for RXNE never ended and the watchdog reset the board.
bool PolledTransfer(const uint8_t* tx, uint8_t* rx, uint16_t size)
{
  SPI_TypeDef* spi = SPI1;
  (void)spi->DATAR;  // drop a stale byte
  for (uint16_t i = 0; i < size; ++i)
  {
    uint32_t spins = 0;
    while ((spi->STATR & SPI_I2S_FLAG_TXE) == 0U)
    {
      if (++spins >= kSpiSpinLimit) return false;
    }
    spi->DATAR = tx[i];
    spins = 0;
    while ((spi->STATR & SPI_I2S_FLAG_RXNE) == 0U)
    {
      if (++spins >= kSpiSpinLimit) return false;
    }
    const uint8_t byte = static_cast<uint8_t>(spi->DATAR);
    if (rx != nullptr) rx[i] = byte;
  }
  return true;
}

template <typename Pin>
bool WaitLevel(const Pin& pin, bool level, uint32_t timeout_us, uint32_t& waited_us)
{
  if (pin.Read() == level)
  {
    waited_us = 0;
    return true;
  }
  const uint32_t start = NowUs();
  while (pin.Read() != level)
  {
    if (NowUs() - start >= timeout_us)
    {
      waited_us = timeout_us;
      return false;
    }
  }
  waited_us = NowUs() - start;
  return true;
}
}  // namespace

Sx1281& Radio()
{
  static Sx1281 radio;
  return radio;
}

void Sx1281::Init()
{
  static Hardware instance;
  instance.nss.Write(true);
  instance.txen.Write(false);
  instance.rxen.Write(false);
  instance.nreset.Write(false);
  hw = &instance;
}

bool Sx1281::Transfer(const uint8_t* tx, uint8_t* rx, uint16_t size)
{
  // 9.1: a command may only start while BUSY is low.
  uint32_t waited = 0;
  if (size == 0U || size > kMaxTransfer ||
      !WaitLevel(fast_busy, false, kBusyTimeoutUs, waited))
    return false;
  fast_nss.Write(false);
  const bool ok = PolledTransfer(tx, rx, size);
  fast_nss.Write(true);
  return ok;
}

bool Sx1281::Command(std::initializer_list<uint8_t> bytes)
{
  return Transfer(bytes.begin(), nullptr, static_cast<uint16_t>(bytes.size()));
}

void Sx1281::ClearIrq() { Command({kClearIrq, 0xFF, 0xFF}); }

void Sx1281::Configure(bool gfsk, const Identity& identity)
{
  gfsk_ = gfsk;
  identity_ = identity;
}

bool Sx1281::WriteSyncWord()
{
  // GFSK: SyncWord1 byte 4 (0x09CE) is 0 and the 4-byte word starts at 0x09CF; FLRC
  // uses 0x09CF..0x09D2 only.
  const uint8_t* word = gfsk_ ? identity_.sync_word : kFlrcSyncWord;
  const uint8_t gfsk_frame[] = {kWriteRegister, 0x09,    0xCE,    0x00,
                                word[0],        word[1], word[2], word[3]};
  const uint8_t flrc_frame[] = {kWriteRegister, 0x09,    0xCF,   word[0],
                                word[1],        word[2], word[3]};
  return gfsk_ ? Transfer(gfsk_frame, nullptr, sizeof(gfsk_frame))
               : Transfer(flrc_frame, nullptr, sizeof(flrc_frame));
}

void Sx1281::SetPacketParams(uint8_t size)
{
  const ModemParams& m = gfsk_ ? kGfsk : kFlrc;
  Command({kSetPacketParams, kPreamble, m.sync_length, m.sync_match, kVariableLength,
           size, m.crc, m.whitening});
}

void Sx1281::Receive()
{
  // 11.6.5: IRQs are cleared by the caller before SetRx.
  hw->txen.Write(false);
  hw->rxen.Write(true);
  Command({kSetRx, 0x00, 0xFF, 0xFF});  // continuous
  preamble_seen_ = false;
  // Silence counts from here: right after a transmit the answer is still on its way.
  rearmed_us_ = NowUs();
}

bool Sx1281::Start()
{
  burst_count_ = 0;  // a burst in progress ends with the reset
  hw->nreset.Write(false);
  LibXR::Thread::Sleep(2);
  hw->nreset.Write(true);
  LibXR::Thread::Sleep(20);

  // 14.1.1 / 14.3.1: STDBY_RC, packet type first, then frequency, buffer, modulation and
  // packet parameters, sync word, Tx parameters, IRQ routing.
  const ModemParams& m = gfsk_ ? kGfsk : kFlrc;
  const uint32_t step = static_cast<uint32_t>(
      (static_cast<uint64_t>(identity_.frequency_hz) << 18) / 52'000'000ULL);
  bool ok =
      Command({kSetStandby, 0x00}) && Command({kSetPacketType, m.packet_type}) &&
      Command({kSetRfFrequency, static_cast<uint8_t>(step >> 16),
               static_cast<uint8_t>(step >> 8), static_cast<uint8_t>(step)}) &&
      Command({kSetBufferBase, kTxBase, kRxBase}) &&
      Command({kSetModulation, m.modulation[0], m.modulation[1], m.modulation[2]}) &&
      Command({kSetPacketParams, kPreamble, m.sync_length, m.sync_match, kVariableLength,
               MaxPayload(), m.crc, m.whitening}) &&
      (!m.sync_used || WriteSyncWord()) &&
      Command({kWriteRegister, kCrcSeedReg >> 8, kCrcSeedReg & 0xFF,
               static_cast<uint8_t>(identity_.crc_seed >> 8),
               static_cast<uint8_t>(identity_.crc_seed)}) &&
      Command({kSetTxParams, static_cast<uint8_t>(kPowerDbm + 18), kRamp}) &&
      Command({kSetDioIrq, kIrqLatched >> 8, kIrqLatched & 0xFF, kIrqMask >> 8,
               kIrqMask & 0xFF, 0, 0, 0, 0}) &&
      Command({kSetAutoFs, 0x01});
  ClearIrq();
  Receive();
  heard_us_ = NowUs();
  return ok;
}

uint8_t Sx1281::Load(Fill fill, void* ctx, uint8_t index)
{
  static uint8_t frame[2 + kMaxPayload];
  const uint8_t size = fill(ctx, index, frame + 2);
  frame[0] = kWriteBuffer;
  frame[1] = kTxBase;
  Transfer(frame, nullptr, static_cast<uint16_t>(2U + size));
  return size;
}

bool Sx1281::Transmit(const uint8_t* data, uint8_t size)
{
  if (size == 0U || size > MaxPayload()) return false;
  struct Packet
  {
    const uint8_t* data;
    uint8_t size;
  } packet{data, size};
  return TransmitBurst(
      1,
      +[](void* ctx, uint8_t, uint8_t* out) -> uint8_t
      {
        const auto* p = static_cast<const Packet*>(ctx);
        for (uint8_t i = 0; i < p->size; ++i) out[i] = p->data[i];
        return p->size;
      },
      &packet);
}

void Sx1281::PrepareTransmit()
{
  // Leave receive for FS (10.8: FS to Tx 27 us).
  Command({kSetFs});
  ClearIrq();  // 11.6.4: clear IRQs before SetTx
  prepared_ = true;
}

bool Sx1281::TransmitBurst(uint8_t count, Fill fill, void* ctx)
{
  StartBurst(count, fill, ctx);
  while (ContinueBurst())
  {
  }
  return burst_sent_;
}

void Sx1281::SendLoaded(uint8_t size)
{
  SetPacketParams(size);
  Command({kSetTx, 0x00, 0x00, 0x00});
  burst_sent_us_ = NowUs();
}

void Sx1281::StartBurst(uint8_t count, Fill fill, void* ctx)
{
  if (count == 0U) return;
  if (!prepared_) PrepareTransmit();
  prepared_ = false;
  hw->rxen.Write(false);
  hw->txen.Write(true);
  burst_fill_ = fill;
  burst_ctx_ = ctx;
  burst_count_ = count;
  burst_index_ = 0;
  burst_sent_ = true;
  SendLoaded(Load(fill, ctx, 0));
}

bool Sx1281::ContinueBurst()
{
  if (burst_count_ == 0U) return false;
  // DIO1 carries TxDone while transmitting (see the file header).
  if (!fast_dio1.Read())
  {
    if (NowUs() - burst_sent_us_ < kTxTimeoutUs) return true;
    burst_sent_ = false;
  }
  ClearIrq();
  if (++burst_index_ < burst_count_)
  {
    SendLoaded(Load(burst_fill_, burst_ctx_, burst_index_));
    return true;
  }
  burst_count_ = 0;
  // Receive accepts any length up to the maximum (variable length filters longer ones).
  SetPacketParams(MaxPayload());
  Receive();
  return false;
}

void Sx1281::Recover()
{
  const uint32_t now = NowUs();
  if (now - heard_us_ >= kRestartUs)
  {
    Start();
  }
  else if (now - rearmed_us_ >= kRearmUs || PreambleStuck(now))
  {
    Command({kSetStandby, kStandbyXosc});
    ClearIrq();
    Receive();
  }
}

bool Sx1281::PreambleStuck(uint32_t now)
{
  if (now - checked_us_ < kStuckCheckUs) return false;
  checked_us_ = now;
  uint8_t status[4] = {kGetIrqStatus, 0, 0, 0};
  Transfer(status, status, sizeof(status));
  const uint16_t irq = static_cast<uint16_t>((status[2] << 8) | status[3]);
  if ((irq & kIrqPreamble) == 0U || (irq & (kIrqSyncValid | kIrqRxDone)) != 0U)
  {
    preamble_seen_ = false;
    return false;
  }
  if (!preamble_seen_)
  {
    preamble_seen_ = true;
    preamble_us_ = now;
    return false;
  }
  return now - preamble_us_ >= kStuckUs;
}

bool Sx1281::PacketBad()
{
  // Errata 16.3: RxDone does not filter packets with CRC errors; check the error byte
  // (packetStatus[23:16], table 11-68).
  uint8_t status[7] = {kGetPacketStatus, 0, 0, 0, 0, 0, 0};
  Transfer(status, status, sizeof(status));
  return (status[4] & kPacketErrors) != 0U;
}

int Sx1281::Poll(uint8_t* data)
{
  if (prepared_)
  {
    // Prepared for a transmit that did not come: listen again.
    prepared_ = false;
    Receive();
    return 0;
  }
  if (!fast_dio1.Read())
  {
    Recover();
    return 0;
  }
  heard_us_ = rearmed_us_ = NowUs();
  ClearIrq();
  if (PacketBad()) return -1;

  uint8_t status[4] = {kGetRxBufferStatus, 0, 0, 0};
  Transfer(status, status, sizeof(status));
  const uint8_t size = status[2], offset = status[3];
  if (size == 0U || size > MaxPayload()) return -1;
  static uint8_t frame[3 + kMaxPayload];
  frame[0] = kReadBuffer;
  frame[1] = offset;
  frame[2] = 0;
  for (uint8_t i = 0; i < size; ++i) frame[3 + i] = 0;
  Transfer(frame, frame, static_cast<uint16_t>(3U + size));
  for (uint8_t i = 0; i < size; ++i) data[i] = frame[3 + i];
  return size;
}
}  // namespace XrDap
