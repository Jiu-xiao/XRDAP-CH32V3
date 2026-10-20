// SPDX-License-Identifier: Apache-2.0
#include "sx1281.hpp"

#include <initializer_list>

#include XRDAP_BOARD_HEADER
#include "ch32_gpio.hpp"
#include "ch32_spi.hpp"
#include "fast_gpio.hpp"
#include "libxr.hpp"
#include "semaphore.hpp"
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
constexpr uint8_t kSetDioIrq = 0x8D, kClearIrq = 0x97, kGetIrq = 0x15;
constexpr uint8_t kGetRxBufferStatus = 0x17, kGetPacketStatus = 0x1D;
constexpr uint8_t kWriteBuffer = 0x1A, kReadBuffer = 0x1B, kWriteRegister = 0x18;
constexpr uint8_t kStandbyXosc = 0x01;

constexpr bool kRxSingle = Sx1281::kRxSingle;
constexpr bool kCrcFromStatus = XRDAP_RADIO_CRC_STATUS != 0;
constexpr bool kAutoFs = XRDAP_RADIO_AUTOFS != 0;
constexpr bool kIrqFromPin = XRDAP_RADIO_IRQ_FROM_PIN != 0;

// Packet parameters (14.1.1 GFSK, 14.3.1 FLRC): 16-bit preamble, the minimum for these bit
// rates; variable length; 2-byte CRC.
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
// FLRC 1.3 Mb/s, 1.2 MHz bandwidth, BT 1.0 (tables 14-31..14-33), coding rate from
// -DXRDAP_RADIO_CODING_RATE; whitening off (table 14-41); 32-bit sync word 1 at 0x09CF
// (see XRDAP_RADIO_FLRC_SYNC for errata 16.4).
constexpr bool kFlrcSync = XRDAP_RADIO_FLRC_SYNC != 0;
constexpr ModemParams kFlrc{0x03,
                            {0x45, XRDAP_RADIO_CODING_RATE, 0x10},
                            kFlrcSync ? uint8_t{0x04} : uint8_t{0x00},
                            kFlrcSync ? uint8_t{0x10} : uint8_t{0x00},
                            0x10,
                            0x08,
                            kFlrcSync};
constexpr uint8_t kFlrcSyncWord[4] = {'X', 'R', 'D', 'P'};
// CRC seed (tables 14-12 and 14-40, reset value 0xFFFF).
constexpr uint16_t kCrcSeedReg = 0x09C8;

// IRQ bits (table 11-73), all on DIO1.
constexpr uint16_t kIrqTxDone = 0x0001, kIrqRxDone = 0x0002, kIrqCrcError = 0x0040;
constexpr uint16_t kIrqTimeout = 0x4000;
constexpr uint16_t kIrqMask = kIrqTxDone | kIrqRxDone | kIrqCrcError | kIrqTimeout;
// Error byte of GetPacketStatus (table 11-68): LengthError, CrcError, AbortError.
constexpr uint8_t kPacketErrors = 0x20 | 0x10 | 0x08;

// Transmit packets start at 0x00, or alternate with 0x80 with ping-pong. Received packets
// start at rx_base_: single receive restarts the pointer at rxBaseAddress (8.2), so after
// each packet receive restarts in the other half while that packet is read out.
constexpr uint8_t kTxBase = 0x00, kHalf = 0x80;
// Single receive times out after kRxTimeoutMs (1 ms steps, table 11-24) and is entered
// again; the timeout also proves the radio still answers.
constexpr uint8_t kRxTimeoutMs = 100;

constexpr int8_t kPowerDbm = 0;  // the module's PA follows
constexpr uint32_t kBusyTimeoutUs = 20'000;
constexpr uint32_t kTxTimeoutUs = 5'000;  // the longest packet takes about 1.3 ms
constexpr uint16_t kMaxTransfer = 3 + Sx1281::kMaxPayload;
static_assert(!(Sx1281::kTxPingPong || Sx1281::kRxSingle) || Sx1281::kMaxPayload <= kHalf,
              "a packet must fit half of the buffer");
// Silence recovery: enter receive again after kRearmUs without any report (continuous
// receive only), reset the radio after kRestartUs.
constexpr uint32_t kRearmUs = 100'000;
constexpr uint32_t kRestartUs = 2'000'000;

alignas(4) uint8_t dma_rx[kMaxTransfer];
alignas(4) uint8_t dma_tx[kMaxTransfer];

struct Hardware
{
  // SPI1 on PA5/PA6/PA7 (APB2 = core clock), mode 0 (9.3): 144 MHz / 8 = 18 MHz, the
  // SX1281's limit; at 216 MHz / 16 = 13.5 MHz.
  LibXR::CH32SPI spi{CH32_SPI1, {dma_rx, sizeof(dma_rx)}, {dma_tx, sizeof(dma_tx)},
                     Board::kRadioSck.Port(), Board::kRadioSck.Mask(),
                     Board::kRadioMiso.Port(), Board::kRadioMiso.Mask(),
                     Board::kRadioMosi.Port(), Board::kRadioMosi.Mask(),
                     0, true, true,
                     XRDAP_APP_SYSCLK_HZ > 144'000'000 ? SPI_BaudRatePrescaler_16
                                                       : SPI_BaudRatePrescaler_8,
                     XRDAP_RADIO_SPI_DMA_MIN};
  LibXR::CH32GPIO nss{Board::kRadioNss.Port(), Board::kRadioNss.Mask()};
  LibXR::CH32GPIO nreset{Board::kRadioNreset.Port(), Board::kRadioNreset.Mask()};
  LibXR::CH32GPIO txen{Board::kRadioTxEn.Port(), Board::kRadioTxEn.Mask()};
  LibXR::CH32GPIO rxen{Board::kRadioRxEn.Port(), Board::kRadioRxEn.Mask()};
  LibXR::CH32GPIO busy{Board::kRadioBusy.Port(), Board::kRadioBusy.Mask(),
                       LibXR::GPIO::Direction::INPUT};
  LibXR::CH32GPIO dio1{Board::kRadioDio1.Port(), Board::kRadioDio1.Mask(),
                       LibXR::GPIO::Direction::INPUT};
  LibXR::CH32GPIO dio2{Board::kRadioDio2.Port(), Board::kRadioDio2.Mask(),
                       LibXR::GPIO::Direction::INPUT};
  LibXR::CH32GPIO dio3{Board::kRadioDio3.Port(), Board::kRadioDio3.Mask(),
                       LibXR::GPIO::Direction::INPUT};
  LibXR::Semaphore done;
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

// Short transfers bypass CH32SPI's buffers and DMA: one byte at a time through the data
// register (RM SPI chapter, TXE/RXNE). Only one byte is in flight, so an interrupt in
// the middle only stretches the transfer and cannot overrun the receive register. The
// DMA channels stay disabled between CH32SPI transfers, so their requests do not
// interfere.
void PolledTransfer(const uint8_t* tx, uint8_t* rx, uint16_t size)
{
  SPI_TypeDef* spi = SPI1;
  (void)spi->DATAR;  // drop a stale byte
  for (uint16_t i = 0; i < size; ++i)
  {
    while ((spi->STATR & SPI_I2S_FLAG_TXE) == 0U)
    {
    }
    spi->DATAR = tx[i];
    while ((spi->STATR & SPI_I2S_FLAG_RXNE) == 0U)
    {
    }
    const uint8_t byte = static_cast<uint8_t>(spi->DATAR);
    if (rx != nullptr) rx[i] = byte;
  }
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
  const uint32_t begin = NowUs();
  bool ok = true;
  fast_nss.Write(false);
  if (size <= XRDAP_RADIO_SPI_DMA_MIN)
  {
    PolledTransfer(tx, rx, size);
  }
  else
  {
    LibXR::SPI::OperationRW op(hw->done, 10);
    ok = hw->spi.ReadAndWrite({rx, rx != nullptr ? size : 0U}, {tx, size}, op, false) ==
         LibXR::ErrorCode::OK;
  }
  fast_nss.Write(true);
  ++spi_transfers;
  spi_us += NowUs() - begin;
  busy_us += waited;
  return ok;
}

bool Sx1281::Command(std::initializer_list<uint8_t> bytes)
{
  return Transfer(bytes.begin(), nullptr, static_cast<uint16_t>(bytes.size()));
}

void Sx1281::ClearIrq() { Command({kClearIrq, 0xFF, 0xFF}); }

Sx1281::Identity Sx1281::DefaultIdentity()
{
  return {2'440'000'000U, 0xFFFF, {'X', 'R', 'D', 'P'}};
}

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
  const uint8_t gfsk_frame[] = {kWriteRegister, 0x09, 0xCE, 0x00,
                                word[0],        word[1], word[2], word[3]};
  const uint8_t flrc_frame[] = {kWriteRegister, 0x09, 0xCF, word[0], word[1], word[2], word[3]};
  return gfsk_ ? Transfer(gfsk_frame, nullptr, sizeof(gfsk_frame))
               : Transfer(flrc_frame, nullptr, sizeof(flrc_frame));
}

void Sx1281::SetPacketParams(uint8_t size)
{
  const ModemParams& m = gfsk_ ? kGfsk : kFlrc;
  Command({kSetPacketParams, kPreamble, m.sync_length, m.sync_match, kVariableLength, size,
           m.crc, m.whitening});
}

void Sx1281::Receive()
{
  // 11.6.5: IRQs are cleared by the caller before SetRx.
  hw->txen.Write(false);
  hw->rxen.Write(true);
  if (kRxSingle)
    Command({kSetRx, 0x02, 0x00, kRxTimeoutMs});  // single, 1 ms steps
  else
    Command({kSetRx, 0x00, 0xFF, 0xFF});  // continuous
  // Silence counts from here: right after a transmit the answer is still on its way.
  rearmed_us_ = NowUs();
}

bool Sx1281::Start()
{
  hw->nreset.Write(false);
  LibXR::Thread::Sleep(2);
  hw->nreset.Write(true);
  LibXR::Thread::Sleep(20);

  // 14.1.1 / 14.3.1: STDBY_RC, packet type first, then frequency, buffer, modulation and
  // packet parameters, sync word, Tx parameters, IRQ routing.
  const ModemParams& m = gfsk_ ? kGfsk : kFlrc;
  const uint32_t step = static_cast<uint32_t>(
      (static_cast<uint64_t>(identity_.frequency_hz) << 18) / 52'000'000ULL);
  bool ok = Command({kSetStandby, 0x00}) && Command({kSetPacketType, m.packet_type}) &&
            Command({kSetRfFrequency, static_cast<uint8_t>(step >> 16),
                     static_cast<uint8_t>(step >> 8), static_cast<uint8_t>(step)}) &&
            Command({kSetBufferBase, kTxBase, rx_base_}) &&
            Command({kSetModulation, m.modulation[0], m.modulation[1], m.modulation[2]}) &&
            Command({kSetPacketParams, kPreamble, m.sync_length, m.sync_match,
                     kVariableLength, MaxPayload(), m.crc, m.whitening}) &&
            (!m.sync_used || WriteSyncWord()) &&
            Command({kWriteRegister, kCrcSeedReg >> 8, kCrcSeedReg & 0xFF,
                     static_cast<uint8_t>(identity_.crc_seed >> 8),
                     static_cast<uint8_t>(identity_.crc_seed)}) &&
            Command({kSetTxParams, static_cast<uint8_t>(kPowerDbm + 18), XRDAP_RADIO_RAMP}) &&
            Command({kSetDioIrq, kIrqMask >> 8, kIrqMask & 0xFF, kIrqMask >> 8,
                     kIrqMask & 0xFF, 0, 0, 0, 0}) &&
            Command({kSetAutoFs, kAutoFs ? uint8_t{0x01} : uint8_t{0x00}});
  ClearIrq();
  Receive();
  heard_us_ = NowUs();
  return ok;
}

uint8_t Sx1281::Load(Fill fill, void* ctx, uint8_t index, uint8_t base)
{
  static uint8_t frame[2 + kMaxPayload];
  const uint8_t size = fill(ctx, index, frame + 2);
  frame[0] = kWriteBuffer;
  frame[1] = base;
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
  // Leave receive for FS (with AutoFs, 10.8: FS to Tx 27 us) or STDBY_XOSC.
  if (kAutoFs)
    Command({kSetFs});
  else
    Command({kSetStandby, kStandbyXosc});
  ClearIrq();  // 11.6.4: clear IRQs before SetTx
  prepared_ = true;
}

bool Sx1281::TransmitBurst(uint8_t count, Fill fill, void* ctx)
{
  if (!prepared_) PrepareTransmit();
  prepared_ = false;
  hw->rxen.Write(false);
  hw->txen.Write(true);

  uint8_t base = kTxBase;
  uint8_t size = Load(fill, ctx, 0, base);
  bool all_sent = true;
  for (uint8_t i = 0; i < count; ++i)
  {
    if (kTxPingPong) Command({kSetBufferBase, base, rx_base_});
    SetPacketParams(size);
    Command({kSetTx, 0x00, 0x00, 0x00});
    const bool more = i + 1U < count;
    const uint8_t next_base = kTxPingPong ? static_cast<uint8_t>(base ^ kHalf) : base;
    uint8_t next_size = 0;
    // Section 8: the buffer is accessible while the current packet is on the air.
    if (more && kTxPingPong) next_size = Load(fill, ctx, static_cast<uint8_t>(i + 1U), next_base);
    uint32_t waited = 0;
    const bool sent = WaitLevel(fast_dio1, true, kTxTimeoutUs, waited);
    tx_wait_us += waited;
    ClearIrq();
    if (sent)
      ++tx_packets;
    else
      ++tx_timeouts;
    all_sent = all_sent && sent;
    if (more && !kTxPingPong) next_size = Load(fill, ctx, static_cast<uint8_t>(i + 1U), next_base);
    base = next_base;
    size = next_size;
  }
  if (kTxPingPong) Command({kSetBufferBase, kTxBase, rx_base_});
  // Receive accepts any length up to the maximum (variable length filters longer ones).
  SetPacketParams(MaxPayload());
  Receive();
  return all_sent;
}

void Sx1281::Recover()
{
  const uint32_t now = NowUs();
  if (now - heard_us_ >= kRestartUs)
  {
    ++restarts;
    Start();
  }
  else if (!kRxSingle && now - rearmed_us_ >= kRearmUs)
  {
    ++rearms;
    Command({kSetStandby, kStandbyXosc});
    ClearIrq();
    Receive();
  }
}

bool Sx1281::PacketBad(uint16_t irq_flags)
{
  if (!kCrcFromStatus) return (irq_flags & kIrqCrcError) != 0U;
  // Errata 16.3: check the error byte (packetStatus[23:16], table 11-68).
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
  uint16_t flags = kIrqRxDone;
  if (kRxSingle || !kIrqFromPin || !kCrcFromStatus)
  {
    uint8_t irq[4] = {kGetIrq, 0, 0, 0};
    Transfer(irq, irq, sizeof(irq));
    flags = static_cast<uint16_t>(irq[2] << 8 | irq[3]);
  }
  ClearIrq();
  if ((flags & kIrqRxDone) == 0U)
  {
    // An Rx timeout (single receive) or a stray flag: listen again.
    if ((flags & kIrqTimeout) != 0U) ++rearms;
    if (kRxSingle) Receive();
    return 0;
  }
  if (PacketBad(flags))
  {
    ++crc_errors;
    if (kRxSingle) Receive();
    return -1;
  }

  uint8_t status[4] = {kGetRxBufferStatus, 0, 0, 0};
  Transfer(status, status, sizeof(status));
  const uint8_t size = status[2], offset = status[3];
  if (size == 0U || size > MaxPayload())
  {
    if (kRxSingle) Receive();
    return -1;
  }
  if (kRxSingle)
  {
    // Receive again at once, into the other half (8.4), then read this packet out.
    rx_base_ = static_cast<uint8_t>(rx_base_ ^ kHalf);
    Command({kSetBufferBase, kTxBase, rx_base_});
    Receive();
  }
  static uint8_t frame[3 + kMaxPayload];
  frame[0] = kReadBuffer;
  frame[1] = offset;
  frame[2] = 0;
  for (uint8_t i = 0; i < size; ++i) frame[3 + i] = 0;
  Transfer(frame, frame, static_cast<uint16_t>(3U + size));
  for (uint8_t i = 0; i < size; ++i) data[i] = frame[3 + i];
  ++rx_packets;
  return size;
}

uint8_t Sx1281::Pins()
{
  return static_cast<uint8_t>((hw->busy.Read() ? 1U : 0U) | (hw->dio1.Read() ? 2U : 0U) |
                              (hw->dio2.Read() ? 4U : 0U) | (hw->dio3.Read() ? 8U : 0U));
}

void Sx1281::SetOutputs(uint8_t mask, uint8_t value)
{
  if (mask & 1U) hw->nreset.Write((value & 1U) != 0U);
  if (mask & 2U) hw->txen.Write((value & 2U) != 0U);
  if (mask & 4U) hw->rxen.Write((value & 4U) != 0U);
}

bool Sx1281::WaitDio1(uint32_t timeout_us, uint32_t& waited_us)
{
  return WaitLevel(fast_dio1, true, timeout_us, waited_us);
}
}  // namespace XrDap
