// SPDX-License-Identifier: Apache-2.0
#pragma once

// SX1281 for the radio link, in one of two modulations chosen at run time: GFSK at
// 2 Mb/s or FLRC at 1.3 Mb/s; variable-length packets with a 2-byte CRC, on the
// VG2392S240X0M2 module whose RF switch is driven by TXEN/RXEN. A pair of probes is told
// apart from other pairs by its identity: frequency, CRC seed and (GFSK) sync word. The
// owner thread drives it; nothing here runs in an interrupt. References are to the
// SX1280/SX1281 datasheet DS_SX1280-1 rev 3.3.
//
// 用于无线链路的 SX1281，调制方式运行时二选一：GFSK 2 Mb/s 或 FLRC 1.3 Mb/s；变长包，
// 2 字节 CRC；VG2392S240X0M2 模块的射频开关由 TXEN/RXEN 控制。一对探针靠身份（频点、
// CRC 种子、GFSK 同步字）与其他对区分。只在所有者线程里驱动，不在中断里运行。引用的
// 章节均指 SX1280/SX1281 数据手册 DS_SX1280-1 rev 3.3。

#include <cstdint>
#include <initializer_list>

// Build switches. Defaults come from A/B runs on a wdap pair (GFSK, 1 m):
// XRDAP_RADIO_RX_SINGLE    0: single receive with a timeout, entered again after every
//                          packet and timeout, as errata 16.1 recommends against the
//                          continuous-receive lock-up; after a packet receive restarts in the
//                          other half of the buffer before the packet is read (8.2, 8.4), so
//                          packets shrink to 127 bytes. Measured slower (1 MiB write 72 vs
//                          85 KiB/s); continuous receive relies on Poll()'s silence recovery.
// XRDAP_RADIO_CRC_STATUS   1: judge a packet by the error byte of GetPacketStatus (errata
//                          16.3: RxDone does not filter packets with CRC errors).
// XRDAP_RADIO_FLRC_SYNC    1: FLRC with the 32-bit sync word. Without it the receiver takes
//                          noise for packets and the link stops (27,538 CRC errors, no
//                          command through). Errata 16.4 lists forbidden sync words for
//                          coding rates 1/2 and 3/4 ("XRDP" is allowed); for coding rate 1
//                          they are not published.
// XRDAP_RADIO_AUTOFS       1: stay in FS after Tx/Rx (11.6.12): 1 MiB write 72 vs 67 KiB/s,
//                          4-byte reads 2.1 vs 1.6 KiB/s.
// XRDAP_RADIO_TX_PINGPONG  0: write the next packet into the other half of the buffer
//                          while the current one is sent (section 8: the buffer is accessible
//                          in every mode but sleep); limits packets to 127 bytes and measured
//                          no faster than 255-byte GFSK packets.
// The round trip of a small command, measured with DAP vendor command 0x9F 0x17 on a wdap
// pair (GFSK), went from 775 to about 600 us with the next three switches and
// PrepareTransmit() (4-byte reads 2.4 -> 3.1 KiB/s):
// XRDAP_RADIO_SPI_DMA_MIN  24: SPI transfers up to this size go byte by byte through the
//                          data register instead of DMA (9.8 -> 6.4 us per transfer).
// XRDAP_RADIO_RAMP         0x60: PA ramp time code of SetTxParams (table 11-49), 8 us
//                          instead of 20 us (0xE0); 4 us saved 35 us per transmit.
// XRDAP_RADIO_IRQ_FROM_PIN 1: in continuous receive only RxDone and CrcError reach DIO1,
//                          so a high DIO1 means a packet and GetIrqStatus is skipped (the
//                          packet status byte still decides about the CRC, errata 16.3).
#ifndef XRDAP_RADIO_SPI_DMA_MIN
#define XRDAP_RADIO_SPI_DMA_MIN 24
#endif
#ifndef XRDAP_RADIO_RAMP
#define XRDAP_RADIO_RAMP 0x60
#endif
#ifndef XRDAP_RADIO_IRQ_FROM_PIN
#define XRDAP_RADIO_IRQ_FROM_PIN 1
#endif
#ifndef XRDAP_RADIO_RX_SINGLE
#define XRDAP_RADIO_RX_SINGLE 0
#endif
#ifndef XRDAP_RADIO_CRC_STATUS
#define XRDAP_RADIO_CRC_STATUS 1
#endif
#ifndef XRDAP_RADIO_FLRC_SYNC
#define XRDAP_RADIO_FLRC_SYNC 1
#endif
#ifndef XRDAP_RADIO_AUTOFS
#define XRDAP_RADIO_AUTOFS 1
#endif
#ifndef XRDAP_RADIO_TX_PINGPONG
#define XRDAP_RADIO_TX_PINGPONG 0
#endif

namespace XrDap
{
class Sx1281
{
 public:
  static constexpr bool kTxPingPong = XRDAP_RADIO_TX_PINGPONG != 0;
  static constexpr bool kRxSingle = XRDAP_RADIO_RX_SINGLE != 0;
  /// Largest packet in either modulation; packets fit half of the 256-byte buffer when
  /// either side alternates halves / 两种调制中最大的包长。
  static constexpr uint8_t kMaxPayload = (kTxPingPong || kRxSingle) ? 127 : 255;

  /// What tells one pair of probes from another / 区分不同探针对的参数。
  struct Identity
  {
    uint32_t frequency_hz;
    uint16_t crc_seed;
    uint8_t sync_word[4];  ///< GFSK only; FLRC keeps "XRDP" (errata 16.4)
  };
  /// The identity of unpaired probes: 2.44 GHz, "XRDP", the reset CRC seed 0xFFFF.
  static Identity DefaultIdentity();

  /// Fills packet `index` of a burst into `out` and returns its size / 填第 index 包。
  using Fill = uint8_t (*)(void* ctx, uint8_t index, uint8_t* out);

  /// Pins and SPI; the radio stays in reset until Start() / 引脚和 SPI，Start() 前保持复位。
  void Init();

  /// Modulation and identity for the next Start() / 下次 Start() 用的调制方式和身份。
  void Configure(bool gfsk, const Identity& identity);
  /// Reset and configure the radio; ends in receive / 复位并配置射频，结束时处于接收。
  bool Start();
  /// Largest packet in the current modulation / 当前调制下的最大包长。
  uint8_t MaxPayload() const { return gfsk_ ? kMaxPayload : 127; }
  bool Gfsk() const { return gfsk_; }

  /// Leave receive for transmit ahead of time, so that the switch overlaps other work;
  /// a Poll() before the transmit listens again / 提前离开接收、准备发射，让切换与
  /// 其他工作重叠；发射前调用 Poll() 则重新进入接收。
  void PrepareTransmit();
  /// Send one packet and return to receive / 发一包后回到接收。
  bool Transmit(const uint8_t* data, uint8_t size);
  /// Send `count` packets back to back, then return to receive / 连发多包后回到接收。
  bool TransmitBurst(uint8_t count, Fill fill, void* ctx);

  /// A received packet: its size, 0 when none arrived, -1 for a bad packet / 收到一包时
  /// 返回长度，没有包时 0，坏包时 -1。
  ///
  /// When the radio reports nothing at all for a while (no packet and, in single
  /// receive, no timeout either) Poll() enters receive again, and resets the radio if
  /// that does not help.
  /// 射频一段时间什么也不报告（没有包，单次接收时也没有超时）时，Poll() 重新进入接收；
  /// 仍无反应则复位射频。
  int Poll(uint8_t* data);

  /// One NSS-framed transfer after BUSY is low / BUSY 变低后一次 NSS 包围的传输。
  bool Transfer(const uint8_t* tx, uint8_t* rx, uint16_t size);

  /// BUSY, DIO1, DIO2, DIO3 as bits 0..3 / BUSY、DIO1、DIO2、DIO3 分别为位 0..3。
  uint8_t Pins();
  /// bit0 NRESET, bit1 TXEN, bit2 RXEN.
  void SetOutputs(uint8_t mask, uint8_t value);
  /// Microseconds until DIO1 went high, or false on timeout / DIO1 变高前的微秒数。
  bool WaitDio1(uint32_t timeout_us, uint32_t& waited_us);

  uint32_t tx_packets = 0, rx_packets = 0, crc_errors = 0, tx_timeouts = 0;
  /// Receive re-entries after silence or an Rx timeout, radio resets after silence /
  /// 静默或接收超时后重新进入接收的次数，静默后复位射频的次数。
  uint32_t rearms = 0, restarts = 0;
  /// Timing: SPI transfers, the time spent in them and waiting for BUSY before them,
  /// time waiting for TX done /
  /// 计时：SPI 传输次数、耗时与之前等 BUSY 的时间，等待发送完成的时间。
  uint32_t spi_transfers = 0, spi_us = 0, busy_us = 0, tx_wait_us = 0;

 private:
  bool Command(std::initializer_list<uint8_t> bytes);
  void Receive();
  void ClearIrq();
  void Recover();
  bool WriteSyncWord();
  uint8_t Load(Fill fill, void* ctx, uint8_t index, uint8_t base);
  bool PacketBad(uint16_t irq_flags);
  void SetPacketParams(uint8_t size);

  bool prepared_ = false;
  bool gfsk_ = true;
  Identity identity_ = DefaultIdentity();

  uint8_t rx_base_ = 0;      // half the next received packet lands in (single receive)
  uint32_t heard_us_ = 0;    // last report from the radio (packet or timeout), or start
  uint32_t rearmed_us_ = 0;  // last time receive was entered
};

Sx1281& Radio();
}  // namespace XrDap
