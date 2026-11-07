// SPDX-License-Identifier: Apache-2.0 AND MIT
#pragma once

// The CH32V208's own 2.4 GHz radio as the transceiver XRDAP-Core's RadioLink asks for
// (xrdap/radio/transceiver.hpp). Packets of up to 251 bytes go over the air in the BLE
// packet format without any BLE connection: access address, whitening and CRC by the
// hardware. One caller context calls every member and nothing runs in an interrupt:
// Poll() and Receive() look at the status register.
//
// The channels are those of BLE, 2 MHz apart from 2402 to 2480 MHz; a frequency between
// two of them goes to the nearer one, the same on both probes. The high rate is the 2M
// PHY, the low rate the 1M PHY, and the link address is the access address on both.
//
// CH32V208 片上 2.4 GHz 射频，作为 XRDAP-Core 的 RadioLink 要的收发器
// （xrdap/radio/transceiver.hpp）。至多 251 字节的包按 BLE 的包格式上空中，不建立 BLE
// 连接：访问地址、白化和 CRC 由硬件完成。所有成员由同一个调用上下文调用，中断里什么也不
// 做：Poll() 和 Receive() 查状态寄存器。
//
// 信道就是 BLE 的信道，2402 到 2480 MHz，间隔 2 MHz；落在两个信道之间的频率取较近的
// 一个，两块探针结果相同。高速率用 2M PHY，低速率用 1M PHY，两档都以链路地址作访问地址。
//
// The register sequences are derived from extralibs/iSLER.h of ch32fun
// (https://github.com/cnlohr/ch32fun), under this notice:
//
// MIT License
//
// Copyright (c) 2023-2024 CNLohr <lohr85@gmail.com>, et. al.
// Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
// Copyright (c) 2023-2024 E. Brombaugh
// Copyright (c) 2023-2024 A. Mandera
// Copyright (c) 2005-2020 Rich Felker, et al.
// Copyright (c) 2013,2014 Michal Ludvig <michal@logix.cz>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "libxr_def.hpp"
#include "timebase.hpp"
#include "xrdap/radio/transceiver.hpp"
#include "xrdap_chip.h"

namespace XrDap
{

class V208Radio
{
 public:
  /// 最大载荷长度 / Max payload size.
  static constexpr uint8_t MAX_PAYLOAD_SIZE = 251;

  /// @brief 校准射频并进入接收，阻塞几 ms / Calibrate the radio and enter receive; blocks
  ///        for a few ms.
  LibXR::ErrorCode Start(const Radio::LinkParams& params)
  {
    if (started_)
    {
      restarts_ = restarts_ + 1U;
    }
    started_ = true;
    phy_ = params.rate == Radio::Rate::kHigh ? kPhy2M : kPhy1M;
    access_address_ = static_cast<uint32_t>(params.address[0]) << 24 |
                      static_cast<uint32_t>(params.address[1]) << 16 |
                      static_cast<uint32_t>(params.address[2]) << 8 | params.address[3];
    crc_init_ = static_cast<uint32_t>(params.crc_seed) << 8 | (params.crc_seed & 0xFFU);
    channel_ = ChannelNumber(params.frequency_hz);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_BLE_CRC, ENABLE);
    Halt();
    InitHardware();
    EnterReceive();
    return LibXR::ErrorCode::OK;
  }

  /// 当前速率下的最大载荷：1M PHY 最长 127 字节，更长的包在空中的时间会超过主机等第一个
  /// 应答包的时间 / Max payload of the current rate: 127 bytes on the 1M PHY, since a
  /// longer packet stays on the air past the host's wait for the first answer packet.
  uint8_t MaxPayload() const { return phy_ == kPhy1M ? kMaxPayload1M : MAX_PAYLOAD_SIZE; }

  /// 收发切换没有可提前做的事 / The switch to transmit has nothing to do ahead of time.
  void PrepareSend() {}

  /// @brief 发送一包：写入载荷并开始发射就返回。上一包还在发射时返回 BUSY；它发完而
  /// Poll()
  ///        还没转回接收时，这一包直接发出 / Send one packet: load it and start the
  ///        transmission, then return. BUSY while the previous packet is on air; once it
  ///        is out and Poll() has not returned to receive yet, this packet goes straight
  ///        out.
  LibXR::ErrorCode Send(const uint8_t* data, uint8_t size)
  {
    if (size > MaxPayload())
    {
      return LibXR::ErrorCode::OUT_OF_RANGE;
    }
    if (state_ == State::TX && !TxFinished())
    {
      return LibXR::ErrorCode::BUSY;
    }
    tx_[0] = 0U;
    tx_[1] = size;
    std::memcpy(tx_ + 2, data, size);
    // 对方要时间从发送转到接收，等于 BLE 的帧间隔。 / The peer needs time to turn from
    // transmit to receive, the frame spacing of BLE.
    while (NowUs() - last_event_us_ < kTurnaroundUs)
    {
    }
    Halt();
    Configure();
    Arm(kModeTx);
    Bb().ctrl_cfg |= kStartTx;
    Bb().ctrl_tx &= ~0x3U;
    Ll().status = kAllStatus;
    tx_started_us_ = NowUs();
    state_ = State::TX;
    Ll().command = kCommandTx;
    return LibXR::ErrorCode::OK;
  }

  /// 发送中，或发完还没转回接收 / Sending, or done and not back in receive yet.
  bool Sending() const { return state_ == State::TX; }

  /// 发完后转回接收 / Return to receive once a transmission is done.
  void Poll()
  {
    if (state_ == State::TX && TxFinished())
    {
      EnterReceive();
    }
  }

  /// @brief 取一包 / Take one packet.
  /// @param out 至少 MaxPayload() 字节 / At least MaxPayload() bytes.
  /// @return 长度；0 没有包；-1 坏包 / The size; 0 no packet; -1 a bad packet.
  int Receive(uint8_t* out)
  {
    if (state_ != State::RX)
    {
      return 0;
    }
    const uint32_t status = Ll().status;
    if ((status & kStatusRxDone) == 0U)
    {
      return 0;
    }
    const bool crc_ok = (Bb().status & kBbStatusCrcError) == 0U;
    Ll().status = status;
    last_event_us_ = NowUs();
    int result = -1;
    const uint8_t size = rx_[1];
    if (crc_ok && size <= MaxPayload())
    {
      std::memcpy(out, rx_ + 2, size);
      rx_packets_ = rx_packets_ + 1U;
      result = size;
    }
    else
    {
      rx_errors_ = rx_errors_ + 1U;
    }
    Halt();
    EnterReceive();
    return result;
  }

  /// @brief 换频率，留在接收 / Change the frequency and stay in receive.
  LibXR::ErrorCode Tune(uint32_t frequency_hz)
  {
    channel_ = ChannelNumber(frequency_hz);
    Halt();
    EnterReceive();
    return LibXR::ErrorCode::OK;
  }

  /// @brief 瞬时信号强度，给信道扫描比较用，按 RadioLink 的 0.5 dBm 单位给出 / The
  ///        instantaneous signal level for the channel scan to compare, in RadioLink's
  ///        units of 0.5 dBm.
  /// @note 基带状态的第 2 字节安静时最大，信号越强越小，没有定标。取负后，安静时低于
  ///       RadioLink 的忙碌阈值，有信号时高于它。Bb().rssi 留着上一包的值，不随信道变化。
  ///       / Byte 2 of the baseband status is largest in silence and falls as the signal
  ///       grows; it is not calibrated. Negated, silence is below RadioLink's busy
  ///       threshold and a signal above it. Bb().rssi keeps the value of the last packet
  ///       and does not follow the channel.
  int16_t RssiInstant() const
  {
    return -static_cast<int16_t>((Bb().status >> 16) & 0xFFU);
  }

  uint32_t TxPackets() const { return tx_packets_; }  ///< 发出的包 / Packets sent
  uint32_t TxErrors() const { return tx_errors_; }    ///< 发送超时 / Transmit timeouts
  uint32_t RxPackets() const { return rx_packets_; }  ///< 收到的好包 / Good packets
  uint32_t RxErrors() const { return rx_errors_; }    ///< 坏包 / Bad packets
  uint32_t Rearms() const { return 0U; }  ///< 没有停顿恢复 / No stall recovery
  /// 重新校准的次数（不含第一次 Start()）/ Calibrations, not counting the first Start().
  uint32_t Restarts() const { return restarts_; }

  /// @brief BLE 通道号（通道号模式）/ The BLE channel number (channel number mode).
  static constexpr uint8_t ChannelNumber(uint32_t frequency_hz)
  {
    constexpr uint32_t kMhz = 1'000'000U;
    uint32_t mhz = (frequency_hz + kMhz) / (2U * kMhz) * 2U;
    mhz = mhz < 2402U ? 2402U : (mhz > 2480U ? 2480U : mhz);
    if (mhz == 2402U)
    {
      return 37U;
    }
    if (mhz == 2426U)
    {
      return 38U;
    }
    if (mhz == 2480U)
    {
      return 39U;
    }
    return static_cast<uint8_t>(mhz < 2426U ? (mhz - 2404U) / 2U
                                            : (mhz - 2428U) / 2U + 11U);
  }

 private:
  // 基带 / Baseband.
  struct BbRegs
  {
    volatile uint32_t ctrl_cfg;  // +0x00 通道、白化、PHY、方向 / channel, PHY, direction
    volatile uint32_t crc_init;  // +0x04
    volatile uint32_t access_address;  // +0x08
    volatile uint32_t reserved_0c;
    volatile uint32_t demod[7];  // +0x10..0x28
    volatile uint32_t ctrl_tx;   // +0x2C 发射功率 / TX power
    volatile uint32_t rssi;      // +0x30
    volatile uint32_t cfg_34;    // +0x34
    volatile uint32_t status;    // +0x38
  };
  static_assert(offsetof(BbRegs, ctrl_tx) == 0x2C && offsetof(BbRegs, status) == 0x38);

  // 链路层：命令、状态和收发 DMA / Link layer: command, status and the TX/RX DMA.
  struct LlRegs
  {
    volatile uint32_t command;     // +0x00
    volatile uint32_t config;      // +0x04
    volatile uint32_t status;      // +0x08 写 1 清除 / write 1 to clear
    volatile uint32_t irq_enable;  // +0x0C
    volatile uint32_t reserved_10[16];
    volatile uint32_t mode;  // +0x50
    volatile uint32_t reserved_54[4];
    volatile uint32_t timer;  // +0x64 倒计数，2 MHz / counts down at 2 MHz
    volatile uint32_t reserved_68[2];
    volatile uint32_t tx_buffer;  // +0x70
    volatile uint32_t rx_buffer;  // +0x74
  };
  static_assert(offsetof(LlRegs, mode) == 0x50 && offsetof(LlRegs, rx_buffer) == 0x74);

  // 射频前端的寄存器，按 4 字节的序号 / Radio front end registers, by 4-byte index.
  struct RfRegs
  {
    volatile uint32_t r[64];
  };

  static BbRegs& Bb() { return *reinterpret_cast<BbRegs*>(0x40024100U); }
  static LlRegs& Ll() { return *reinterpret_cast<LlRegs*>(0x40024200U); }
  static RfRegs& Rf() { return *reinterpret_cast<RfRegs*>(0x40025000U); }

  enum class State : uint8_t
  {
    STOPPED,
    RX,
    TX
  };

  static constexpr uint32_t kCommandRx = 0x01U;
  static constexpr uint32_t kCommandTx = 0x02U;
  static constexpr uint32_t kCommandStop = 0x08U;
  static constexpr uint32_t kStatusRxDone = 0x01U;
  static constexpr uint32_t kStatusTxDone = 0x02U;
  static constexpr uint32_t kAllStatus = 0xFFFFFFFFU;
  static constexpr uint32_t kBbStatusCrcError = 0x20U;
  static constexpr uint32_t kBbStatusClear = 0xF0U;
  static constexpr uint32_t kModeTx = 0x5AU;
  static constexpr uint32_t kModeRx = 0x59U;
  static constexpr uint32_t kModeTune = 0x5DU;
  static constexpr uint32_t kStartTx = 0x00800000U;
  static constexpr uint32_t kPhy2M = 0U;
  static constexpr uint32_t kPhy1M = 0x1000U;
  // 发射功率 0 dBm，同 WCH 库的 LL_TX_POWEER_0_DBM。 / Transmit power 0 dBm, as WCH's
  // LL_TX_POWEER_0_DBM.
  static constexpr uint32_t kTxPowerCode = 0x09U;
  static constexpr uint8_t kMaxPayload1M = 127U;
  static constexpr uint32_t kTurnaroundUs = 150U;
  static constexpr uint32_t kTxTimeoutUs = 5'000U;
  // 校准等待，定时器 2 MHz / Calibration waits, timer at 2 MHz.
  static constexpr uint32_t kTuneClearTicks = 100U;
  static constexpr uint32_t kTuneTicks = 8'000U;
  static constexpr uint32_t kFilterPulseTicks = 8U;
  static constexpr uint32_t kFilterTicks = 2'000U;

  // 前端寄存器的序号 / Indices of the front end registers.
  static constexpr size_t kRfTune = 1;
  static constexpr size_t kRfPower = 2;
  static constexpr size_t kRfFilter = 3;
  static constexpr size_t kRf10 = 10;
  static constexpr size_t kRf11 = 11;
  static constexpr size_t kRf15 = 15;
  static constexpr size_t kRfBand = 14;
  static constexpr size_t kRfRxFilterValue = 20;
  static constexpr size_t kRfRxAdc = 22;
  static constexpr size_t kRfCo = 36;
  static constexpr size_t kRfGa = 37;
  static constexpr size_t kRfFilterResult = 39;
  static constexpr size_t kRfCoTable = 40;
  static constexpr size_t kRfCoTableWords = 10;

  static uint32_t NowUs()
  {
    return static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds());
  }

  // 停掉正在进行的收发。 / Stop what is running.
  static void Halt()
  {
    Ll().mode &= ~0x7U;
    Ll().command |= kCommandStop;
    Ll().timer = 0U;
  }

  void Arm(uint32_t mode)
  {
    Bb().ctrl_cfg = (Bb().ctrl_cfg & ~0x180U) | 0x100U;
    Rf().r[kRfPower] |= 0x330000U;
    Ll().mode = mode;
  }

  void Configure()
  {
    Rf().r[kRf11] &= ~0x2U;  // 通道号模式 / channel number mode
    Bb().ctrl_cfg = (Bb().ctrl_cfg & 0xFFFF0F80U) | phy_ | channel_;
    Bb().access_address = access_address_;
    Bb().crc_init = crc_init_;
    Bb().demod[0] = 0x003222D0U;
    Bb().demod[1] = 0x08101901U;
    Bb().demod[2] = 0x00031624U;
    Bb().demod[4] = 0x00090083U;
    Bb().demod[5] = 0x01006310U;
    Bb().demod[6] = 0x000028BEU;
    Ll().tx_buffer = reinterpret_cast<uintptr_t>(tx_);
    Ll().rx_buffer = reinterpret_cast<uintptr_t>(rx_);
    Ll().config &= ~0x1U;  // 手动收发，不自动应答 / basic mode, no automatic answer
  }

  void EnterReceive()
  {
    Configure();
    Arm(kModeRx);
    Bb().ctrl_tx |= 0x1U;
    Bb().status = kBbStatusClear;
    Ll().status = kAllStatus;
    state_ = State::RX;
    Ll().command = kCommandRx;
  }

  // 发完（或超时）时为 true。 / True once the packet is out, or timed out.
  bool TxFinished()
  {
    const uint32_t status = Ll().status;
    if ((status & kStatusTxDone) != 0U)
    {
      Ll().status = status;
      tx_packets_ = tx_packets_ + 1U;
    }
    else if (NowUs() - tx_started_us_ >= kTxTimeoutUs)
    {
      tx_errors_ = tx_errors_ + 1U;
      Halt();
    }
    else
    {
      return false;
    }
    last_event_us_ = NowUs();
    state_ = State::STOPPED;
    return true;
  }

  void InitHardware()
  {
    // 32 MHz 晶振的负载电容和电流，同 WCH 的 WCHBLE_Init() / Load capacitance and current
    // of the 32 MHz crystal, as in WCH's WCHBLE_Init().
    OSC->HSE_CAL_CTRL &= ~(0x07UL << 28);
    OSC->HSE_CAL_CTRL |= 0x03UL << 28;
    OSC->HSE_CAL_CTRL |= 3UL << 24;

    Ll().irq_enable = 0xF00FU;
    Ll().rx_buffer = reinterpret_cast<uintptr_t>(rx_);
    Ll().status = kAllStatus;
    Rf().r[kRf10] = 0x480U;
    // 这个字段是复位值时发射端在 2432 MHz 上发不出能收到的包。 / With this field at its
    // reset value the transmitter sends nothing receivable on 2432 MHz.
    Rf().r[kRf15] = (Rf().r[kRf15] & 0x1FFFFFFFU) | 0x40000000U;

    Bb().ctrl_cfg |= 0x00800000U;
    Bb().ctrl_cfg |= 0x10000000U;
    Bb().cfg_34 = 0x1D0U;
    Bb().ctrl_tx = 0x80010EC8U | kTxPowerCode << 25;
    Bb().demod[4] = 0x00090083U;

    // 校准期间用 1M PHY。 / The 1M PHY during the calibration.
    Bb().ctrl_cfg = (Bb().ctrl_cfg & 0xFFFF0FFFU) | kPhy1M;
    Arm(kModeTune);
    TuneTx();
    TuneRx();
    Bb().ctrl_cfg = (Bb().ctrl_cfg & ~0x180U) | 0x80U;
    Rf().r[kRfPower] &= 0xFFCDFFFFU;
    Ll().mode = 0U;
    Ll().timer = 0U;
  }

  static bool BandDone() { return (Rf().r[kRfCo] & 0x06000000U) == 0x06000000U; }

  // 一个频段的 VCO 校准，返回电容值（低字节）和增益（高字节）。 / VCO calibration of one
  // band; returns the capacitor value (low byte) and the gain (next byte).
  static uint32_t TuneBand(uint32_t band)
  {
    Rf().r[kRfTune] &= ~0x1U;
    Rf().r[kRfBand] = (Rf().r[kRfBand] & 0xFFFE00FFU) | band << 8;
    Rf().r[kRfTune] |= 0x1U;
    // 上一个频段的完成位不会马上清掉，先等它清掉，否则读到的是上一个频段的结果。 / The
    // done bits of the previous band take a moment to clear; reading before that gives
    // the previous band's result.
    Ll().timer = kTuneClearTicks;
    while (BandDone() && Ll().timer != 0U)
    {
    }
    Ll().timer = kTuneTicks;
    while (!BandDone() && Ll().timer != 0U)
    {
    }
    return (Rf().r[kRfCo] & 0x3FU) | ((Rf().r[kRfGa] >> 10) & 0x7FU) << 8;
  }

  static void TuneTx()
  {
    Rf().r[kRfTune] &= ~0x100U;
    Rf().r[kRf10] &= ~0x1000U;
    Rf().r[kRf11] &= ~0x10U;
    Rf().r[kRfPower] |= 0x20000U;
    Rf().r[kRfTune] |= 0x10U;

    const uint32_t low = TuneBand(0xBFU);   // 2402 MHz
    const uint32_t high = TuneBand(0xE7U);  // 2480 MHz
    const uint32_t mid = TuneBand(0xD3U);   // 2440 MHz

    // 每个通道相对中间频段的电容修正，每字节两个通道。 / The capacitor correction of each
    // channel against the middle band, two channels to a byte.
    const int co_low = static_cast<int>(low & 0x3FU) - static_cast<int>(mid & 0x3FU);
    const int co_high = static_cast<int>(mid & 0x3FU) - static_cast<int>(high & 0x3FU);
    constexpr int kHalf = static_cast<int>(kRfCoTableWords * 4U / 2U);
    for (size_t word = 0; word < kRfCoTableWords; ++word)
    {
      uint32_t value = 0U;
      for (int byte = 0; byte < 4; ++byte)
      {
        const int f = static_cast<int>(word) * 4 + byte;
        const int tune =
            f < kHalf ? co_low * (kHalf - f) / kHalf : co_high * (f - kHalf) / kHalf;
        const uint32_t nibble = static_cast<uint32_t>(tune) & 0xFU;
        value |= (nibble | nibble << 4) << (byte * 8);
      }
      Rf().r[kRfCoTable + word] = value;
    }

    Rf().r[kRfTune] &= ~0x11U;
    Rf().r[kRf10] |= 0x1000U;
    Rf().r[kRf11] |= 0x10U;
    Rf().r[kRfBand] = (Rf().r[kRfBand] & 0xFFFFFFC0U) | (mid & 0x3FU);
    Rf().r[kRfBand] = (Rf().r[kRfBand] & 0x80FFFFFFU) | ((mid >> 8) & 0x7FU) << 24;
    Rf().r[kRfTune] |= 0x100U;
  }

  static void TuneRx()
  {
    Rf().r[kRfRxFilterValue] &= ~0x10000U;
    Rf().r[kRfPower] |= 0x200000U;
    Rf().r[kRfFilter] &= ~0x10U;
    Ll().timer = kFilterPulseTicks;
    while (Ll().timer != 0U)
    {
    }
    Rf().r[kRfFilter] |= 0x10U;
    Rf().r[kRfTune] |= 0x1000U;
    Ll().timer = kFilterTicks;
    while ((Rf().r[kRfFilterResult] & 0x100U) == 0U && Ll().timer != 0U)
    {
    }
    const uint32_t filter = Rf().r[kRfFilterResult] & 0x1FU;
    Rf().r[kRfRxFilterValue] |= 0x10000U;
    Rf().r[kRfRxFilterValue] = (Rf().r[kRfRxFilterValue] & 0xFFFFFFE0U) | filter;
    Rf().r[kRfPower] &= ~0x200000U;

    Rf().r[kRfRxAdc] &= ~0x10000U;
    Rf().r[kRfPower] |= 0x10000U;
    Rf().r[kRfFilter] &= ~0x100U;
    Rf().r[kRfTune] &= ~0x10000U;
    Rf().r[kRfFilter] |= 0x100U;
    Rf().r[kRfTune] |= 0x10000U;
  }

  State state_ = State::STOPPED;
  bool started_ = false;
  uint32_t phy_ = kPhy2M;
  uint32_t access_address_ = 0U;
  uint32_t crc_init_ = 0U;
  uint8_t channel_ = 0U;
  uint32_t tx_started_us_ = 0U;
  uint32_t last_event_us_ = 0U;
  uint32_t tx_packets_ = 0U;
  uint32_t tx_errors_ = 0U;
  uint32_t rx_packets_ = 0U;
  uint32_t rx_errors_ = 0U;
  uint32_t restarts_ = 0U;
  alignas(4) uint8_t tx_[2 + MAX_PAYLOAD_SIZE] = {};
  alignas(4) uint8_t rx_[272] = {};
};

static_assert(V208Radio::ChannelNumber(2402000000U) == 37U);
static_assert(V208Radio::ChannelNumber(2404000000U) == 0U);
static_assert(V208Radio::ChannelNumber(2424000000U) == 10U);
static_assert(V208Radio::ChannelNumber(2426000000U) == 38U);
static_assert(V208Radio::ChannelNumber(2428000000U) == 11U);
static_assert(V208Radio::ChannelNumber(2430000000U) == 12U);
static_assert(V208Radio::ChannelNumber(2432000000U) == 13U);
static_assert(V208Radio::ChannelNumber(2434000000U) == 14U);
static_assert(V208Radio::ChannelNumber(2448500000U) == 21U);
static_assert(V208Radio::ChannelNumber(2478000000U) == 36U);
static_assert(V208Radio::ChannelNumber(2480000000U) == 39U);
static_assert(V208Radio::ChannelNumber(2481500000U) == 39U);

}  // namespace XrDap
