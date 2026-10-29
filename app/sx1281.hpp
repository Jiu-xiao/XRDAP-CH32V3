// SPDX-License-Identifier: Apache-2.0
#pragma once

// SX1281 for the radio link, in one of two modulations chosen at run time: GFSK at
// 2 Mb/s or FLRC at 1.3 Mb/s; variable-length packets with a 2-byte CRC, on the
// VG2392S240X0M2 module whose RF switch is driven by TXEN/RXEN. A pair of probes is told
// apart from other pairs by its identity: frequency, CRC seed and (GFSK) sync word.
// References are to the SX1280/SX1281 datasheet DS_SX1280-1 rev 3.3.
//
// The radio receives continuously and stays in FS between transmit and receive
// (SetAutoFs, 11.6.12). Only RxDone and CrcError reach DIO1 while receiving, so a high
// DIO1 means a packet; the error byte of GetPacketStatus decides about the CRC (errata
// 16.3). Continuous receive can stop hearing packets (errata 16.1): measured on a wdap
// pair, the receiver flags PreambleDetected, finds no sync word and then hears nothing,
// not even bad packets, until it enters receive again. PreambleDetected and SyncWordValid
// are therefore latched in the IRQ status (not on DIO1); Poll() looks every 1 ms and
// enters receive again when a preamble has had no sync word for 2 ms (a whole packet
// takes at most about 1.3 ms). Poll() also enters receive again after 100 ms without a
// packet and resets the radio after 2 s.
//
// Context: one owner thread (the application's main loop) calls every member; nothing
// runs in an interrupt. Calls block for at most a few milliseconds (BUSY, TX done);
// StartBurst() and ContinueBurst() do not wait for TX done, so the owner can do other
// work while a packet is on air.
//
// 用于无线链路的 SX1281，调制方式运行时二选一：GFSK 2 Mb/s 或 FLRC 1.3 Mb/s；变长包，
// 2 字节 CRC；VG2392S240X0M2 模块的射频开关由 TXEN/RXEN 控制。一对探针靠身份（频点、
// CRC 种子、GFSK 同步字）与其他对区分。引用的章节均指 SX1280/SX1281 数据手册
// DS_SX1280-1 rev 3.3。
//
// 射频连续接收，收发之间停在 FS（SetAutoFs，11.6.12）。接收时只有 RxDone 和 CrcError
// 接到 DIO1，DIO1 为高即有包；CRC 由 GetPacketStatus 的错误字节判断（勘误 16.3）。连续
// 接收可能收不到包（勘误 16.1）：在一对 wdap 上实测，接收机报出 PreambleDetected、找不到
// 同步字，之后什么都收不到（连坏包都没有），直到重新进入接收。所以 PreambleDetected 和
// SyncWordValid 也锁存在中断状态里（不接 DIO1）；Poll() 每 1 ms 看一次，前导码出现 2 ms
// 还没有同步字就重新进入接收（一个完整的包最多约 1.3 ms）。另外 Poll() 100 ms 没收到包
// 时重新进入接收，2 s 时复位射频。
//
// 上下文：所有成员只由一个所有者线程（应用主循环）调用，不在中断里运行。每次调用最多
// 阻塞几毫秒（等 BUSY、等发送完成）；StartBurst() 和 ContinueBurst() 不等发送完成，包在
// 空中时所有者可以做别的事。

#include <cstdint>
#include <initializer_list>

#include "pair_identity.hpp"

namespace XrDap
{
class Sx1281
{
 public:
  /// 两种调制中最大的包长 / Largest packet in either modulation.
  static constexpr uint8_t kMaxPayload = 255;

  /// 一对探针的射频身份（pair_identity.hpp）/ Radio identity of a pair
  /// (pair_identity.hpp).
  using Identity = RadioIdentity;

  /// 填第 index 包并返回其长度 / Fills packet `index` of a burst into `out` and returns
  /// its size.
  using Fill = uint8_t (*)(void* ctx, uint8_t index, uint8_t* out);

  /// @brief 配置引脚和 SPI；Start() 之前射频保持复位 / Pins and SPI; the radio stays in
  ///        reset until Start().
  /// @note 启动时调用一次 / Call once at start-up.
  void Init();

  /// @brief 下次 Start() 用的调制方式和身份 / Modulation and identity for the next
  ///        Start().
  void Configure(bool gfsk, const Identity& identity);

  /// @brief 复位并配置射频，结束时处于接收 / Reset and configure the radio; ends in
  ///        receive.
  /// @return 所有配置命令都被接受时为 true；false 时射频状态不确定，Poll() 在 2 s 无包后
  ///         会再次复位 / true when every configuration command was accepted; on false
  ///         the radio state is undefined and Poll() resets it again after 2 s without a
  ///         packet.
  /// @note 阻塞约 22 ms（复位脉冲和启动）/ Blocks about 22 ms (reset pulse and start-up).
  bool Start();

  /// 当前调制下的最大包长 / Largest packet in the current modulation.
  uint8_t MaxPayload() const { return gfsk_ ? kMaxPayload : 127; }

  /// @brief 提前离开接收、准备发射，让切换与其他工作重叠；发射前调用 Poll() 则重新进入
  ///        接收 / Leave receive for transmit ahead of time, so that the switch overlaps
  ///        other work; a Poll() before the transmit listens again.
  void PrepareTransmit();

  /// @brief 发一包后回到接收 / Send one packet and return to receive.
  /// @return 包发出（TxDone）时为 true；长度无效或 5 ms 内没有 TxDone 时为 false /
  ///         true once the packet went out (TxDone); false for an invalid size or no
  ///         TxDone within 5 ms.
  bool Transmit(const uint8_t* data, uint8_t size);

  /// @brief 连发 count 包后回到接收 / Send `count` packets back to back, then return to
  ///        receive.
  /// @return 每一包都发出时为 true / true when every packet went out.
  bool TransmitBurst(uint8_t count, Fill fill, void* ctx);

  /// @brief 开始连发 count 包：发出第 0 包就返回，其余由 ContinueBurst() 发 / Start
  ///        sending `count` packets: returns once packet 0 is on air; ContinueBurst()
  ///        sends the rest.
  /// @note fill 和 ctx 要保持有效直到 Bursting() 为 false；连发期间不能调用 Poll() /
  ///       fill and ctx must stay valid until Bursting() is false; do not call Poll()
  ///       during the burst.
  void StartBurst(uint8_t count, Fill fill, void* ctx);

  /// @brief 推进连发：空中的包发完（TxDone，或 5 ms 超时）就发下一包，最后一包之后回到
  ///        接收 / Advance the burst: once the packet on air is done (TxDone, or 5 ms
  ///        without it) send the next one, and after the last one return to receive.
  /// @return 连发还没结束时为 true / true while the burst goes on.
  /// @note 不等发送完成 / Does not wait for TX done.
  bool ContinueBurst();

  /// 是否在连发 / Whether a burst is in progress.
  bool Bursting() const { return burst_count_ != 0U; }

  /// 上一次连发是否每一包都发出 / Whether every packet of the last burst went out.
  bool BurstSent() const { return burst_sent_; }

  /// @brief 取一个收到的包 / Fetch one received packet.
  /// @param data 至少 kMaxPayload 字节 / At least kMaxPayload bytes.
  /// @return 包长；没有包时 0；坏包（CRC、长度）时 -1 / The packet size; 0 when none
  ///         arrived; -1 for a bad packet (CRC, length).
  /// @note 除读包的一次 SPI 外不阻塞；顺带执行静默恢复（见文件头）/ Does not block apart
  ///       from reading the packet over SPI; also runs the silence recovery (see the file
  ///       header).
  int Poll(uint8_t* data);

 private:
  /// BUSY 变低后一次 NSS 包围的传输 / One NSS-framed transfer after BUSY is low.
  bool Transfer(const uint8_t* tx, uint8_t* rx, uint16_t size);
  bool Command(std::initializer_list<uint8_t> bytes);
  void Receive();
  void ClearIrq();
  void Recover();
  bool PreambleStuck(uint32_t now);
  bool WriteSyncWord();
  uint8_t Load(Fill fill, void* ctx, uint8_t index);
  bool PacketBad();
  void SetPacketParams(uint8_t size);
  void SendLoaded(uint8_t size);

  Fill burst_fill_ = nullptr;
  void* burst_ctx_ = nullptr;
  uint8_t burst_count_ = 0;  // 0: no burst
  uint8_t burst_index_ = 0;  // packet on air
  uint32_t burst_sent_us_ = 0;
  bool burst_sent_ = true;

  bool prepared_ = false;
  bool gfsk_ = true;
  Identity identity_ = DefaultIdentity();

  uint32_t heard_us_ = 0;     // last packet from the radio, or start
  uint32_t rearmed_us_ = 0;   // last time receive was entered
  uint32_t checked_us_ = 0;   // last look at the IRQ status for a stuck preamble
  uint32_t preamble_us_ = 0;  // when a preamble without a sync word was first seen
  bool preamble_seen_ = false;
};

/// @brief 本板唯一的 SX1281 / The board's SX1281.
/// @note 只由主循环线程使用 / Used by the main loop thread only.
Sx1281& Radio();
}  // namespace XrDap
