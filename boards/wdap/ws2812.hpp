// SPDX-License-Identifier: Apache-2.0
#pragma once

// A WS2812B chain on an SPI MOSI line. Every data bit of the chain becomes a run of SPI
// bits: a high head of t0h or t1h SPI bits, then low up to the bit period; the SPI clock
// is set to at most 18 MHz and the run lengths follow from it (WS2812B-2020 datasheet:
// T0H 220..380 ns, T1H 580 ns..1 µs, T0L 580 ns..1 µs, T1L >= 580 ns, bit >= 1.25 µs).
// A frame goes out by DMA; the line stays low between frames, which is the reset.
//
// SPI MOSI 线上的 WS2812B 灯串。灯串的每个数据位变成一串 SPI 位：
// 先是 t0h 或 t1h 个高电平 SPI 位，其余为低，直到位周期结束；
// SPI 时钟设为不超过 18 MHz，位串长度由它算出（WS2812B-2020 数据手册：
// T0H 220..380 ns，T1H 580 ns..1 µs，T0L 580 ns..1 µs，T1L >= 580 ns，
// 位周期 >= 1.25 µs）。一帧经 DMA 发出；帧间线保持低电平，即复位。

#include <cstddef>
#include <cstdint>

#include "spi.hpp"
#include "xrdap/light/status_light.hpp"

namespace XrDap
{

class Ws2812
{
 public:
  static constexpr uint8_t kMaxLeds = 8;
  /// 18 MHz 下一个数据位最多的 SPI 位数 / Most SPI bits per data bit at 18 MHz.
  static constexpr uint8_t kMaxSpiBits = 23;
  /// 一帧的字节数：前后各一个低电平字节 / Bytes of a frame: one low byte before and
  /// after.
  static constexpr size_t kFrameBytes = 2 + (kMaxLeds * 24 * kMaxSpiBits + 7) / 8;

  /// @brief 配置 SPI 时钟并算出位串长度 / Set the SPI clock and work out the runs.
  explicit Ws2812(LibXR::SPI& spi);

  /// @brief 发出一帧 / Send one frame.
  /// @return OK；上一帧还在发时为 BUSY / OK; BUSY while the previous frame is still
  ///         going out.
  LibXR::ErrorCode Show(const Light::Rgb* leds, uint8_t count);

 private:
  void PutByte(uint8_t value, size_t& bit);

  LibXR::SPI& spi_;
  LibXR::SPI::OperationRW op_;
  uint8_t bits_per_bit_ = kMaxSpiBits;
  uint8_t t0h_ = 0;
  uint8_t t1h_ = 0;
  uint8_t frame_[kFrameBytes] = {};
};

}  // namespace XrDap
