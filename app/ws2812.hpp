// SPDX-License-Identifier: Apache-2.0
#pragma once
// WS2812B chain on TIM1_CH3N (PB15, default mapping, CH32V307 datasheet pin table).
// TIM1 runs a PWM period of 1.28 us and DMA1 channel 5 (TIM1_UP request, RM table 11-2)
// loads one compare value per bit at each update event: 0.30 us high for a 0 bit,
// 0.65 us for a 1 bit. WS2812B-2020 datasheet p.4: T0H 220..380 ns, T1H/T0L/T1L
// 580 ns..1 us, T0H+T0L and T1H+T1L >= 1.25 us, reset low >= 280 us; 24 bits per LED,
// GRB, most significant bit first. A frame ends on a zero compare value, so the line
// stays low until the next frame, which is the reset. The CPU only fills the buffer.
//
// TIM1_CH3N（PB15，默认映射，见 CH32V307 数据手册引脚表）上的 WS2812B 灯链。TIM1 以
// 1.28 us 为 PWM 周期，DMA1 通道 5（TIM1_UP 请求，RM 表 11-2）在每个更新事件装入一位的
// 比较值：0 码高 0.30 us，1 码高 0.65 us。WS2812B-2020 数据手册第 4 页：T0H 220..380 ns，
// T1H/T0L/T1L 580 ns..1 us，T0H+T0L 与 T1H+T1L 不小于 1.25 us，复位低电平不小于 280 us；
// 每颗 24 位，GRB 顺序，高位先发。一帧以比较值 0 结尾，线保持低电平直到下一帧，即复位。
// CPU 只负责填缓冲。

#include <cstdint>

namespace XrDap
{
struct Rgb
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

class Ws2812
{
 public:
  static constexpr uint8_t kMaxLeds = 8;

  /// TIM1、DMA1 通道 5 和 PB15；线为低电平 / TIM1, DMA1 channel 5 and PB15; the line
  /// idles low.
  static void Init();

  /// 上一帧还在发送 / The previous frame is still being sent.
  static bool Busy();

  /// 编码并开始发送一帧，立即返回；上一帧未发完时不发送并返回 false / Encode a frame
  /// and start sending it, returning at once; false without sending while the previous
  /// frame is still going out.
  static bool Show(const Rgb* leds, uint8_t count);
};
}  // namespace XrDap
