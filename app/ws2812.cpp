// SPDX-License-Identifier: Apache-2.0
#include "ws2812.hpp"

#include <cstring>

namespace XrDap
{

namespace
{

constexpr uint32_t kMaxSpiHz = 18'000'000;
constexpr uint32_t kMinSpiHz = 9'000'000;
constexpr uint32_t kBitNs = 1250;
constexpr uint32_t kT0hNs = 330;
constexpr uint32_t kT1hNs = 650;

uint8_t SpiBits(uint32_t ns, uint32_t hz, bool round_up)
{
  const uint64_t scaled = static_cast<uint64_t>(ns) * hz;
  const uint64_t bits = round_up ? (scaled + 999'999'999ULL) / 1'000'000'000ULL
                                 : (scaled + 500'000'000ULL) / 1'000'000'000ULL;
  return static_cast<uint8_t>(bits);
}

}  // namespace

Ws2812::Ws2812(LibXR::SPI& spi) : spi_(spi)
{
  LibXR::SPI::Configuration config;
  config.clock_polarity = LibXR::SPI::ClockPolarity::LOW;
  config.clock_phase = LibXR::SPI::ClockPhase::EDGE_1;
  config.prescaler = spi_.CalcPrescaler(kMaxSpiHz, kMinSpiHz, true);
  config.double_buffer = false;
  ASSERT(config.prescaler != LibXR::SPI::Prescaler::UNKNOWN);
  spi_.SetConfig(config);
  const uint32_t hz = spi_.GetBusSpeed();
  bits_per_bit_ = SpiBits(kBitNs, hz, true);
  t0h_ = SpiBits(kT0hNs, hz, false);
  t1h_ = SpiBits(kT1hNs, hz, false);
  ASSERT(bits_per_bit_ <= kMaxSpiBits && t0h_ != 0U && t1h_ > t0h_);
}

void Ws2812::PutByte(uint8_t value, size_t& bit)
{
  for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U)
  {
    const uint8_t high = (value & mask) != 0U ? t1h_ : t0h_;
    for (uint8_t i = 0; i < bits_per_bit_; ++i, ++bit)
    {
      if (i < high)
      {
        frame_[bit / 8U] |= static_cast<uint8_t>(0x80U >> (bit % 8U));
      }
    }
  }
}

LibXR::ErrorCode Ws2812::Show(const Light::Rgb* leds, uint8_t count)
{
  if (count > kMaxLeds)
  {
    count = kMaxLeds;
  }
  std::memset(frame_, 0, sizeof(frame_));
  size_t bit = 8;  // 第一个字节保持低电平 / the first byte stays low
  for (uint8_t i = 0; i < count; ++i)
  {
    // WS2812B 按 G、R、B 的顺序收 / The WS2812B takes G, R, B.
    PutByte(leds[i].g, bit);
    PutByte(leds[i].r, bit);
    PutByte(leds[i].b, bit);
  }
  const size_t bytes = (bit + 7U) / 8U + 1U;  // 末尾一个低电平字节 / one low byte after
  return spi_.Write(LibXR::ConstRawData(frame_, bytes), op_);
}

}  // namespace XrDap
