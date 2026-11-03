// SPDX-License-Identifier: Apache-2.0
#include "status_lights.hpp"

#include "timebase.hpp"

namespace XrDap
{

namespace
{

constexpr uint32_t kFrameMs = 20;
// 每颗灯的峰值（0..255）：灯串用 USB 供电，离眼睛近。 / Peak level per LED (0..255):
// the chain runs from USB power and sits close to the eye.
constexpr uint32_t kPeak = 32;
constexpr uint32_t kCometHead = 40;
constexpr uint32_t kCometTail = 12;
constexpr uint32_t kPermille = 1000;

uint32_t NowMs() { return static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds()); }

uint8_t Clamp(uint32_t v) { return v > 255U ? 255U : static_cast<uint8_t>(v); }

void AddWhite(Light::Rgb& c, uint32_t v)
{
  c = {Clamp(c.r + v), Clamp(c.g + v), Clamp(c.b + v)};
}

}  // namespace

StatusLights::StatusLights(const Probe& probe, const RadioPart& radio)
    : probe_(probe),
      radio_(radio),
      spi_(CH32_SPI2, {spi_rx_, sizeof(spi_rx_)}, {spi_tx_, sizeof(spi_tx_)},
           Board::kStatusLightSck.Port(), Board::kStatusLightSck.Mask(),
           Board::kStatusLightMiso.Port(), Board::kStatusLightMiso.Mask(),
           Board::kStatusLights.Port(), Board::kStatusLights.Mask()),
      chain_(spi_),
      rule_(NowMs()),
      next_ms_(NowMs())
{
}

bool StatusLights::Poll()
{
  const uint32_t now = NowMs();
  // 到时间就画一帧；晚了就从现在重新数 20 ms，不追赶。 / Draw when due; a late call
  // restarts the 20 ms rhythm from now instead of catching up.
  if (static_cast<int32_t>(now - next_ms_) >= 0)
  {
    next_ms_ = now + kFrameMs;
    Draw(now);
  }
  return false;
}

void StatusLights::Draw(uint32_t now)
{
  Light::Link link = probe_.UsbConfigured() ? Light::Link::WIRED : Light::Link::NONE;
  if (radio_.Wireless())
  {
    link = Light::Link::WIRELESS;
  }
  const Light::Setting setting =
      radio_.Pairing() ? Light::Setting::CHANGING : Light::Setting::NONE;
  const Light::Frame f = rule_.Update(link, setting, probe_.Activity().Read(now), now);
  const uint32_t level = kPeak * f.level / kPermille;
  const Light::Rgb base{static_cast<uint8_t>(f.color.r * level / 255U),
                        static_cast<uint8_t>(f.color.g * level / 255U),
                        static_cast<uint8_t>(f.color.b * level / 255U)};
  for (Light::Rgb& led : leds_)
  {
    led = base;
  }
  // 每个新脉冲让彗星前进一颗灯：命令不断时约每秒 15 步。 / A new pulse moves the comet
  // one LED on: about 15 steps a second under continuous traffic.
  if (f.pulse > last_pulse_)
  {
    comet_ = static_cast<uint8_t>((comet_ + 1U) % kCount);
  }
  last_pulse_ = f.pulse;
  if (f.pulse != 0U)
  {
    AddWhite(leds_[comet_], kCometHead * f.pulse / kPermille);
    AddWhite(leds_[(comet_ + kCount - 1U) % kCount], kCometTail * f.pulse / kPermille);
  }
  (void)chain_.Show(leds_, kCount);
}

}  // namespace XrDap
