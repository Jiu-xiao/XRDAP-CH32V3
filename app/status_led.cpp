// SPDX-License-Identifier: Apache-2.0
#include "status_led.hpp"

#include "timebase.hpp"

namespace XrDap
{

namespace
{

uint32_t NowMs() { return static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds()); }

// 脉冲的前一半熄灭：一阵 DAP 命令一个短暗点。 / Dark for the first half of the pulse:
// one short gap per burst of DAP commands.
constexpr uint32_t kPulseDarkAbove = 500;

}  // namespace

StatusLed::StatusLed(const Probe& probe)
    : probe_(probe), led_(Board::kLed.Port(), Board::kLed.Mask()), rule_(NowMs())
{
  Write(false);
}

bool StatusLed::Poll()
{
  const uint32_t now = NowMs();
  const Light::Link link =
      probe_.UsbConfigured() ? Light::Link::WIRED : Light::Link::NONE;
  const Light::Frame frame =
      rule_.Update(link, Light::Setting::NONE, probe_.Activity().Read(now), now);
  Write(link == Light::Link::WIRED && frame.pulse < kPulseDarkAbove);
  return false;
}

void StatusLed::Write(bool lit) { led_.Write(lit == Board::kLedActiveHigh); }

}  // namespace XrDap
