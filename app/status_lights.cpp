// SPDX-License-Identifier: Apache-2.0
#include "status_lights.hpp"

#include XRDAP_BOARD_HEADER
#include <optional>

#include "timebase.hpp"
#include "ws2812.hpp"
#include "xrdap/light/status_light.hpp"
#ifdef XRDAP_HAS_RADIO
#include "dap_session.hpp"
#include "pairing.hpp"
#endif

namespace XrDap
{
namespace
{
constexpr uint32_t kFrameMs = 20;  // 50 frames per second
constexpr uint8_t kCount = Board::kStatusLightCount;
static_assert(kCount <= Ws2812::kMaxLeds, "WS2812 chain longer than the driver buffer");

// Peak level per LED (0..255): the chain runs from USB power and sits close to the eye.
constexpr uint32_t kPeak = 32;
constexpr uint8_t kCometHead = 40;
constexpr uint8_t kCometTail = 12;

struct Inputs
{
  const DapActivity* activity;
  bool (*usb_configured)(void*);
  void* usb_ctx;
};
Inputs inputs{};

uint8_t Clamp(uint32_t v) { return v > 255U ? 255U : static_cast<uint8_t>(v); }

void AddWhite(Rgb& c, uint32_t v)
{
  c = {Clamp(c.r + v), Clamp(c.g + v), Clamp(c.b + v)};
}

uint32_t NowMs() { return static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds()); }

/// The frame state, created by StatusLightsBegin.
struct Frame
{
  Light::StatusLight rule;
  Rgb leds[kCount];
  uint8_t comet = 0;
  uint32_t last_pulse = 0;
  uint32_t next_ms;
  explicit Frame(uint32_t now) : rule(now), next_ms(now + kFrameMs) {}
};
std::optional<Frame> frame;

void DrawFrame(Frame& st, uint32_t now)
{
  Light::Link link =
      inputs.usb_configured(inputs.usb_ctx) ? Light::Link::WIRED : Light::Link::NONE;
  Light::Setting setting = Light::Setting::NONE;
#ifdef XRDAP_HAS_RADIO
  if (RadioSessionWireless() || RemoteSessionActive())
  {
    link = Light::Link::WIRELESS;
  }
  if (GetPairState() == PairState::RUNNING)
  {
    setting = Light::Setting::CHANGING;
  }
#endif

  const Light::Frame f = st.rule.Update(link, setting, inputs.activity->Read(now), now);
  const uint32_t level = kPeak * f.level / 1000U;
  const Rgb base{static_cast<uint8_t>(f.color.r * level / 255U),
                 static_cast<uint8_t>(f.color.g * level / 255U),
                 static_cast<uint8_t>(f.color.b * level / 255U)};
  for (uint8_t i = 0; i < kCount; ++i)
  {
    st.leds[i] = base;
  }
  // A new pulse moves the comet one LED on: about 15 steps a second under continuous
  // traffic.
  if (f.pulse > st.last_pulse)
  {
    st.comet = static_cast<uint8_t>((st.comet + 1U) % kCount);
  }
  st.last_pulse = f.pulse;
  if (f.pulse != 0U)
  {
    AddWhite(st.leds[st.comet], kCometHead * f.pulse / 1000U);
    AddWhite(st.leds[(st.comet + kCount - 1U) % kCount], kCometTail * f.pulse / 1000U);
  }
  (void)Ws2812::Show(st.leds, kCount);
}
}  // namespace

void StatusLightsBegin(const DapActivity& activity, bool (*usb_configured)(void*),
                       void* usb_ctx)
{
  inputs = {&activity, usb_configured, usb_ctx};
  Ws2812::Init();
  frame.emplace(NowMs());
}

void StatusLightsPoll()
{
  if (!frame)
  {
    return;
  }
  const uint32_t now = NowMs();
  // Wrap-safe: due when now is at or past next_ms. A late call draws one frame and
  // restarts the 20 ms rhythm from now instead of catching up.
  if (static_cast<int32_t>(now - frame->next_ms) < 0)
  {
    return;
  }
  frame->next_ms = now + kFrameMs;
  DrawFrame(*frame, now);
}

}  // namespace XrDap
