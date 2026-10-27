// SPDX-License-Identifier: Apache-2.0
#include "status_led.hpp"

#include <new>

#include XRDAP_BOARD_HEADER
#include "ch32v30x.h"
#include "timebase.hpp"
#include "xrdap/light/status_light.hpp"

namespace XrDap
{
namespace
{
const DapActivity* activity = nullptr;
bool (*usb_configured)(void*) = nullptr;
void* usb_ctx = nullptr;
// A function-local static would need a guard (no thread-safe statics) and the startup
// code runs no constructors: built on the first update instead.
alignas(Light::StatusLight) uint8_t rule_storage[sizeof(Light::StatusLight)];
Light::StatusLight* rule = nullptr;

uint32_t NowMs() { return static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds()); }

void Write(bool lit)
{
  GPIO_WriteBit(Board::kLed.Port(), Board::kLed.Mask(),
                lit == Board::kLedActiveHigh ? Bit_SET : Bit_RESET);
}
}  // namespace

void StatusLedBegin(const DapActivity& a, bool (*configured)(void*), void* ctx)
{
  activity = &a;
  usb_configured = configured;
  usb_ctx = ctx;
  RCC_APB2PeriphClockCmd(PortClock(Board::kLed.port), ENABLE);
  Write(false);
  GPIO_InitTypeDef init{};
  init.GPIO_Pin = Board::kLed.Mask();
  init.GPIO_Mode = GPIO_Mode_Out_PP;
  init.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(Board::kLed.Port(), &init);
}

void StatusLedUpdate()
{
  if (activity == nullptr) return;
  const uint32_t now = NowMs();
  if (rule == nullptr) rule = new (rule_storage) Light::StatusLight(now);
  const Light::Link link =
      usb_configured(usb_ctx) ? Light::Link::WIRED : Light::Link::NONE;
  const Light::Frame f =
      rule->Update(link, Light::Setting::NONE, activity->Read(now), now);
  // Half of the pulse dark: one short gap per DAP command burst.
  Write(link == Light::Link::WIRED && f.pulse < 500U);
}
}  // namespace XrDap
