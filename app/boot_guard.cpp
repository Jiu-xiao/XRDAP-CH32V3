// SPDX-License-Identifier: Apache-2.0
#include "boot_guard.hpp"

#include <cstdint>

#include "ch32v30x.h"
#include "reserved_ram.h"
#include "timebase.hpp"

namespace XrDap
{

namespace
{

struct GuardState
{
  uint32_t magic;
  uint32_t watchdog_resets;
};
constexpr uint32_t kGuardMagic = 0x44524758U;  // "XGRD"
constexpr uint32_t kSafeModeResets = 3;
constexpr uint32_t kClearAfterMs = 30'000;
constexpr uint32_t kTimeoutMs = 8'000;
constexpr uint32_t kFeedMs = 10;

volatile GuardState& State()
{
  return *reinterpret_cast<volatile GuardState*>(XRDAP_BOOT_GUARD_ADDR);
}

}  // namespace

BootGuard::BootGuard() : watchdog_(kTimeoutMs, kFeedMs)
{
  const uint32_t flags = RCC->RSTSCKR;
  RCC->RSTSCKR |= RCC_RMVF;
  volatile GuardState& state = State();
  if ((flags & RCC_PORRSTF) != 0U || state.magic != kGuardMagic)
  {
    state.magic = kGuardMagic;
    state.watchdog_resets = 0;
  }
  if ((flags & RCC_IWDGRSTF) != 0U)
  {
    state.watchdog_resets = state.watchdog_resets + 1U;
  }
  safe_mode_ = state.watchdog_resets >= kSafeModeResets;
}

void BootGuard::Feed()
{
  watchdog_.Feed();
  if (!cleared_ &&
      static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds()) >= kClearAfterMs)
  {
    State().watchdog_resets = 0;
    cleared_ = true;
  }
}

}  // namespace XrDap
