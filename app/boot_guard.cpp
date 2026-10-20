// SPDX-License-Identifier: Apache-2.0
#include "boot_guard.hpp"

#include <cstdint>

#include "ch32v30x.h"
#include "ch32v30x_iwdg.h"
#include "timebase.hpp"

namespace XrDap
{
namespace
{
// Kept in the reserved top of RAM next to the boot mailbox (common/boot_mailbox.h); the
// startup code does not clear it, so it survives a watchdog reset.
struct GuardState
{
  uint32_t magic;
  uint32_t watchdog_resets;
};
constexpr uintptr_t kGuardAddr = 0x2000FFE0u;
constexpr uint32_t kGuardMagic = 0x44524758u;  // "XGRD"
constexpr uint32_t kSafeModeResets = 3;
constexpr uint32_t kClearAfterMs = 30'000;
// LSI 40 kHz / 256 = 156.25 Hz; 1250 counts = 8 s (RM 7.2, 12-bit reload).
constexpr uint16_t kReload = 1250;

volatile GuardState& State() { return *reinterpret_cast<volatile GuardState*>(kGuardAddr); }

bool safe_mode = false;
bool cleared = false;
}  // namespace

void BootGuardStart()
{
  const uint32_t flags = RCC->RSTSCKR;
  RCC->RSTSCKR |= RCC_RMVF;
  volatile GuardState& state = State();
  if ((flags & RCC_PORRSTF) != 0U || state.magic != kGuardMagic)
  {
    state.magic = kGuardMagic;
    state.watchdog_resets = 0;
  }
  if ((flags & RCC_IWDGRSTF) != 0U) state.watchdog_resets = state.watchdog_resets + 1U;
  safe_mode = state.watchdog_resets >= kSafeModeResets;

  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(IWDG_Prescaler_256);
  IWDG_SetReload(kReload);
  IWDG_ReloadCounter();
  IWDG_Enable();
}

void BootGuardFeed()
{
  IWDG_ReloadCounter();
  if (!cleared &&
      static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds()) >= kClearAfterMs)
  {
    State().watchdog_resets = 0;
    cleared = true;
  }
}

bool SafeMode() { return safe_mode; }
}  // namespace XrDap
