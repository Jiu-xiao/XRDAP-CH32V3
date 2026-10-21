// SPDX-License-Identifier: Apache-2.0
#include "config.hpp"

#include <cstdint>

#include "boot_guard.hpp"
#include "ch32_flash.hpp"
#include "ch32v30x.h"
#include "ch32v30x_flash.h"
#include "flash_layout.hpp"
#include "settings_log.hpp"

namespace XrDap
{
namespace
{
using SettingsLog::Record;

constexpr uint32_t kFlashBase = Layout::kFlashBase;
constexpr uint32_t kConfigBase = Layout::kAppBase + Layout::kConfigOffset;  // CPU address
constexpr uint32_t kConfigEnd = kConfigBase + Layout::kConfigSize;
constexpr uint32_t kPageSize = LibXR::CH32Flash::PageSize();
static_assert(kConfigBase % kPageSize == 0U && Layout::kConfigSize % kPageSize == 0U &&
                  Layout::kConfigSize >= 2U * kPageSize,
              "the settings take at least two whole pages");
static_assert(kPageSize % sizeof(Record) == 0U, "records tile pages");
// The sector table ends where the settings end, so CH32Flash's window stops in front
// of the seal.
constexpr LibXR::FlashRegion kRegions[] = {
    {kFlashBase, kPageSize, kConfigEnd / kPageSize}};
constexpr uint32_t kSlots = Layout::kConfigSize / sizeof(Record);
constexpr uint32_t kSlotsPerPage = kPageSize / sizeof(Record);

// RM 32.1, note 2: above 120 MHz, halve HCLK around flash operations so that the flash
// access clock stays at or below 60 MHz, and restore it afterwards. LibXR's CH32Flash
// sets SCKMOD itself and expects SystemCoreClock <= 120 MHz.
class SlowBus
{
 public:
  SlowBus() : core_(SystemCoreClock)
  {
    if (core_ <= 120'000'000U) return;
    hpre_ = RCC->CFGR0 & RCC_HPRE;
    sckmod_ = FLASH->CTLR & kSckMod;
    RCC->CFGR0 = (RCC->CFGR0 & ~RCC_HPRE) | RCC_HPRE_DIV2;
    SystemCoreClock = core_ / 2U;
  }
  ~SlowBus()
  {
    if (core_ <= 120'000'000U) return;
    FLASH_Unlock();
    FLASH->CTLR = (FLASH->CTLR & ~kSckMod) | sckmod_;
    FLASH_Lock();
    RCC->CFGR0 = (RCC->CFGR0 & ~RCC_HPRE) | hpre_;
    SystemCoreClock = core_;
  }
  SlowBus(const SlowBus&) = delete;
  SlowBus& operator=(const SlowBus&) = delete;

 private:
  static constexpr uint32_t kSckMod = 1U << 25;
  uint32_t core_;
  uint32_t hpre_ = 0;
  uint32_t sckmod_ = 0;
};

const Record* Slots()
{
  return reinterpret_cast<const Record*>(kFlashBase + kConfigBase);
}

LibXR::CH32Flash& Flash()
{
  static LibXR::CH32Flash flash(kRegions, 1, kFlashBase + kConfigBase);
  return flash;
}

Config config;
bool loaded = false;
SettingsLog::Head head;
}  // namespace

void ConfigInit()
{
  if (SafeMode())
  {
    config.role = RoleSetting::LOCAL;  // a plain wired probe
    return;
  }
  head = SettingsLog::Find(Slots(), kSlots);
  if (head.found) config = Slots()[head.slot].config;
  loaded = true;
}

const Config& GetConfig() { return config; }

bool SaveConfig(const Config& next)
{
  if (!loaded) return false;
  const uint32_t slot = SettingsLog::NextSlot(head, Slots(), kSlots, kSlotsPerPage);
  const Record record = SettingsLog::Make(next, head.sequence + 1U);
  {
    SlowBus slow;
    LibXR::CH32Flash& flash = Flash();
    if (slot % kSlotsPerPage == 0U &&
        flash.Erase(slot / kSlotsPerPage * kPageSize, kPageSize) != LibXR::ErrorCode::OK)
      return false;
    if (flash.Write(slot * sizeof(Record), {&record, sizeof(record)}) !=
        LibXR::ErrorCode::OK)
      return false;
  }
  const Record& written = Slots()[slot];
  if (!SettingsLog::Valid(written) || written.sequence != record.sequence) return false;
  head = {true, slot, record.sequence};
  config = next;
  return true;
}
}  // namespace XrDap
