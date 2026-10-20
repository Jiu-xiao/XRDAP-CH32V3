// SPDX-License-Identifier: Apache-2.0
#include "config.hpp"

#include <cstddef>
#include <cstring>

#include "boot_guard.hpp"
#include "ch32_flash.hpp"
#include "ch32v30x.h"
#include "ch32v30x_flash.h"

namespace XrDap
{
namespace
{
// The flash controller sees the flash at 0x08000000; the application window and the
// seal use CPU addresses from 0 (cmake/ch32v307.cmake).
constexpr uint32_t kFlashBase = 0x08000000U;
constexpr uint32_t kConfigEnd = XRDAP_APP_BASE + XRDAP_APP_SIZE - XRDAP_SEAL_SIZE;
constexpr uint32_t kConfigBase = kConfigEnd - XRDAP_CONFIG_SIZE;
constexpr uint32_t kPageSize = LibXR::CH32Flash::PageSize();
static_assert(kConfigBase % kPageSize == 0U && XRDAP_CONFIG_SIZE % kPageSize == 0U &&
                  XRDAP_CONFIG_SIZE >= 2U * kPageSize,
              "the settings take at least two whole pages");
// The sector table ends where the settings end, so CH32Flash's window stops in front
// of the seal.
constexpr LibXR::FlashRegion kRegions[] = {{kFlashBase, kPageSize, kConfigEnd / kPageSize}};

// The settings are a log of fixed-size records, each written once into erased flash;
// the valid record with the highest sequence number wins. Writing the next record never
// erases the page that holds the current one, so a power loss during a save leaves the
// previous settings in place. Erased CH32 flash reads 0xE339E339 rather than all ones
// (RM chapter 32, notes on the erase operations), so a slot counts as free when it reads
// that pattern.
struct Record
{
  uint32_t magic;
  uint32_t sequence;
  Config config;
  uint32_t crc;
  uint8_t pad[20];
};
static_assert(sizeof(Record) == 64 && kPageSize % sizeof(Record) == 0U, "records tile pages");
constexpr uint32_t kMagic = 0x47464358U;  // "XCFG"
constexpr uint32_t kErasedWord = 0xE339E339U;
constexpr uint32_t kSlots = XRDAP_CONFIG_SIZE / sizeof(Record);
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

uint32_t Crc32(const uint8_t* data, size_t size)
{
  uint32_t crc = 0xFFFFFFFFU;
  for (size_t i = 0; i < size; ++i)
  {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
  }
  return ~crc;
}

uint32_t RecordCrc(const Record& record)
{
  return Crc32(reinterpret_cast<const uint8_t*>(&record), offsetof(Record, crc));
}

const Record& Slot(uint32_t index)
{
  return *reinterpret_cast<const Record*>(kFlashBase + kConfigBase + index * sizeof(Record));
}

bool SlotValid(uint32_t index)
{
  const Record& record = Slot(index);
  return record.magic == kMagic && record.crc == RecordCrc(record);
}

bool SlotErased(uint32_t index)
{
  const auto* words = reinterpret_cast<const uint32_t*>(&Slot(index));
  for (size_t i = 0; i < sizeof(Record) / sizeof(uint32_t); ++i)
    if (words[i] != kErasedWord) return false;
  return true;
}

LibXR::CH32Flash& Flash()
{
  static LibXR::CH32Flash flash(kRegions, 1, kFlashBase + kConfigBase);
  return flash;
}

Config config;
bool loaded = false;
bool have_record = false;
uint32_t current_slot = 0, current_sequence = 0;
}  // namespace

void ConfigInit()
{
  if (SafeMode())
  {
    config.role = RoleSetting::LOCAL;  // a plain wired probe
    return;
  }
  for (uint32_t i = 0; i < kSlots; ++i)
  {
    if (!SlotValid(i)) continue;
    if (!have_record || Slot(i).sequence > current_sequence)
    {
      have_record = true;
      current_slot = i;
      current_sequence = Slot(i).sequence;
    }
  }
  if (have_record) config = Slot(current_slot).config;
  loaded = true;
}

const Config& GetConfig() { return config; }

bool SaveConfig(const Config& next)
{
  if (!loaded) return false;
  uint32_t slot = have_record ? (current_slot + 1U) % kSlots : 0U;
  // A slot that is not erased (left by something else) moves the write to the next page.
  if (slot % kSlotsPerPage != 0U && !SlotErased(slot))
    slot = (slot / kSlotsPerPage + 1U) * kSlotsPerPage % kSlots;

  Record record{};
  record.magic = kMagic;
  record.sequence = current_sequence + 1U;
  record.config = next;
  record.crc = RecordCrc(record);
  {
    SlowBus slow;
    LibXR::CH32Flash& flash = Flash();
    if (slot % kSlotsPerPage == 0U &&
        flash.Erase(slot / kSlotsPerPage * kPageSize, kPageSize) != LibXR::ErrorCode::OK)
      return false;
    if (flash.Write(slot * sizeof(Record), {&record, sizeof(record)}) != LibXR::ErrorCode::OK)
      return false;
  }
  if (!SlotValid(slot) || Slot(slot).sequence != record.sequence) return false;
  have_record = true;
  current_slot = slot;
  current_sequence = record.sequence;
  config = next;
  return true;
}
}  // namespace XrDap
