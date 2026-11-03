// SPDX-License-Identifier: Apache-2.0
#include "boot_info.hpp"

#include <cstring>

#include "boot_slots.hpp"
#include "reserved_ram.h"

namespace XrDap
{
namespace
{
namespace Slots = BootSlots;

uint8_t* PutSlot(uint8_t* out, Slots::Slot slot, char* version)
{
  const uint32_t base = Slots::SlotBase(slot);
  const auto* record = reinterpret_cast<const Slots::Record*>(base + Slots::kRecordOffset);
  const uint32_t first = *reinterpret_cast<const volatile uint32_t*>(base);
  const bool valid = Slots::RecordValid(*record, slot, first);
  const uint32_t seq = valid ? record->seq : 0U;
  *out++ = valid ? 1U : 0U;
  std::memcpy(out, &seq, sizeof(seq));
  std::memset(version, 0, sizeof(Slots::Record::version));
  if (valid)
  {
    std::memcpy(version, record->version, sizeof(Slots::Record::version) - 1U);
  }
  return out + sizeof(seq);
}
}  // namespace

uint16_t BootInfoAnswer(uint8_t* resp, uint16_t resp_cap)
{
  if (resp_cap < kBootInfoSize)
  {
    resp[0] = kOpBootInfo;
    resp[1] = 3;  // bad argument: the answer does not fit
    return 2;
  }
  const auto* running = reinterpret_cast<const volatile uint32_t*>(XRDAP_BOOT_SLOT_ADDR);
  resp[0] = kOpBootInfo;
  resp[1] = 0;
  resp[2] = running[0] == XRDAP_BOOT_SLOT_MAGIC && running[1] <= 1U
                ? static_cast<uint8_t>(running[1])
                : 0xFFU;
  constexpr size_t kVersion = sizeof(Slots::Record::version);
  uint8_t* out = resp + 3;
  out = PutSlot(out, Slots::Slot::A, reinterpret_cast<char*>(resp + 13));
  out = PutSlot(out, Slots::Slot::B, reinterpret_cast<char*>(resp + 13 + kVersion));
  return static_cast<uint16_t>(13U + 2U * kVersion);
}
}  // namespace XrDap
