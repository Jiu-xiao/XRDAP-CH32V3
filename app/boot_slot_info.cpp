// SPDX-License-Identifier: Apache-2.0
#include "boot_slot_info.hpp"

#include <cstring>

#include "boot_slots.hpp"
#include "reserved_ram.h"
#include "xrdap/control/control.hpp"
#include "xrdap/radio/settings.hpp"

namespace XrDap
{

namespace
{

void Fill(BootSlots::Slot slot, BootSlotInfo::Slot& out, char* version)
{
  const uint32_t base = BootSlots::SlotBase(slot);
  const auto& record =
      *reinterpret_cast<const BootSlots::Record*>(base + BootSlots::kRecordOffset);
  const uint32_t first = *reinterpret_cast<const volatile uint32_t*>(base);
  const bool valid = BootSlots::RecordValid(record, slot, first);
  const uint32_t seq = valid ? record.seq : 0U;
  out.valid = valid ? 1U : 0U;
  for (int i = 0; i < 4; ++i)
  {
    out.seq[i] = static_cast<uint8_t>(seq >> (8 * i));
  }
  std::memset(version, 0, sizeof(record.version));
  if (valid)
  {
    std::memcpy(version, record.version, sizeof(record.version) - 1U);
  }
}

}  // namespace

bool BootSlotInfo::Control(const uint8_t* req, uint16_t len, uint8_t* resp, uint16_t cap,
                           uint16_t& out)
{
  if (len < 1U || req[0] != kOp)
  {
    return false;
  }
  if (cap < sizeof(Answer))
  {
    out = Control::StatusOnly(kOp, Radio::Status::kBadRequest, resp, cap);
    return true;
  }
  const auto* running = reinterpret_cast<const volatile uint32_t*>(XRDAP_BOOT_SLOT_ADDR);
  Answer answer{};
  answer.op = kOp;
  answer.status = Radio::Status::kOk;
  answer.running = running[0] == XRDAP_BOOT_SLOT_MAGIC && running[1] <= 1U
                       ? static_cast<uint8_t>(running[1])
                       : 0xFFU;
  Fill(BootSlots::Slot::A, answer.slot[0], answer.version[0]);
  Fill(BootSlots::Slot::B, answer.slot[1], answer.version[1]);
  std::memcpy(resp, &answer, sizeof(answer));
  out = sizeof(answer);
  return true;
}

}  // namespace XrDap
