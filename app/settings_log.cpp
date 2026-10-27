// SPDX-License-Identifier: Apache-2.0
#include "settings_log.hpp"

#include <cstring>

namespace XrDap::SettingsLog
{
uint32_t Crc32(const uint8_t* data, size_t size)
{
  uint32_t crc = 0xFFFFFFFFU;
  for (size_t i = 0; i < size; ++i)
  {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
  }
  return ~crc;
}

namespace
{
uint32_t RecordCrc(const Record& record)
{
  return Crc32(reinterpret_cast<const uint8_t*>(&record), offsetof(Record, crc));
}
}  // namespace

Record Make(const Config& config, uint32_t sequence)
{
  Record record{};
  record.magic = kMagic;
  record.sequence = sequence;
  record.config = config;
  record.crc = RecordCrc(record);
  return record;
}

bool Valid(const Record& record)
{
  return record.magic == kMagic && record.crc == RecordCrc(record);
}

bool Erased(const Record& record)
{
  uint32_t words[sizeof(Record) / sizeof(uint32_t)];
  std::memcpy(words, &record, sizeof(words));
  for (uint32_t word : words)
    if (word != kErasedWord) return false;
  return true;
}

Head Find(const Record* slots, uint32_t count)
{
  Head head;
  for (uint32_t i = 0; i < count; ++i)
  {
    if (!Valid(slots[i])) continue;
    if (!head.found || slots[i].sequence > head.sequence)
    {
      head.found = true;
      head.slot = i;
      head.sequence = slots[i].sequence;
    }
  }
  return head;
}

uint32_t NextSlot(const Head& head, const Record* slots, uint32_t count,
                  uint32_t per_page)
{
  uint32_t slot = head.found ? (head.slot + 1U) % count : 0U;
  // A slot that is not erased (left by something else) moves the write to the next page.
  if (slot % per_page != 0U && !Erased(slots[slot]))
    slot = (slot / per_page + 1U) * per_page % count;
  return slot;
}
}  // namespace XrDap::SettingsLog
