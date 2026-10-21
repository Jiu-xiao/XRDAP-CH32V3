// SPDX-License-Identifier: Apache-2.0
#pragma once

// Record log of the settings (config.hpp). Pure logic over an array of record slots: the
// memory-mapped settings flash on the probe, a RAM array in tests/link_test.cpp.
//
// The settings are a log of 64-byte records, each written once into erased flash; the
// valid record (magic and CRC-32 correct) with the highest sequence number wins. The next
// record goes into the slot after the current one, wrapping at the end; a slot that
// starts a page is erased first (the page then holds no current record), and a slot in
// the middle of a page that is not erased moves the write to the start of the next page.
// So a power loss during a save leaves the previous record in place. Erased CH32 flash
// reads 0xE339E339 rather than all ones (RM chapter 32, notes on the erase operations).
//
// 设置的记录日志（config.hpp）。纯逻辑，作用于一个记录槽数组：探针上是映射到内存的设置区
// Flash，tests/link_test.cpp 里是 RAM 数组。设置是一串 64 字节记录，每条只写一次到已擦除
// 的 Flash；有效（魔数、CRC-32 正确）且序号最大的记录生效。下一条写到当前记录之后的槽，到
// 尾回绕；页首的槽先擦整页（该页此时没有当前记录），页中间未擦除的槽则改写到下一页的页
// 首。
// 所以保存中途掉电时上一条记录仍在。CH32 擦除后的 Flash 读出 0xE339E339 而不是全 1（RM 第
// 32 章擦除操作的注释）。

#include <cstddef>
#include <cstdint>

#include "config.hpp"

namespace XrDap::SettingsLog
{
struct Record
{
  uint32_t magic;
  uint32_t sequence;
  Config config;
  uint32_t crc;
  uint8_t pad[20];
};
static_assert(sizeof(Record) == 64, "a record is 64 bytes");

inline constexpr uint32_t kMagic = 0x47464358U;       // "XCFG"
inline constexpr uint32_t kErasedWord = 0xE339E339U;  // erased CH32 flash

uint32_t Crc32(const uint8_t* data, size_t size);

/// @brief 生成一条记录 / Build a record.
Record Make(const Config& config, uint32_t sequence);
/// 魔数和 CRC 都对 / Magic and CRC correct.
bool Valid(const Record& record);
/// 整条记录都是擦除值 / The whole record reads as erased.
bool Erased(const Record& record);

/// 当前记录 / The current record.
struct Head
{
  bool found = false;
  uint32_t slot = 0;
  uint32_t sequence = 0;
};

/// @brief 找当前记录：有效且序号最大的 / Find the current record: valid, highest
///        sequence.
Head Find(const Record* slots, uint32_t count);

/// @brief 下一条记录写在哪个槽 / The slot the next record goes into.
/// @param per_page 每页的槽数；页首的槽在写之前要擦整页 / Slots per page; a slot that
///        starts a page needs the page erased before the write.
uint32_t NextSlot(const Head& head, const Record* slots, uint32_t count,
                  uint32_t per_page);
}  // namespace XrDap::SettingsLog
