// SPDX-License-Identifier: Apache-2.0
#pragma once

// Bootloader slots. Stage 1 (bootloader/stage1.S) sits at address 0 and is never
// updated; it starts one of two bootloader slots, A or B, each closed by a slot record in
// its last XRDAP_BL_RECORD_SIZE bytes. A slot counts when its record is valid
// (RecordValid); of two valid slots stage 1 starts the one with the larger sequence
// number, and the other one while the boot key is held at power-up (SelectSlot). The
// running bootloader writes only the other slot and writes its record last, so an update
// that stops half way leaves the running slot in charge. Stage 1 implements RecordValid
// and SelectSlot in assembly; the two must stay identical.
//
// 引导槽。stage1（bootloader/stage1.S）位于地址 0，永不更新；它启动 A、B 两个引导槽之一，
// 每个槽最后 XRDAP_BL_RECORD_SIZE 字节是槽记录。记录有效（RecordValid）的槽才算数；两个都
// 有效时 stage1 启动序号大的那个，上电时按住 boot 键则启动另一个（SelectSlot）。运行中的
// 引导只写另一个槽，并最后写槽记录，所以中途停下的更新不会影响正在用的槽。stage1
// 用汇编实现 RecordValid 和 SelectSlot，两者必须一致。

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace XrDap::BootSlots
{
inline constexpr uint32_t kSlotBase[2] = {XRDAP_BL_SLOT_A_BASE, XRDAP_BL_SLOT_B_BASE};
inline constexpr uint32_t kSlotSize = XRDAP_BL_SLOT_SIZE;
inline constexpr uint32_t kRecordSize = XRDAP_BL_RECORD_SIZE;
inline constexpr uint32_t kRecordOffset = kSlotSize - kRecordSize;
inline constexpr uint32_t kImageLimit = kRecordOffset;

inline constexpr uint32_t kSealMagic = 0x4C414553U;    // "SEAL", LibXR's DFU seal record
inline constexpr uint32_t kRecordMagic = 0x534C4258U;  // "XBLS"
// Words a slot cannot start with: blank or erased flash (the CH32V30x flash reads
// 0xE339E339 when erased).
inline constexpr uint32_t kErasedWord = 0xE339E339U;

// Marker in front of the version string inside a bootloader image; the bootloader that
// writes a new image copies the string after it into the record.
inline constexpr char kVersionTag[] = "XRDAP-BLVER:";

enum class Slot : uint8_t
{
  A = 0,
  B = 1,
  NONE = 0xFF,
};

constexpr Slot Other(Slot slot) { return slot == Slot::A ? Slot::B : Slot::A; }
constexpr uint32_t SlotBase(Slot slot) { return kSlotBase[static_cast<uint8_t>(slot)]; }

// The record. Its first 16 bytes are LibXR's DFU seal record (magic, size, CRC-32 and its
// complement), so LibXR's own check of the slot's DFU interface still works; the slot
// fields follow. Written in one page write after the image; the rest of the page is 0xFF.
//
// 槽记录。前 16 字节是 LibXR 的 DFU seal 记录（魔数、长度、CRC-32 及其反码），LibXR 对该
// DFU 接口自己的检查照常成立；后面是槽字段。在镜像之后一次页写入；页内其余字节为 0xFF。
struct Record
{
  uint32_t seal_magic;  // kSealMagic
  uint32_t image_size;  // bytes of the image from the slot base
  uint32_t crc32;       // CRC-32 of the image (LibXR CRC32, as the DFU seal)
  uint32_t crc32_inv;   // ~crc32
  uint32_t magic;       // kRecordMagic
  uint32_t seq;         // one more than the slot that wrote it; 1 for the first slot
  uint32_t seq_inv;     // ~seq
  uint32_t base;        // the slot's base address
  char version[32];     // firmware version of the image, NUL terminated
};
static_assert(sizeof(Record) == 64, "the record layout is fixed (stage1.S reads it)");
static_assert(sizeof(Record) <= kRecordSize, "the record fits its page");

// Stage 1's rule for a valid slot. `first_word` is the slot's first word.
constexpr bool RecordValid(const Record& r, Slot slot, uint32_t first_word)
{
  return r.seal_magic == kSealMagic && (r.crc32 ^ r.crc32_inv) == 0xFFFFFFFFU &&
         r.magic == kRecordMagic && (r.seq ^ r.seq_inv) == 0xFFFFFFFFU &&
         r.base == SlotBase(slot) && r.image_size != 0U && r.image_size <= kImageLimit &&
         first_word != 0U && first_word != 0xFFFFFFFFU && first_word != kErasedWord;
}

// Stage 1's choice. Of two valid slots the larger sequence number wins (A on a tie); a
// held boot key turns the choice to the other valid slot.
constexpr Slot SelectSlot(bool valid_a, uint32_t seq_a, bool valid_b, uint32_t seq_b,
                          bool key_held)
{
  if (!valid_a && !valid_b) return Slot::NONE;
  if (!valid_b) return Slot::A;
  if (!valid_a) return Slot::B;
  const Slot newer = seq_b > seq_a ? Slot::B : Slot::A;
  return key_held ? Other(newer) : newer;
}

inline Record MakeRecord(Slot slot, uint32_t seq, uint32_t image_size, uint32_t crc32,
                         const char* version)
{
  Record r = {};
  r.seal_magic = kSealMagic;
  r.image_size = image_size;
  r.crc32 = crc32;
  r.crc32_inv = ~crc32;
  r.magic = kRecordMagic;
  r.seq = seq;
  r.seq_inv = ~seq;
  r.base = SlotBase(slot);
  std::memset(r.version, 0, sizeof(r.version));
  if (version != nullptr)
  {
    std::strncpy(r.version, version, sizeof(r.version) - 1U);
  }
  return r;
}

// True when image[i] starts kVersionTag followed by at least one character (the image
// also holds the bare tag constant, which this skips).
inline bool VersionTagAt(const uint8_t* image, size_t size, size_t i)
{
  constexpr size_t kTagLen = sizeof(kVersionTag) - 1U;
  return i + kTagLen < size && std::memcmp(image + i, kVersionTag, kTagLen) == 0 &&
         image[i + kTagLen] != 0U;
}

// Copies the version string that follows the first VersionTagAt in image[0, size) into
// out (NUL terminated, at most out_size - 1 characters); false when there is none.
inline bool FindVersion(const uint8_t* image, size_t size, char* out, size_t out_size)
{
  constexpr size_t kTagLen = sizeof(kVersionTag) - 1U;
  if (out_size == 0U) return false;
  for (size_t i = 0; i < size; ++i)
  {
    if (!VersionTagAt(image, size, i)) continue;
    size_t n = 0;
    for (size_t j = i + kTagLen; j < size && image[j] != 0U && n + 1U < out_size; ++j)
    {
      out[n++] = static_cast<char>(image[j]);
    }
    out[n] = '\0';
    return true;
  }
  out[0] = '\0';
  return false;
}
}  // namespace XrDap::BootSlots
