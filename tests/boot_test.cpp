// SPDX-License-Identifier: Apache-2.0
// Tests of the boot layout on the build machine: the flash layout shared by the
// bootloader and the application (common/flash_layout.hpp) and stage 1's slot rules
// (common/boot_slots.hpp; bootloader/stage1.S implements the same). The expected values
// come from those headers and cmake/xrdap.cmake.
//
// 在编译机上测试启动布局：bootloader 和应用共用的 Flash 布局
// （common/flash_layout.hpp）和 stage 1 的槽规则（common/boot_slots.hpp；
// bootloader/stage1.S 实现同样的规则）。判据来自这两个头文件和 cmake/xrdap.cmake。

#include <cstdio>
#include <cstring>

#include "boot_slots.hpp"
#include "flash_layout.hpp"

namespace
{
int failures = 0;
int checks = 0;

#define CHECK(cond)                                               \
  do                                                              \
  {                                                               \
    ++checks;                                                     \
    if (!(cond))                                                  \
    {                                                             \
      ++failures;                                                 \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                             \
  } while (0)

using namespace XrDap;

void TestFlashLayout()
{
  using namespace Layout;
  // The settings and the seal stay at the flash addresses of the single-bootloader
  // layout, so a board keeps its settings across the migration to the A/B boot chain.
  CHECK(kAppBase == 0x1B000U && kAppBase + kConfigOffset == 0x3F700U &&
        kAppBase + kSealOffset == 0x3FF00U);
  CHECK(!OverlapsConfig(0, kConfigOffset));     // the largest image
  CHECK(OverlapsConfig(0, kConfigOffset + 1));  // one byte more
  CHECK(OverlapsConfig(kConfigOffset - 1, 2));
  CHECK(OverlapsConfig(kSealOffset - 1, 1));
  CHECK(!OverlapsConfig(kSealOffset, 256));  // the seal itself
  CHECK(!OverlapsConfig(kConfigOffset, 0));
  CHECK(OverlapsConfig(0xFFFFFF00U, 0x200U) == false);  // no wrap-around
}

// Stage 1's rules.
void TestBootSlots()
{
  using namespace BootSlots;
  using BootSlots::Slot;
  // The layout: two slots between stage 1's sector and the application, records last.
  CHECK(kSlotBase[0] == 0x1000U && kSlotBase[1] == 0xE000U && kSlotSize == 0xD000U);
  CHECK(kSlotBase[1] + kSlotSize == Layout::kAppBase);
  CHECK(kRecordOffset == 0xCF00U && kImageLimit == 0xCF00U);

  const Record a = MakeRecord(Slot::A, 7, 0x8000, 0x12345678, "0.2.0+gabcdef1");
  CHECK(RecordValid(a, Slot::A, 0x0000006FU));
  CHECK(!RecordValid(a, Slot::B, 0x0000006FU));  // the record names its own slot
  CHECK(!RecordValid(a, Slot::A, 0U));           // blank, programmed 0, erased slot start
  CHECK(!RecordValid(a, Slot::A, 0xFFFFFFFFU));
  CHECK(!RecordValid(a, Slot::A, kErasedWord));
  CHECK(std::strcmp(a.version, "0.2.0+gabcdef1") == 0);
  Record bad = a;
  bad.seal_magic ^= 1U;
  CHECK(!RecordValid(bad, Slot::A, 0x6FU));
  bad = a;
  bad.crc32_inv ^= 1U;
  CHECK(!RecordValid(bad, Slot::A, 0x6FU));
  bad = a;
  bad.magic = 0xFFFFFFFFU;
  CHECK(!RecordValid(bad, Slot::A, 0x6FU));
  bad = a;
  bad.seq_inv = bad.seq;  // a damaged sequence number
  CHECK(!RecordValid(bad, Slot::A, 0x6FU));
  bad = a;
  bad.image_size = 0U;
  CHECK(!RecordValid(bad, Slot::A, 0x6FU));
  bad = a;
  bad.image_size = kImageLimit + 1U;
  CHECK(!RecordValid(bad, Slot::A, 0x6FU));
  bad = a;
  bad.image_size = kImageLimit;  // the largest image
  CHECK(RecordValid(bad, Slot::A, 0x6FU));
  Record erased;
  std::memset(&erased, 0xFF, sizeof(erased));
  CHECK(!RecordValid(erased, Slot::A, 0x6FU));

  // SelectSlot: the truth table of stage 1.
  CHECK(SelectSlot(false, 0, false, 0, false) == Slot::NONE);
  CHECK(SelectSlot(false, 0, false, 0, true) == Slot::NONE);
  CHECK(SelectSlot(true, 3, false, 0, false) == Slot::A);
  CHECK(SelectSlot(true, 3, false, 9, true) ==
        Slot::A);  // one valid slot: the key changes nothing
  CHECK(SelectSlot(false, 9, true, 3, false) == Slot::B);
  CHECK(SelectSlot(false, 9, true, 3, true) == Slot::B);
  CHECK(SelectSlot(true, 3, true, 4, false) == Slot::B);
  CHECK(SelectSlot(true, 3, true, 4, true) == Slot::A);
  CHECK(SelectSlot(true, 5, true, 4, false) == Slot::A);
  CHECK(SelectSlot(true, 5, true, 4, true) == Slot::B);
  CHECK(SelectSlot(true, 4, true, 4, false) == Slot::A);  // a tie: A
  CHECK(SelectSlot(true, 4, true, 4, true) == Slot::B);
  CHECK(SelectSlot(true, 0xFFFFFFFFU, true, 1U, false) ==
        Slot::A);  // unsigned comparison

  // FindVersion: the bare tag constant is skipped, the marker with a string is found.
  const char image[] = "abc\0XRDAP-BLVER:\0xyzXRDAP-BLVER:0.2.0+g1234567\0tail";
  char version[32];
  CHECK(FindVersion(reinterpret_cast<const uint8_t*>(image), sizeof(image), version,
                    sizeof(version)));
  CHECK(std::strcmp(version, "0.2.0+g1234567") == 0);
  const char none[] = "XRDAP-BLVER:\0nothing";
  CHECK(!FindVersion(reinterpret_cast<const uint8_t*>(none), sizeof(none), version,
                     sizeof(version)));
}
}  // namespace

int main()
{
  TestFlashLayout();
  TestBootSlots();
  std::printf("%d checks, %d failed\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
