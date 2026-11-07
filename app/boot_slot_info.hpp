// SPDX-License-Identifier: Apache-2.0
#pragma once

// Control request 0x17 (XRDAP-Core docs/protocol.md 8.3): which bootloader slot runs and
// what the two slots hold (common/boot_slots.hpp). The running slot is the one the
// bootloader wrote into the reserved RAM before it started the application
// (common/reserved_ram.h); valid and seq follow stage 1's rule, version is the string in
// the slot's record.
//
// 控制请求 0x17（XRDAP-Core docs/protocol.md 8.3）：
// 正在运行哪个引导槽，两个槽里是什么（common/boot_slots.hpp）。
// 运行的槽是引导程序启动应用前写进保留 RAM 的那个（common/reserved_ram.h）；
// valid 和 seq 按 stage 1 的规则，version 是槽记录里的字符串。

#include <cstdint>

#include "part.hpp"

namespace XrDap
{

class BootSlotInfo final : public Part
{
 public:
  static constexpr uint8_t kOp = 0x17;

  /// 一个槽 / One slot.
  struct __attribute__((packed)) Slot
  {
    uint8_t valid;
    uint8_t seq[4];  ///< 小端 / little endian
  };

  /// 0x17 的应答 / Answer of 0x17.
  struct __attribute__((packed)) Answer
  {
    uint8_t op;
    uint8_t status;
    uint8_t running;  ///< 0 A，1 B，0xFF 不知道 / 0 A, 1 B, 0xFF unknown
    Slot slot[2];
    char version[2][32];
  };
  static_assert(sizeof(Answer) == 77, "the answer layout is fixed");

  bool Control(const uint8_t* req, uint16_t len, uint8_t* resp, uint16_t cap,
               uint16_t& out) override;
};

}  // namespace XrDap
