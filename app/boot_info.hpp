// SPDX-License-Identifier: Apache-2.0
#pragma once

// Control request 0x17 BootInfo (XRDAP-Core docs/protocol.md 3.7.1): which bootloader
// slot runs and what the two slots hold (common/boot_slots.hpp).
//   0x17 -> 0 running-slot (0 A, 1 B, 0xFF unknown) a-valid a-seq[4] b-valid b-seq[4]
//           a-version[32] b-version[32]
// The running slot is the one the bootloader wrote into the reserved RAM before it
// started the application (common/reserved_ram.h); valid and seq follow stage 1's rule,
// version is the string in the slot's record.
//
// 控制请求 0x17 BootInfo：哪个引导槽在运行、两个槽里各是什么。运行中的槽来自引导启动应用前
// 写进保留 RAM 的值；valid、seq 按 stage1 的规则判断，version 是槽记录里的字符串。

#include <cstdint>

namespace XrDap
{
inline constexpr uint8_t kOpBootInfo = 0x17;
inline constexpr uint16_t kBootInfoSize = 77;

/// @brief 填写 BootInfo 应答，返回长度 / Fill the BootInfo answer and return its size.
/// @note resp_cap 至少 kBootInfoSize / resp_cap at least kBootInfoSize.
uint16_t BootInfoAnswer(uint8_t* resp, uint16_t resp_cap);
}  // namespace XrDap
