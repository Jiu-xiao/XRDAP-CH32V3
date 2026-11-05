// SPDX-License-Identifier: Apache-2.0
#pragma once

// Watchdog and safe mode. The independent watchdog resets the chip when the main loop
// stops feeding it for 8 s: a fatal error in LibXR or a trap ends in an endless loop, and
// the watchdog turns that into a reset. Three watchdog resets in a row start the
// application in safe mode: the settings in flash are neither read nor written and the
// probe runs as a plain wired probe, so bad settings cannot keep it from reaching DFU. A
// power-on reset or 30 s of normal running clears the count, which lives in the reserved
// top of RAM (common/reserved_ram.h) and survives a watchdog reset.
//
// 看门狗和安全模式。主循环 8 s 不喂狗时独立看门狗复位芯片：
// LibXR 的致命错误或陷阱最后停在死循环里，看门狗把它变成复位。
// 连续三次看门狗复位后以安全模式启动：不读也不写 Flash 里的设置，
// 探针只作有线探针，坏设置不会让它进不了 DFU。
// 上电复位或正常运行 30 s 清零计数；计数放在 RAM 顶部的保留区
// （common/reserved_ram.h），看门狗复位后仍在。

#include "ch32_watchdog.hpp"

namespace XrDap
{

class BootGuard
{
 public:
  /// @brief 启动看门狗并数看门狗复位 / Start the watchdog and count watchdog resets.
  /// @note 启动时最先构造 / Construct first at start-up.
  BootGuard();

  /// @brief 喂狗；正常运行 30 s 后清零计数 / Feed the watchdog; clears the count after
  ///        30 s of normal running.
  void Feed();

  /// 安全模式 / Safe mode.
  bool SafeMode() const { return safe_mode_; }

 private:
  LibXR::CH32Watchdog watchdog_;
  bool safe_mode_ = false;
  bool cleared_ = false;
};

}  // namespace XrDap
