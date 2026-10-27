// SPDX-License-Identifier: Apache-2.0
#pragma once

// Watchdog and safe mode. The independent watchdog (LSI 40 kHz, RM 7.2) resets the chip
// when the main loop stops feeding it for about 8 s: a fatal error in LibXR or a trap
// ends in an endless loop, and the watchdog turns that into a reset. Three watchdog
// resets in a row start the application in safe mode: the configuration in flash is
// neither read nor written and the probe runs as a plain wired probe, so a bad
// configuration cannot keep the probe from reaching DFU. A power-on reset or 30 s of
// normal running clears the count.
//
// 看门狗与安全模式。独立看门狗（LSI 40 kHz，RM 7.2）在主循环约 8 s 没喂狗时复位芯片：
// LibXR 的致命错误或陷阱最终停在死循环里，看门狗把它变成一次复位。连续三次看门狗复位后
// 应用以安全模式启动：不读写 flash 里的配置，只当有线探针用，坏配置不会让探针进不了
// DFU。上电复位或正常运行 30 s 后计数清零。

namespace XrDap
{
/// Reads the reset cause, decides on safe mode and starts the watchdog / 读复位原因、
/// 判断安全模式并启动看门狗。
void BootGuardStart();
/// Feed the watchdog; call from the main loop / 喂狗，由主循环调用。
void BootGuardFeed();
/// True when this start is in safe mode / 本次启动处于安全模式时为 true。
bool SafeMode();
}  // namespace XrDap
