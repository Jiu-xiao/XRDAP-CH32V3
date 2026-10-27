// SPDX-License-Identifier: Apache-2.0
#pragma once

// Settings kept in flash: link role, modulation and pairing. They live in the
// XRDAP_CONFIG_SIZE bytes in front of the DFU seal at the end of the application window;
// the bootloader only erases the blocks a firmware image covers, so an update keeps
// them. Safe mode (boot_guard.hpp) neither reads nor writes them.
//
// 保存在 flash 里的设置：链路角色、调制方式和配对。位于应用窗口末尾 DFU 封记之前的
// XRDAP_CONFIG_SIZE 字节；bootloader 只擦固件镜像覆盖的块，升级后设置保留。
// 安全模式（boot_guard.hpp）下既不读也不写。

#include <cstdint>

namespace XrDap
{
/// Link role setting / 链路角色设置。
enum class RoleSetting : uint8_t
{
  AUTO = 0,    ///< decided per debug session / 每次调试会话自动决定
  LOCAL = 1,   ///< always wired / 总是有线
  HOST = 2,    ///< always forward over the radio / 总是经无线转发
  REMOTE = 3,  ///< always execute commands from the radio / 总是执行无线收到的命令
};

enum class Modulation : uint8_t
{
  GFSK = 0,  ///< 2 Mb/s, 255-byte packets
  FLRC = 1,  ///< 1.3 Mb/s, 127-byte packets
};

struct Config
{
  RoleSetting role = RoleSetting::AUTO;
  Modulation modulation = Modulation::GFSK;
  uint8_t paired = 0;  ///< 1 once paired; 0 uses the default radio identity
  uint8_t reserved = 0;
  uint32_t link_id = 0;  ///< pair identity, the same on both probes
  uint8_t peer_uid[12] = {};
  uint8_t spare[12] = {};
};
static_assert(sizeof(Config) == 32, "the stored record keeps its size");

/// @brief 读设置：安全模式或没有存过时用默认值 / Load the settings: defaults in safe mode
///        or when none are stored (record log: settings_log.hpp).
/// @note 启动时由主任务调用一次，早于 GetConfig() / Call once from the main task at
///       start-up, before GetConfig().
void ConfigInit();

/// @brief 当前设置 / The current settings.
/// @note 引用一直有效，内容在 SaveConfig() 成功时改变；只在主任务里读，别的任务需要时拷贝
///       一份 / The reference stays valid; its contents change when SaveConfig()
///       succeeds. Read it on the main task only; other tasks take a copy there.
const Config& GetConfig();

/// @brief 保存新设置并使其成为当前设置 / Store new settings and make them current.
/// @return 写入并读回校验成功时为 true；安全模式下、ConfigInit() 之前或 Flash 擦写失败时
///         为 false，当前设置不变 / true once written and verified; false in safe mode,
///         before ConfigInit() or when erasing or writing the flash failed, with the
///         current settings unchanged.
/// @note 只由主任务调用；擦写期间 HCLK 减半（约数毫秒）/ Main task only; HCLK runs at
///       half speed while the flash is erased and written (a few milliseconds).
bool SaveConfig(const Config& config);
}  // namespace XrDap
