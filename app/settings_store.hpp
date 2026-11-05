// SPDX-License-Identifier: Apache-2.0
#pragma once

// The radio settings (XRDAP-Core Radio::Settings) in the XRDAP_CONFIG_SIZE bytes in front
// of the DFU seal at the end of the application window (common/flash_layout.hpp); the
// bootloader erases only the blocks an image covers, so an update keeps them. They are a
// key of LibXR's DatabaseRawSequential: every save erases and rewrites a whole block,
// first the backup, then the main one, which works on the CH32 flash whose erased words
// read 0xE339E339. In safe mode (boot_guard.hpp) the flash is neither read nor written.
//
// 无线设置（XRDAP-Core 的 Radio::Settings）放在应用窗口末尾
// DFU seal 前面的 XRDAP_CONFIG_SIZE 字节里（common/flash_layout.hpp）；
// 引导程序只擦镜像覆盖的块，升级时设置保留。
// 它们是 LibXR DatabaseRawSequential 的一个键：每次保存先擦写备份块、再擦写主块，
// 整块重写，适用于擦除后读出 0xE339E339 的 CH32 Flash。
// 安全模式（boot_guard.hpp）下不读也不写 Flash。

#include <optional>

#include "ch32_flash.hpp"
#include "database.hpp"
#include "xrdap/radio/settings.hpp"

namespace XrDap
{

class SettingsStore final : public Radio::Store
{
 public:
  /// @brief 打开存储并读出设置；安全模式下不碰 Flash，用默认设置 / Open the store and
  ///        read the settings; in safe mode the flash is not touched and the defaults
  ///        are used.
  explicit SettingsStore(bool safe_mode);

  /// 读出的设置 / The settings read.
  const Radio::Settings& Current() const { return settings_; }

  /// @return 写入成功时为 true；安全模式下为 false / true once written; false in safe
  ///         mode.
  bool Save(const Radio::Settings& settings) override;

 private:
  struct Storage
  {
    Storage();
    LibXR::CH32Flash flash;
    LibXR::DatabaseRawSequential database;
    LibXR::Database::Key<Radio::Settings> key;
  };

  std::optional<Storage> storage_;
  Radio::Settings settings_{};
};

}  // namespace XrDap
