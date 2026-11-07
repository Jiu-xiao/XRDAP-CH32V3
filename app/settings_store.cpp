// SPDX-License-Identifier: Apache-2.0
#include "settings_store.hpp"

#include "flash_layout.hpp"

namespace XrDap
{

namespace
{

// 256 字节一页（快速擦写的单位）；表在设置区末尾结束，Flash 窗口就是设置区。
// 256-byte pages (the fast erase/program unit); the table ends where the settings end,
// so the flash window is the settings area.
constexpr uint32_t kPageSize = 256;
constexpr uint32_t kSettingsBegin = Layout::kAppBase + Layout::kConfigOffset;
constexpr uint32_t kSettingsEnd = kSettingsBegin + Layout::kConfigSize;
static_assert(kSettingsBegin % kPageSize == 0 && Layout::kConfigSize % kPageSize == 0,
              "the settings area is whole pages");
constexpr LibXR::FlashRegion kRegions[] = {
    {Layout::kFlashBase, kPageSize, kSettingsEnd / kPageSize}};
// 数据库在内存里的缓冲，放得下一个键和它的名字。 / The database's buffer in RAM, room
// for one key and its name.
constexpr size_t kBufferSize = 256;

}  // namespace

SettingsStore::Storage::Storage()
    : flash(kRegions, 1, Layout::kFlashBase + kSettingsBegin),
      database(flash, kBufferSize),
      key(database, "radio", Radio::Settings{})
{
}

SettingsStore::SettingsStore(bool safe_mode)
{
  if (safe_mode)
  {
    return;
  }
  storage_.emplace();
  settings_ = storage_->key.data_;
}

bool SettingsStore::Save(const Radio::Settings& settings)
{
  if (!storage_)
  {
    return false;
  }
  if (storage_->key.Set(settings) != LibXR::ErrorCode::OK)
  {
    return false;
  }
  settings_ = settings;
  return true;
}

}  // namespace XrDap
