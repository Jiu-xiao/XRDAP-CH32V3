// SPDX-License-Identifier: Apache-2.0
// XR-DAP-V307 USB DFU bootloader.
//
// Stage 1 starts it from one of the two bootloader slots (common/boot_slots.hpp); the
// same code is linked once for each slot (XRDAP_BL_SLOT). At every reset it decides
// whether to start the application or stay in DFU mode. It stays when the application
// asked for it through the boot mailbox (DFU DETACH), when the board's boot key is held,
// or when the application window holds no valid sealed image. In DFU mode it exposes two
// USB DFU interfaces on USBHS (dfu-util works; WinUSB is announced for Windows):
// interface 0 writes the application, and a completed download writes the seal record and
// starts the new image; interface 1 writes a bootloader image into the other slot, closes
// it with a slot record whose sequence number is one more than its own, and restarts in
// DFU mode, so that stage 1 starts the new bootloader.
//
// XR-DAP-V307 的 USB DFU bootloader。
// stage1 从两个引导槽之一启动它（common/boot_slots.hpp）；同一份代码为每个槽各链接一次
// （XRDAP_BL_SLOT）。每次复位时决定启动应用还是停留在 DFU 模式。应用经启动信箱请求
// （DFU DETACH）、上电时按住板上的 bootloader 键，或应用区没有有效的带 seal 镜像时停留。
// DFU 模式下在 USBHS 上提供两个 USB DFU 接口（可用 dfu-util，Windows 上自动使用 WinUSB）：
// 接口 0 写应用，下载完成后写入 seal 记录并启动新镜像；接口 1 把引导镜像写进另一个槽，用序号
// 比自己大 1 的槽记录收尾，然后重启并停在 DFU，由 stage1 启动新的引导。

#include <cstddef>
#include <cstdint>
#include <cstring>

#include XRDAP_BOARD_HEADER
#include "ch32_flash.hpp"
#include "ch32_timebase.hpp"
#include "ch32_usb_dev.hpp"
#include "ch32v30x.h"
#include "dfu/dfu_bootloader.hpp"
#include "boot_slots.hpp"
#include "flash_layout.hpp"
#include "libxr.hpp"
#include "reserved_ram.h"
#include "thread.hpp"
#include "chip_uid.hpp"
#include "xrdap/probe/probe_identity.hpp"

namespace
{
namespace Board = XrDap::Board;
namespace Layout = XrDap::Layout;
namespace Slots = XrDap::BootSlots;

constexpr uint32_t kFlashBase = Layout::kFlashBase;
constexpr uint32_t kPageSize = LibXR::CH32Flash::PageSize();
constexpr uint32_t kAppBase = Layout::kAppBase;  // CPU address of the application
constexpr uint32_t kAppSize = Layout::kAppSize;
constexpr uint32_t kLedPeriodMs = 250U;
// The bootloader restarts this long after it closed the other slot, once the host has
// read the final DFU status.
constexpr uint32_t kRestartAfterSlotMs = 500U;

constexpr Slots::Slot kSelf = XRDAP_BL_SLOT == 0 ? Slots::Slot::A : Slots::Slot::B;
constexpr Slots::Slot kPeer = Slots::Other(kSelf);
constexpr uint32_t kPeerBase = Slots::SlotBase(kPeer);

// Stage 1 reads the boot key with the pin the board's cmake gives it.
static_assert(Board::kBootKey.port == XRDAP_BOOT_KEY_GPIO &&
                  Board::kBootKey.pin == XRDAP_BOOT_KEY_PIN,
              "stage 1 and the bootloader read the same boot key");

static_assert(kAppBase % kPageSize == 0U && kAppSize % kPageSize == 0U,
              "the application window must be page aligned");

// Pages from the flash start to the end of the application window; the DFU storage
// area is the application window at its end.
constexpr LibXR::FlashRegion kFlashRegions[] = {
    {kFlashBase, kPageSize, (kAppBase + kAppSize) / kPageSize}};

// The application window minus the settings: erases and writes that would touch the
// settings fail, so a DFU download larger than the image area is refused (the DFU
// state machine reports the error) instead of overwriting the settings.
class AppWindowFlash : public LibXR::Flash
{
 public:
  explicit AppWindowFlash(LibXR::Flash& flash)
      : LibXR::Flash(flash.MinEraseSize(), flash.MinWriteSize(),
                     {reinterpret_cast<void*>(kFlashBase + kAppBase), kAppSize}),
        flash_(flash)
  {
  }

  LibXR::ErrorCode Erase(size_t offset, size_t size) override
  {
    if (Layout::OverlapsConfig(static_cast<uint32_t>(offset),
                               static_cast<uint32_t>(size)))
    {
      return LibXR::ErrorCode::OUT_OF_RANGE;
    }
    return flash_.Erase(offset, size);
  }

  LibXR::ErrorCode Write(size_t offset, LibXR::ConstRawData data) override
  {
    if (Layout::OverlapsConfig(static_cast<uint32_t>(offset),
                               static_cast<uint32_t>(data.size_)))
    {
      return LibXR::ErrorCode::OUT_OF_RANGE;
    }
    return flash_.Write(offset, data);
  }

  LibXR::ErrorCode Read(size_t offset, LibXR::RawData data) override
  {
    return flash_.Read(offset, data);
  }

 private:
  LibXR::Flash& flash_;
};

// The other bootloader slot behind DFU interface 1. A download that starts at offset 0
// first erases the slot's record, so that a download that stops half way leaves no valid
// record behind. LibXR's seal write at the record offset becomes the slot record: the
// seal's size and CRC, the next sequence number and the version string of the new image.
// Images not linked for this slot (the second vector word is the image base) or longer
// than the slot allows are refused, and the slot stays without a record.
class PeerSlotFlash : public LibXR::Flash
{
 public:
  PeerSlotFlash(LibXR::Flash& flash, uint32_t next_seq)
      : LibXR::Flash(flash.MinEraseSize(), flash.MinWriteSize(),
                     {reinterpret_cast<void*>(kFlashBase + kPeerBase), Slots::kSlotSize}),
        flash_(flash),
        next_seq_(next_seq)
  {
  }

  LibXR::ErrorCode Erase(size_t offset, size_t size) override
  {
    if (offset + size > Slots::kSlotSize)
    {
      return LibXR::ErrorCode::OUT_OF_RANGE;
    }
    if (offset == 0U)
    {
      const auto ec = flash_.Erase(Slots::kRecordOffset, Slots::kRecordSize);
      if (ec != LibXR::ErrorCode::OK)
      {
        return ec;
      }
    }
    return flash_.Erase(offset, size);
  }

  LibXR::ErrorCode Write(size_t offset, LibXR::ConstRawData data) override
  {
    if (offset == Slots::kRecordOffset)
    {
      return WriteRecord(data);
    }
    if (offset + data.size_ > Slots::kImageLimit)
    {
      return LibXR::ErrorCode::OUT_OF_RANGE;
    }
    return flash_.Write(offset, data);
  }

  LibXR::ErrorCode Read(size_t offset, LibXR::RawData data) override
  {
    return flash_.Read(offset, data);
  }

  // True once after a slot record was written.
  bool TakeClosed()
  {
    const bool closed = closed_;
    closed_ = false;
    return closed;
  }

 private:
  LibXR::ErrorCode WriteRecord(LibXR::ConstRawData seal_data)
  {
    uint32_t seal[4] = {};
    uint32_t vectors[2] = {};
    if (seal_data.size_ < sizeof(seal) ||
        flash_.Read(0U, {vectors, sizeof(vectors)}) != LibXR::ErrorCode::OK)
    {
      return LibXR::ErrorCode::FAILED;
    }
    std::memcpy(seal, seal_data.addr_, sizeof(seal));
    const uint32_t image_size = seal[1];
    if (seal[0] != Slots::kSealMagic || image_size == 0U ||
        image_size > Slots::kImageLimit || vectors[1] != kPeerBase)
    {
      return LibXR::ErrorCode::FAILED;
    }
    char version[sizeof(Slots::Record::version)] = {};
    FindImageVersion(image_size, version, sizeof(version));
    const Slots::Record record =
        Slots::MakeRecord(kPeer, next_seq_, image_size, seal[2], version);
    alignas(4) uint8_t page[Slots::kRecordSize];
    std::memset(page, 0xFF, sizeof(page));
    std::memcpy(page, &record, sizeof(record));
    const auto ec = flash_.Write(Slots::kRecordOffset, {page, sizeof(page)});
    closed_ = ec == LibXR::ErrorCode::OK;
    return ec;
  }

  // Slots::FindVersion over the image in flash, read in chunks through the flash driver.
  // Each pass searches the bytes where a whole tag and string fit and keeps the rest for
  // the next pass, so a tag that crosses a chunk boundary is still found whole.
  void FindImageVersion(uint32_t image_size, char* out, size_t out_size)
  {
    constexpr size_t kChunk = 256U;
    constexpr size_t kKeep = sizeof(Slots::kVersionTag) + sizeof(Slots::Record::version);
    uint8_t buffer[kKeep + kChunk];
    size_t kept = 0U;
    out[0] = '\0';
    for (uint32_t offset = 0U; offset < image_size; offset += kChunk)
    {
      const size_t n = image_size - offset < kChunk ? image_size - offset : kChunk;
      if (flash_.Read(offset, {buffer + kept, n}) != LibXR::ErrorCode::OK)
      {
        return;
      }
      const size_t total = kept + n;
      const bool last = offset + n >= image_size;
      // A tag may start anywhere in [0, starts); the bytes after it are in the buffer.
      const size_t starts = last ? total : (total > kKeep ? total - kKeep : 0U);
      if (TagStartsBefore(buffer, total, starts) &&
          Slots::FindVersion(buffer, total, out, out_size))
      {
        return;
      }
      kept = total < kKeep ? total : kKeep;
      std::memmove(buffer, buffer + total - kept, kept);
    }
    out[0] = '\0';
  }

  static bool TagStartsBefore(const uint8_t* buffer, size_t size, size_t limit)
  {
    for (size_t i = 0; i < limit; ++i)
    {
      if (Slots::VersionTagAt(buffer, size, i))
      {
        return true;
      }
    }
    return false;
  }

  LibXR::Flash& flash_;
  uint32_t next_seq_;
  bool closed_ = false;
};

// The version marker the bootloader that writes this image into the other slot looks
// for (Slots::kVersionTag).
extern "C" __attribute__((used)) const char xrdap_bl_version[] =
    "XRDAP-BLVER:" XRDAP_FIRMWARE_VERSION;
static_assert(sizeof(Slots::kVersionTag) == 13U, "the literal above starts with the tag");

alignas(64) uint8_t ep0_buffer[64];

// Same VID:PID as the CH32V305 XR-DAP bootloader, so the existing host tools apply.
constexpr uint16_t kUsbVid = 0x1209;
constexpr uint16_t kUsbPid = 0x3072;

bool ConsumeBootRequest()
{
  auto* mailbox = reinterpret_cast<volatile uint32_t*>(XRDAP_BOOT_MAILBOX_ADDR);
  const bool requested =
      mailbox[0] == XRDAP_BOOT_REQUEST_MAGIC0 && mailbox[1] == XRDAP_BOOT_REQUEST_MAGIC1;
  mailbox[0] = 0U;
  mailbox[1] = 0U;
  return requested;
}

void ConfigurePin(const XrDap::BoardPin& pin, GPIOMode_TypeDef mode)
{
  RCC_APB2PeriphClockCmd(XrDap::PortClock(pin.port), ENABLE);
  GPIO_InitTypeDef init = {};
  init.GPIO_Pin = pin.Mask();
  init.GPIO_Speed = GPIO_Speed_10MHz;
  init.GPIO_Mode = mode;
  GPIO_Init(pin.Port(), &init);
}

bool BootKeyHeld()
{
  ConfigurePin(Board::kBootKey, GPIO_Mode_IPU);
  LibXR::Timebase::DelayMicroseconds(100);  // let the pull-up settle
  return GPIO_ReadInputDataBit(Board::kBootKey.Port(), Board::kBootKey.Mask()) ==
         Bit_RESET;
}

// The CH32 startup code puts `j handle_reset` at the image base and the vector table
// right after it, whose first entry is the image base itself.
bool AppVectorsPlausible()
{
  const auto* words = reinterpret_cast<const volatile uint32_t*>(kAppBase);
  const uint32_t first = words[0];
  return first != 0xFFFFFFFFU && first != 0U && words[1] == kAppBase;
}

uint32_t OwnSequence()
{
  const auto* record = reinterpret_cast<const Slots::Record*>(Slots::SlotBase(kSelf) +
                                                              Slots::kRecordOffset);
  const uint32_t first = *reinterpret_cast<const uint32_t*>(Slots::SlotBase(kSelf));
  return Slots::RecordValid(*record, kSelf, first) ? record->seq : 0U;
}

void RecordRunningSlot()
{
  auto* word = reinterpret_cast<volatile uint32_t*>(XRDAP_BOOT_SLOT_ADDR);
  word[0] = XRDAP_BOOT_SLOT_MAGIC;
  word[1] = static_cast<uint32_t>(kSelf);
}

// Restarts in DFU mode (boot mailbox), so that stage 1 starts the newest bootloader and
// it stays in DFU for the application download that usually follows.
[[noreturn]] void RestartInDfu()
{
  auto* mailbox = reinterpret_cast<volatile uint32_t*>(XRDAP_BOOT_MAILBOX_ADDR);
  mailbox[0] = XRDAP_BOOT_REQUEST_MAGIC0;
  mailbox[1] = XRDAP_BOOT_REQUEST_MAGIC1;
  USBHSD->CONTROL = USBHS_UC_CLR_ALL | USBHS_UC_RESET_SIE;  // drops the pull-up
  NVIC_SystemReset();
  for (;;)
  {
  }
}

// Leaves the bus and starts the application with the core as close to its reset state
// as possible; the application's USB stack enumerates afresh.
[[noreturn]] void JumpToApp()
{
  RecordRunningSlot();
  NVIC_DisableIRQ(USBHS_IRQn);
  NVIC_DisableIRQ(USBHSWakeup_IRQn);
  USBHSD->INT_EN = 0U;
  USBHSD->CONTROL = USBHS_UC_CLR_ALL | USBHS_UC_RESET_SIE;  // also drops the pull-up
  USBHSD->CONTROL = 0U;

  __disable_irq();
  SysTick->CTLR = 0U;
  SysTick->SR = 0U;
  for (size_t i = 0; i < sizeof(PFIC->IRER) / sizeof(PFIC->IRER[0]); ++i)
  {
    PFIC->IRER[i] = 0xFFFFFFFFU;
    PFIC->IPRR[i] = 0xFFFFFFFFU;
  }
  reinterpret_cast<void (*)()>(kAppBase)();
  for (;;)
  {
  }
}

void JumpToAppThunk(void*) { JumpToApp(); }
void NoLaunch(void*) {}

void BlinkLed(uint32_t now_ms)
{
#ifdef XRDAP_HAS_LED
  const bool on = ((now_ms / kLedPeriodMs) & 1U) != 0U;
  GPIO_WriteBit(Board::kLed.Port(), Board::kLed.Mask(), on ? Bit_SET : Bit_RESET);
#else
  (void)now_ms;
#endif
}
}  // namespace

extern "C" void bootloader_main()
{
  LibXR::CH32Timebase timebase;
  LibXR::PlatformInit();

  static LibXR::CH32Flash flash(kFlashRegions, 1, kFlashBase + kAppBase);
  static AppWindowFlash app_window(flash);
  static LibXR::USB::DfuBootloaderClass dfu(app_window, 0U, kAppSize, Layout::kSealOffset,
                                            JumpToAppThunk, nullptr, true,
                                            "XR-DAP application");
  static LibXR::CH32Flash peer_flash_raw(kFlashRegions, 1, kFlashBase + kPeerBase);
  static PeerSlotFlash peer_flash(peer_flash_raw, OwnSequence() + 1U);
  // The DFU region is the whole slot, so that its erase map covers the record page;
  // downloads stop in front of the record (LibXR's payload limit is the seal offset).
  static LibXR::USB::DfuBootloaderClass peer_dfu(
      peer_flash, 0U, Slots::kSlotSize, Slots::kRecordOffset, NoLaunch, nullptr, false,
      kSelf == Slots::Slot::A ? "XR-DAP bootloader slot B" : "XR-DAP bootloader slot A");
  // Keeps the version marker in the image (--gc-sections).
  asm volatile("" : : "r"(xrdap_bl_version));

  const bool requested = ConsumeBootRequest();
  if (!requested && !BootKeyHeld() && AppVectorsPlausible() && dfu.HasValidImage())
  {
    JumpToApp();
  }

#ifdef XRDAP_HAS_LED
  ConfigurePin(Board::kLed, GPIO_Mode_Out_PP);
#endif

  static constexpr auto kStrings = LibXR::USB::DescriptorStrings::MakeLanguagePack(
      LibXR::USB::DescriptorStrings::Language::EN_US, XrDap::kVendorName,
      "XR-DAP-V307 DFU " XRDAP_FIRMWARE_VERSION, XrDap::kSerialPrefix);
  static LibXR::CH32USBOtgHS usb({{ep0_buffer}}, {{ep0_buffer}}, kUsbVid, kUsbPid,
                                 XRDAP_VERSION_BCD, {&kStrings}, {{&dfu, &peer_dfu}},
                                 {XrDap::ChipUid(), XrDap::kChipUidSize});
  usb.Init(false);
  usb.Start(false);

  uint32_t closed_at_ms = 0U;
  bool restart_pending = false;
  for (;;)
  {
    const auto now_ms = static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    BlinkLed(now_ms);
    dfu.Process();
    peer_dfu.Process();
    if (peer_flash.TakeClosed())
    {
      closed_at_ms = now_ms;
      restart_pending = true;
    }
    if (restart_pending && now_ms - closed_at_ms >= kRestartAfterSlotMs)
    {
      RestartInDfu();
    }
    if (dfu.TryConsumeAppLaunch(now_ms) && AppVectorsPlausible() && dfu.HasValidImage())
    {
      JumpToApp();
    }
    // dfu.Process() erases and programs the flash in this loop; one pass per millisecond
    // keeps up with the DFU poll timeout.
    LibXR::Thread::Sleep(1);
  }
}
