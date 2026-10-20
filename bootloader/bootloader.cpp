// SPDX-License-Identifier: Apache-2.0
// XR-DAP-V307 USB DFU bootloader.
//
// It occupies the start of the flash and decides at every reset whether to start the
// application or stay in DFU mode. It stays when the application asked for it through
// the boot mailbox (DFU DETACH), when the board's boot key is held, or when the
// application window holds no valid sealed image. In DFU mode it exposes a standard
// USB DFU interface on USBHS (dfu-util works; WinUSB is announced for Windows); a
// completed download writes the seal record and starts the new image.
//
// XR-DAP-V307 的 USB DFU bootloader。
// 位于 Flash 起始处，每次复位时决定启动应用还是停留在 DFU 模式。应用经启动信箱请求
// （DFU DETACH）、上电时按住板上的 bootloader 键，或应用区没有有效的带 seal 镜像时停留。
// DFU 模式下在 USBHS 上提供标准 USB DFU 接口（可用 dfu-util，Windows 上自动使用
// WinUSB）；下载完成后写入 seal 记录并启动新镜像。

#include <cstddef>
#include <cstdint>

#include XRDAP_BOARD_HEADER
#include "boot_mailbox.h"
#include "ch32_flash.hpp"
#include "ch32_timebase.hpp"
#include "ch32_usb_dev.hpp"
#include "ch32v30x.h"
#include "dfu/dfu_bootloader.hpp"
#include "libxr.hpp"
#include "thread.hpp"

namespace
{
namespace Board = XrDap::Board;

constexpr uint32_t kFlashBase = 0x08000000U;  // flash as seen by the flash controller
constexpr uint32_t kPageSize = LibXR::CH32Flash::PageSize();
constexpr uint32_t kAppBase = XRDAP_APP_BASE;  // CPU address of the application
constexpr uint32_t kAppSize = XRDAP_APP_SIZE;
constexpr uint32_t kSealOffset = kAppSize - XRDAP_SEAL_SIZE;
constexpr uint32_t kLedPeriodMs = 250U;

static_assert(kAppBase % kPageSize == 0U && kAppSize % kPageSize == 0U,
              "the application window must be page aligned");

// Pages from the flash start to the end of the application window; the DFU storage
// area is the application window at its end.
constexpr LibXR::FlashRegion kFlashRegions[] = {
    {kFlashBase, kPageSize, (kAppBase + kAppSize) / kPageSize}};

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
  return GPIO_ReadInputDataBit(Board::kBootKey.Port(), Board::kBootKey.Mask()) == Bit_RESET;
}

// The CH32 startup code puts `j handle_reset` at the image base and the vector table
// right after it, whose first entry is the image base itself.
bool AppVectorsPlausible()
{
  const auto* words = reinterpret_cast<const volatile uint32_t*>(kAppBase);
  const uint32_t first = words[0];
  return first != 0xFFFFFFFFU && first != 0U && words[1] == kAppBase;
}

// Leaves the bus and starts the application with the core as close to its reset state
// as possible; the application's USB stack enumerates afresh.
[[noreturn]] void JumpToApp()
{
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

void BlinkLed(uint32_t now_ms)
{
  if constexpr (Board::kHasLed)
  {
    const bool on = ((now_ms / kLedPeriodMs) & 1U) != 0U;
    GPIO_WriteBit(Board::kLed.Port(), Board::kLed.Mask(), on ? Bit_SET : Bit_RESET);
  }
}
}  // namespace

extern "C" void bootloader_main()
{
  LibXR::CH32Timebase timebase;
  LibXR::PlatformInit();

  static LibXR::CH32Flash flash(kFlashRegions, 1, kFlashBase + kAppBase);
  static LibXR::USB::DfuBootloaderClass dfu(flash, 0U, kAppSize, kSealOffset, JumpToAppThunk);

  const bool requested = ConsumeBootRequest();
  if (!requested && !BootKeyHeld() && AppVectorsPlausible() && dfu.HasValidImage())
  {
    JumpToApp();
  }

  if constexpr (Board::kHasLed)
  {
    ConfigurePin(Board::kLed, GPIO_Mode_Out_PP);
  }

  static constexpr auto kStrings = LibXR::USB::DescriptorStrings::MakeLanguagePack(
      LibXR::USB::DescriptorStrings::Language::EN_US, "XRobot", "XR-DAP-V307 DFU",
      "XRobot-XRDAP-");
  static LibXR::CH32USBOtgHS usb({{ep0_buffer}}, {{ep0_buffer}}, kUsbVid, kUsbPid, 0x0100,
                                 {&kStrings}, {{&dfu}},
                                 {reinterpret_cast<void*>(0x1FFFF7E8), 12});
  usb.Init(false);
  usb.Start(false);

  for (;;)
  {
    const auto now_ms = static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    BlinkLed(now_ms);
    dfu.Process();
    if (dfu.TryConsumeAppLaunch(now_ms) && AppVectorsPlausible() && dfu.HasValidImage())
    {
      JumpToApp();
    }
    // Bare-metal LibXR runs its timer tasks inside Sleep, and the DFU backend erases and
    // programs the flash from a 1 ms timer task, so every pass sleeps one tick.
    LibXR::Thread::Sleep(1);
  }
}
