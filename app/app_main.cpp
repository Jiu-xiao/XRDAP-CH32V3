// SPDX-License-Identifier: Apache-2.0
// XR-DAP-V307: a CMSIS-DAP v2 debug probe on the CH32V307, built on LibXR.
//   USBHS: a Bulk v2 DAP interface plus a DFU runtime interface; DFU DETACH restarts
//          into the bootloader for a firmware update.
//
// XR-DAP-V307：基于 LibXR 的 CH32V307 CMSIS-DAP v2 调试探针。
//   USBHS：Bulk v2 DAP 接口，外加 DFU runtime 接口；DFU DETACH 重启进入 bootloader 升级。

#include <cstdint>

#include XRDAP_BOARD_HEADER
#include "boot_mailbox.h"
#include "ch32_gpio.hpp"
#include "ch32_timebase.hpp"
#include "ch32_usb_dev.hpp"
#include "daplink_v2_profile_swd.hpp"
#include "debug/swd_general_gpio.hpp"
#include "dfu/dfu_runtime.hpp"
#include "fast_gpio.hpp"
#include "libxr.hpp"
#include "probe_info.hpp"
#include "thread.hpp"

namespace
{
using namespace XrDap;

constexpr uint16_t kUsbVid = 0x1D50;  // XRUSB VID:PID, shared by the XR-DAP family
constexpr uint16_t kUsbPid = 0x6199;
constexpr uint16_t kUsbBcd = 0x0300;  // distinct from the ESP32 XR-DAP releases
constexpr uint16_t kDapPacketSize = 1024;
constexpr uint8_t kDapPacketCount = 4;
// Delay-loop iterations per microsecond of the SWD half period at 144 MHz. One
// iteration takes about 21.5 ns on top of a fixed ~100 ns per half period; 46 still
// ran 0.6 % fast at 50 kHz, 47 keeps every setting at or below the requested clock.
constexpr uint32_t kSwdLoopsPerUs = 47;

using Swclk = FastGpioFor<Board::kSwclk>;
using Swdio = FastGpioFor<Board::kSwdio>;
using Swd = LibXR::Debug::SwdGeneralGPIO<Swclk, Swdio>;
using Dap = LibXR::USB::DapLinkV2Class<Swd, kDapPacketSize, kDapPacketCount, kDapPacketSize>;

alignas(64) uint8_t ep0_buffer[64];
alignas(64) uint8_t ep1_dap_in[2 * kDapPacketSize];
alignas(64) uint8_t ep1_dap_out[2 * kDapPacketSize];

// DFU DETACH: leave a request in the boot mailbox and reset into the bootloader.
void RestartIntoBootloader(void*)
{
  auto* mailbox = reinterpret_cast<volatile uint32_t*>(XRDAP_BOOT_MAILBOX_ADDR);
  mailbox[0] = XRDAP_BOOT_REQUEST_MAGIC0;
  mailbox[1] = XRDAP_BOOT_REQUEST_MAGIC1;
  NVIC_SystemReset();
}
}  // namespace

extern "C" void app_main()
{
  static LibXR::CH32Timebase timebase;
  LibXR::PlatformInit(3, 8192);

  static Swclk swclk;
  static Swdio swdio;
  static Swd swd(swclk, swdio, kSwdLoopsPerUs);
  static LibXR::CH32GPIO nreset(Board::kNreset.Port(), Board::kNreset.Mask());
  nreset.Write(true);

  static Dap dap(LibXR::USB::Endpoint::EPNumber::EP1, LibXR::USB::Endpoint::EPNumber::EP1,
                 swd, &nreset);
  SetProbeInfo(dap, Board::kName);
  static LibXR::USB::DfuRuntimeClass dfu(RestartIntoBootloader);

  static constexpr auto kStrings = LibXR::USB::DescriptorStrings::MakeLanguagePack(
      LibXR::USB::DescriptorStrings::Language::EN_US, kVendorName, kUsbProductName,
      kSerialPrefix);
  static LibXR::CH32USBOtgHS usb({{ep0_buffer}, {ep1_dap_out}}, {{ep0_buffer}, {ep1_dap_in}},
                                 kUsbVid, kUsbPid, kUsbBcd, {&kStrings}, {{&dap, &dfu}},
                                 {ChipUid(), kChipUidSize});
  usb.Init(false);
  usb.Start(false);

  // LibXR runs DAP commands when the application calls Step(). Step waits at most
  // kStepWaitMs for a command, so a DFU DETACH is carried out from the same loop soon
  // after its advertised timeout.
  constexpr uint32_t kStepWaitMs = 10;
  for (;;)
  {
    (void)dap.Step(kStepWaitMs);
    dfu.Process();
  }
}
