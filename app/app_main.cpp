// SPDX-License-Identifier: Apache-2.0
// XR-DAP-V307: a CMSIS-DAP v2 debug probe on the CH32V307, built on LibXR.
//   USBHS: a Bulk v2 DAP interface plus a DFU runtime interface; DFU DETACH restarts
//          into the bootloader for a firmware update.
//
// XR-DAP-V307：基于 LibXR 的 CH32V307 CMSIS-DAP v2 调试探针。
//   USBHS：Bulk v2 DAP 接口，外加 DFU runtime 接口；DFU DETACH 重启进入 bootloader 升级。

#include <cstdint>

#include XRDAP_BOARD_HEADER
#include "boot_guard.hpp"
#include "ch32_gpio.hpp"
#include "ch32_timebase.hpp"
#include "ch32_usb_dev.hpp"
#include "config.hpp"
#include "dap_vendor.hpp"
#include "daplink_v2_profile_swd.hpp"
#include "debug/swd_general_gpio.hpp"
#include "dfu/dfu_runtime.hpp"
#include "fast_gpio.hpp"
#include "libxr.hpp"
#include "probe_info.hpp"
#include "reserved_ram.h"
#include "sampler_task.hpp"
#ifdef XRDAP_HAS_STATUS_LIGHTS
#include "status_lights.hpp"
#endif
#ifdef XRDAP_HAS_RADIO
#include "dap_session.hpp"
#include "pairing.hpp"
#include "radio_command.hpp"
#include "sx1281.hpp"
#endif

namespace
{
using namespace XrDap;

constexpr uint16_t kUsbVid = 0x1D50;  // XRUSB VID:PID, shared by the XR-DAP family
constexpr uint16_t kUsbPid = 0x6199;
constexpr uint16_t kUsbBcd = 0x0300;  // distinct from the ESP32 XR-DAP releases
constexpr uint16_t kDapPacketSize = 1024;
constexpr uint8_t kDapPacketCount = 4;
constexpr uint32_t kSamplerPriority = 5;
#ifdef XRDAP_HAS_STATUS_LIGHTS
// The DAP loop runs at 3 and does not block while the radio is in use; the light task
// shares its priority so time slicing still runs it.
constexpr uint32_t kLightsPriority = 3;
#endif
// Delay-loop iterations per microsecond of the SWD half period, as LibXR's
// SwdGeneralGPIO counts them. A half period takes a fixed ~60 ns plus ~14.6 ns per
// iteration at 216 MHz, which LibXR does not model. The SWD clock does not need to be
// exact: with this factor settings from ~24 MHz up use the no-delay path (~14 MHz on
// wdap), and slow settings run up to ~40 % fast. Other core clocks scale it.
constexpr uint32_t kSwdLoopsPerUs =
    static_cast<uint32_t>((48ULL * XRDAP_APP_SYSCLK_HZ) / 216'000'000ULL);

using Swclk = FastGpioFor<Board::kSwclk>;
using Swdio = FastGpioFor<Board::kSwdio>;
using Swd = LibXR::Debug::SwdGeneralGPIO<Swclk, Swdio>;
using Dap =
    LibXR::USB::DapLinkV2Class<Swd, kDapPacketSize, kDapPacketCount, kDapPacketSize>;

alignas(64) uint8_t ep0_buffer[64];
alignas(64) uint8_t ep1_dap_in[2 * kDapPacketSize];
alignas(64) uint8_t ep1_dap_out[2 * kDapPacketSize];

// The DFU runtime interface, also counting USB bus resets (libxr reports them to every
// class): sampling started from USB stops when the bus resets, as the host is gone.
class DfuRuntime : public LibXR::USB::DfuRuntimeClass
{
 public:
  using DfuRuntimeClass::DfuRuntimeClass;

  static uint32_t BusResets(void* ctx) { return static_cast<DfuRuntime*>(ctx)->resets_; }

  void OnDeviceEvent(bool in_isr, LibXR::USB::DeviceEvent event,
                     uint8_t endpoint) override
  {
    if (event == LibXR::USB::DeviceEvent::RESET)
    {
      resets_ = resets_ + 1U;
    }
    DfuRuntimeClass::OnDeviceEvent(in_isr, event, endpoint);
  }

 private:
  volatile uint32_t resets_ = 0;
};

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
  BootGuardStart();
  ConfigInit();

  static Swclk swclk;
  static Swdio swdio;
  static Swd swd(swclk, swdio, kSwdLoopsPerUs);
  static LibXR::CH32GPIO nreset(Board::kNreset.Port(), Board::kNreset.Mask());
  nreset.Write(true);

  static Dap dap(LibXR::USB::Endpoint::EPNumber::EP1, LibXR::USB::Endpoint::EPNumber::EP1,
                 swd, &nreset);
  static char serial[SerialBufferSize(kChipUidSize)];
  FormatSerial(ChipUid(), kChipUidSize, serial);
  SetProbeInfo(dap, {kProductName, Board::kName, serial, kFirmwareVersion});
  static DfuRuntime dfu(RestartIntoBootloader);
  // XR vendor extension (0x80..0x85: multi-address read/write, probe-side sampling) from
  // XRDAP-Core. Its sampling task outranks the DAP loop (priority 3).
  static XrDap::DapVendor vendor(swd, kDapPacketSize);
  vendor.SetUsbResetSource(&DfuRuntime::BusResets, &dfu);
  StartSamplerTask(vendor, kSamplerPriority);
#ifdef XRDAP_HAS_STATUS_LIGHTS
  StatusLightsBegin(
      kLightsPriority, vendor, [](void* d) { return static_cast<Dap*>(d)->IsInited(); },
      &dap);
#endif
#ifdef XRDAP_HAS_RADIO
  Radio().Init();
  // Commands from the peer: 0x9F for this probe's radio and settings, DAP for the target.
  // When the remote session ends, what the peer started stops with it (B6). Until
  // XRDAP-Core has a radio transport, "what the peer started" is sampling begun while a
  // radio session drove the probe.
  static bool peer_drove_probe = false;
  DapSessionInit(
      +[](const uint8_t* req, uint16_t req_len, uint8_t* resp, uint16_t resp_cap)
      {
        if (req[0] == kProbeCommand)
          return RadioVendorCommand(req, req_len, resp, resp_cap);
        peer_drove_probe = true;
        uint16_t size = 0;
        (void)dap.ExecuteExternal(req, req_len, resp, resp_cap, size);
        return size;
      },
      +[]()
      {
        if (!peer_drove_probe) return;
        peer_drove_probe = false;
        vendor.OnTransportClosed();
      });
  // The radio link takes 0x9F (this probe's radio and settings, not DAP) and, during a
  // session over the radio, every command; those forwarded still count as DAP activity.
  // What runs here goes to the vendor extension, which passes standard commands on.
  dap.SetCommandHook(
      +[](void* ctx, const uint8_t* req, uint16_t req_len, uint8_t* resp,
          uint16_t resp_cap, uint16_t& out_len)
      {
        if (RadioCommandHook(req, req_len, resp, resp_cap, out_len))
        {
          if (req_len != 0U && req[0] != kProbeCommand)
          {
            static_cast<XrDap::DapVendor*>(ctx)->Observe(req, req_len);
          }
          return true;
        }
        return XrDap::DapVendor::Hook(ctx, req, req_len, resp, resp_cap, out_len);
      },
      &vendor);
  DapSessionApplyConfig();
#else
  dap.SetCommandHook(&XrDap::DapVendor::Hook, &vendor);
#endif

  static constexpr auto kStrings = LibXR::USB::DescriptorStrings::MakeLanguagePack(
      LibXR::USB::DescriptorStrings::Language::EN_US, kVendorName, kUsbProductName,
      kSerialPrefix);
  static LibXR::CH32USBOtgHS usb({{ep0_buffer}, {ep1_dap_out}},
                                 {{ep0_buffer}, {ep1_dap_in}}, kUsbVid, kUsbPid, kUsbBcd,
                                 {&kStrings}, {{&dap, &dfu}}, {ChipUid(), kChipUidSize});
  usb.Init(false);
  usb.Start(false);

  // LibXR runs DAP commands when the application calls Step(). Step waits at most
  // kStepWaitMs for a command, so a DFU DETACH is carried out from the same loop soon
  // after its advertised timeout. With the radio in use the loop does not wait: it
  // serves the radio between USB commands and watches KEY2 for pairing.
  constexpr uint32_t kStepWaitMs = 10;
  for (;;)
  {
    BootGuardFeed();
#ifdef XRDAP_HAS_RADIO
    // Pairing takes the radio over; the settings then apply again either way.
    if (PairingPoll()) DapSessionApplyConfig();
    if (DapSessionRadioInUse())
    {
      DapSessionService();
      (void)dap.Step(0);
      dfu.Process();
      continue;
    }
#endif
    (void)dap.Step(kStepWaitMs);
    dfu.Process();
  }
}
