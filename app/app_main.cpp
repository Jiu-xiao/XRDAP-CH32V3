// SPDX-License-Identifier: Apache-2.0
// XR-DAP-V307: a CMSIS-DAP v2 debug probe on the CH32V307 (and the CH32V305 board),
// built on LibXR.
//   USBHS: a Bulk v2 DAP interface plus a DFU runtime interface; DFU DETACH restarts
//          into the bootloader for a firmware update.
//
// XR-DAP-V307：基于 LibXR 的 CH32V307（以及 CH32V305 板）CMSIS-DAP v2 调试探针。
//   USBHS：Bulk v2 DAP 接口，外加 DFU runtime 接口；DFU DETACH 重启进入 bootloader 升级。

#include <cstdint>

#include XRDAP_BOARD_HEADER
#include "boot_guard.hpp"
#include "ch32_gpio.hpp"
#include "ch32_timebase.hpp"
#include "ch32_usb_dev.hpp"
#include "config.hpp"
#include "daplink_v2_profile_swd.hpp"
#include "debug/swd_general_gpio.hpp"
#include "dfu/dfu_runtime.hpp"
#include "fast_gpio.hpp"
#include "libxr.hpp"
#include "probe_info.hpp"
#include "reserved_ram.h"
#include "sampler_task.hpp"
#include "xrdap/session/executor.hpp"
#ifdef XRDAP_HAS_STATUS_LIGHTS
#include "status_lights.hpp"
#endif
#ifdef XRDAP_HAS_STATUS_LED
#include "status_led.hpp"
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

constexpr uint16_t kDapPacketSize = 1024;
constexpr uint8_t kDapPacketCount = XRDAP_DAP_PACKET_COUNT;  // per board (cmake)
constexpr uint32_t kSamplerPriority = 5;
#ifdef XRDAP_HAS_STATUS_LIGHTS
// The DAP loop runs at 3 and does not block while the radio is in use; the light task
// shares its priority so time slicing still runs it.
constexpr uint32_t kLightsPriority = 3;
#endif
// SWD bit-bang timing in CPU cycles (LibXR SwdGpioTiming, x10), per board: one SWCLK
// period on the delayed path costs a + b * N cycles, N delay loops per half period; the
// path without delay costs c cycles per period.
constexpr LibXR::Debug::SwdGpioTiming kSwdTiming{
    Board::kSwdStepX10, Board::kSwdInterceptX10, Board::kSwdFastX10};

using Swclk = FastGpioFor<Board::kSwclk>;
using Swdio = FastGpioFor<Board::kSwdio>;
using Swd = LibXR::Debug::SwdGeneralGPIO<Swclk, Swdio>;
using DapClass =
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
  // The watchdog first: a hang anywhere after this point ends in a watchdog reset, which
  // on v305 leaves the board in its bootloader (autorun off) and on the CH32V307 boards
  // restarts the sealed application.
  BootGuardStart();
  Board::Init();
  static LibXR::CH32Timebase timebase;
  LibXR::PlatformInit(3, 8192);
  ConfigInit();

  static Swclk swclk;
  static Swdio swdio;
  static Swd swd(swclk, swdio, XRDAP_APP_SYSCLK_HZ, kSwdTiming);
  static LibXR::CH32GPIO nreset(Board::kNreset.Port(), Board::kNreset.Mask());
  nreset.Write(true);

  static DapClass dap(LibXR::USB::Endpoint::EPNumber::EP1,
                      LibXR::USB::Endpoint::EPNumber::EP1, swd, &nreset);
  static char serial[SerialBufferSize(kChipUidSize)];
  FormatSerial(ChipUid(), kChipUidSize, serial);
  SetProbeInfo(dap, {kProductName, Board::kName, serial, kFirmwareVersion});
  static DfuRuntime dfu(RestartIntoBootloader);
  // XRDAP-Core: one transport owns the probe at a time (USB, or the radio peer on a
  // remote), every DAP command goes through the executor, and the XR vendor extension
  // (multi-address read/write, probe-side sampling) runs behind it. A USB bus reset ends
  // the USB session. The sampling task outranks the DAP loop (priority 3).
  static SessionOwner session;
  session.SetUsbResetSource(&DfuRuntime::BusResets, &dfu);
  static DapVendor vendor(swd, session);
  static DapActivity activity;
  static Executor<DapClass> exec(dap, session, vendor, activity);
  StartSamplerTask(vendor, kSamplerPriority);
#ifdef XRDAP_HAS_STATUS_LIGHTS
  StatusLightsBegin(
      kLightsPriority, activity,
      [](void* d) { return static_cast<DapClass*>(d)->IsInited(); }, &dap);
#endif
#ifdef XRDAP_HAS_STATUS_LED
  StatusLedBegin(
      activity, [](void* d) { return static_cast<DapClass*>(d)->IsInited(); }, &dap);
#endif
#ifdef XRDAP_HAS_RADIO
  Radio().Init();
  // On a remote, the peer's commands are the RADIO transport; its session ends with the
  // link's session (B6), which ends RADIO's ownership and the sampling it started.
  DapSessionInit(
      +[](const uint8_t* req, uint16_t req_len, uint8_t* resp, uint16_t resp_cap)
      {
        uint16_t size = 0;
        exec.Execute(Transport::RADIO, req, req_len, resp, resp_cap, size);
        return size;
      },
      +[]() { exec.EndSession(Transport::RADIO); });
  exec.SetFreeCommands(&kSettingsQueryFree, 1U);
  // After the owner check: 0x9F is this probe's radio and settings; on the host, the
  // session layer forwards USB commands over the radio during a wireless session.
  exec.SetForwarder(
      +[](void*, Transport t, const uint8_t* req, uint16_t len, uint8_t* resp,
          uint16_t cap, uint16_t& out)
      {
        if (req[0] == kProbeCommand)
        {
          out = RadioVendorCommand(req, len, resp, cap);
          return true;
        }
        return t == Transport::USB &&
               DapSessionHandleUsbCommand(req, len, resp, cap, out);
      },
      nullptr);
  DapSessionApplyConfig();
#endif

  static constexpr auto kStrings = LibXR::USB::DescriptorStrings::MakeLanguagePack(
      LibXR::USB::DescriptorStrings::Language::EN_US, kVendorName, kUsbProductName,
      kSerialPrefix);
  static LibXR::CH32USBOtgHS usb(
      {{ep0_buffer}, {ep1_dap_out}}, {{ep0_buffer}, {ep1_dap_in}}, Family::kUsbVid,
      Family::kUsbPid, kUsbBcd, {&kStrings}, {{&dap, &dfu}}, {ChipUid(), kChipUidSize});
  usb.Init(false);
  usb.Start(false);

  // StepUsb() waits up to kStepWaitMs for a USB command (libxr WaitForWork, without
  // holding the executor's turn) and then runs it, so a DFU DETACH is carried out from
  // the same loop soon after its advertised timeout, and the status LED is drawn at
  // least every kStepWaitMs. With the radio in use the loop does
  // not wait: it serves the radio between USB commands and watches KEY2 for pairing.
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
      (void)exec.StepUsb(0);
      dfu.Process();
      continue;
    }
#endif
    (void)exec.StepUsb(kStepWaitMs);
    dfu.Process();
#ifdef XRDAP_HAS_STATUS_LED
    StatusLedUpdate();
#endif
  }
}
