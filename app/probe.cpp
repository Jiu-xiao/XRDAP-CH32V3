// SPDX-License-Identifier: Apache-2.0
#include "probe.hpp"

#include "libxr.hpp"
#include "probe_info.hpp"
#include "reserved_ram.h"
#include "target_reset.hpp"
#include "xrdap/control/control.hpp"

namespace XrDap
{

namespace
{

// SWD 位操作时序，CPU 周期 x10（LibXR SwdGpioTiming），每板标定 / SWD bit-bang timing in
// CPU cycles x10 (LibXR SwdGpioTiming), calibrated per board.
constexpr LibXR::Debug::SwdGpioTiming kSwdTiming{
    Board::kSwdStepX10, Board::kSwdInterceptX10, Board::kSwdFastX10};
// 没有部件有事时主循环最长等多久。 / Longest wait of the main loop when no part has
// work.
constexpr uint32_t kStepWaitMs = 10;

char serial[SerialBufferSize(kChipUidSize)];

constexpr auto kStrings = LibXR::USB::DescriptorStrings::MakeLanguagePack(
    LibXR::USB::DescriptorStrings::Language::EN_US, kVendorName, kUsbProductName,
    kSerialPrefix);

// DFU DETACH：在启动信箱里留下请求，复位进入引导程序。 / DFU DETACH: leave a request in
// the boot mailbox and reset into the bootloader.
void RestartIntoBootloader(void*)
{
  auto* mailbox = reinterpret_cast<volatile uint32_t*>(XRDAP_BOOT_MAILBOX_ADDR);
  mailbox[0] = XRDAP_BOOT_REQUEST_MAGIC0;
  mailbox[1] = XRDAP_BOOT_REQUEST_MAGIC1;
  NVIC_SystemReset();
}

}  // namespace

Probe::DfuRuntime::DfuRuntime()
    : ResetCountingUsb<LibXR::USB::DfuRuntimeClass>(RestartIntoBootloader)
{
}

Probe::BoardInit::BoardInit() { Board::Init(); }

Probe::PlatformInit::PlatformInit() { LibXR::PlatformInit(); }

Probe::Probe()
    : swd_(swclk_, swdio_, XRDAP_APP_SYSCLK_HZ, kSwdTiming),
      nreset_(TargetResetOutput(Board::kNreset)),
      dap_(kDapInEndpoint, kDapOutEndpoint, swd_, &nreset_),
      vendor_(swd_),
      exec_(dap_, vendor_, activity_),
      tick_(vendor_),
      usb_control_(dap_, exec_),
      usb_(Family::kUsbVid, Family::kUsbPid, kUsbBcd, {&kStrings}, {{&dap_, &dfu_}},
           {ChipUid(), kChipUidSize})
{
  FormatSerial(ChipUid(), kChipUidSize, serial);
  SetProbeInfo(dap_, {kProductName, Board::kName, serial, kFirmwareVersion});
  // USB 总线复位让连在 USB 上的连接作废。 / A USB bus reset lapses a connection on USB.
  vendor_.SetUsbResetSource(&dfu_.BusResets());
  exec_.SetControl(&Probe::HandleControl, this);
  usb_.Init(false);
  usb_.Start(false);
}

uint16_t Probe::HandleControl(void* ctx, const uint8_t* req, uint16_t len, uint8_t* resp,
                              uint16_t cap)
{
  const auto* self = static_cast<const Probe*>(ctx);
  uint16_t out = 0;
  for (size_t i = 0; i < self->part_count_; ++i)
  {
    if (self->parts_[i]->Control(req, len, resp, cap, out))
    {
      return out;
    }
  }
  return Control::StatusOnly(req[0], Control::kStatusUnsupported, resp, cap);
}

void Probe::SetForwarder(Exec::Forwarder forwarder, void* ctx)
{
  exec_.SetForwarder(forwarder, ctx);
}

void Probe::ExecuteHere(Transport t, const uint8_t* req, uint16_t len, uint8_t* resp,
                        uint16_t cap, uint16_t& out)
{
  exec_.ExecuteHere(t, req, len, resp, cap, out);
}

void Probe::ControlHere(const uint8_t* req, uint16_t len, uint8_t* resp, uint16_t cap,
                        uint16_t& out)
{
  exec_.ControlHere(req, len, resp, cap, out);
}

void Probe::LinkLost(Transport t) { exec_.LinkLost(t); }

void Probe::Run(std::initializer_list<Part*> parts)
{
  parts_ = parts.begin();
  part_count_ = parts.size();
  for (;;)
  {
    guard_.Feed();
    bool busy = false;
    for (Part* part : parts)
    {
      busy = part->Poll() || busy;
    }
    exec_.Run(tick_, busy ? 0U : kStepWaitMs);
    dfu_.Process();
  }
}

}  // namespace XrDap
