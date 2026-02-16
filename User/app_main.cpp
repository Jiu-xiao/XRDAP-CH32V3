#include <cstdint>

#include "cdc_uart.hpp"
#include "ch32_gpio.hpp"
#include "ch32_timebase.hpp"
#include "ch32_usb.hpp"
#include "ch32_usb_dev.hpp"
#include "daplink_v2.hpp"
#include "debug/swd_general_gpio.hpp"
#include "libxr.hpp"
#include "thread.hpp"

namespace
{
static uint8_t ep0_buffer_hs[64];
static uint8_t ep1_dap_in_hs[1024];
static uint8_t ep2_dap_out_hs[1024];
static uint8_t ep3_cdc_in_hs[1024];
static uint8_t ep4_cdc_out_hs[1024];
static uint8_t ep5_cdc_comm_in_hs[256];
}  // namespace

template <>
LibXR::ErrorCode
LibXR::Debug::SwdGeneralGPIO<LibXR::CH32GPIO, LibXR::CH32GPIO>::SetClockHz(uint32_t hz)
{
  if (hz == 0u)
  {
    clock_hz_ = 0u;
    half_period_ns_ = 0u;
    half_period_loops_ = 0u;
    return LibXR::ErrorCode::OK;
  }

  if (hz < MIN_HZ)
  {
    hz = MIN_HZ;
  }
  if (hz > MAX_HZ)
  {
    hz = MAX_HZ;
  }

  clock_hz_ = hz;

  const uint32_t den = 2u * hz;
  half_period_ns_ = (NS_PER_SEC + den - 1u) / den;

  if (loops_per_us_ == 0u)
  {
    half_period_loops_ = 0u;
    return LibXR::ErrorCode::OK;
  }

  const uint64_t loops_num =
      static_cast<uint64_t>(loops_per_us_) * static_cast<uint64_t>(half_period_ns_);
  if (loops_num < LOOPS_SCALE)
  {
    half_period_loops_ = 0u;
    return LibXR::ErrorCode::OK;
  }

  uint64_t loops_ceil = (loops_num + CEIL_BIAS) / LOOPS_SCALE;
  if (loops_ceil > UINT32_MAX)
  {
    loops_ceil = UINT32_MAX;
  }
  half_period_loops_ = static_cast<uint32_t>(loops_ceil);
  return LibXR::ErrorCode::OK;
}

extern "C" void app_main()
{
  LibXR::CH32Timebase timebase;
  (void)timebase;
  LibXR::PlatformInit(3, 8192);

  // Original SWD path for WCH-LinkE style wiring: PB13(SWCLK) + PA9(SWDIO).
  LibXR::CH32GPIO swclk(GPIOB, GPIO_Pin_13);
  LibXR::CH32GPIO swdio(GPIOA, GPIO_Pin_9);
  LibXR::CH32GPIO nreset(GPIOC, GPIO_Pin_8);

  using SwdGpio = LibXR::Debug::SwdGeneralGPIO<LibXR::CH32GPIO, LibXR::CH32GPIO>;
  SwdGpio swd(swclk, swdio, 8);
  LibXR::USB::DapLinkV2Class dap(swd, &nreset);
  LibXR::USB::CDCUart cdc(1024, 1024, 8);

  static constexpr auto USB_LANG_PACK_EN_US = LibXR::USB::DescriptorStrings::MakeLanguagePack(
      LibXR::USB::DescriptorStrings::Language::EN_US, "XRobot", "CMSIS-DAP",
      "XRUSB-DEMO-XRDAP-");

  LibXR::CH32USBOtgHS usb_dev_hs(
      {
          {ep0_buffer_hs},
          {ep1_dap_in_hs, true},
          {ep2_dap_out_hs, false},
          {ep3_cdc_in_hs, true},
          {ep4_cdc_out_hs, false},
          {ep5_cdc_comm_in_hs, true},
      },
      0x0D28, 0x2040, 0x0201, {&USB_LANG_PACK_EN_US}, {{&dap, &cdc}},
      {reinterpret_cast<void*>(0x1FFFF7E8), 12});

  usb_dev_hs.Init(false);
  usb_dev_hs.Start(false);

  while (1)
  {
    LibXR::Thread::Sleep(1000);
  }
}
