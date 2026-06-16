#include <cstdint>

#include "ch32_gpio.hpp"
#include "ch32_fast_gpio.hpp"
#include "ch32_power.hpp"
#include "ch32_timebase.hpp"
#include "ch32_usb.hpp"
#include "ch32_usb_dev.hpp"
#include "ch32v30x_iwdg.h"
#include "custom_bootloader_protocol.h"
#include "daplink_v2.hpp"
#include "debug/swd_general_gpio.hpp"
#include "dfu.hpp"
#include "libxr.hpp"
#include "thread.hpp"

namespace {
static uint8_t ep0_buffer_hs[64];
static uint8_t ep1_dap_in_hs[1024];
static uint8_t ep2_dap_out_hs[1024];
static constexpr uint32_t kAppWatchdogTimeoutMs = 2000u;

class AppBootloaderPowerManager : public LibXR::CH32PowerManager {
 public:
  void JumpToBootloader() override {
    auto* req0 = reinterpret_cast<volatile uint32_t*>(CH32_CUSTOM_BL_REQ_ADDR0);
    auto* req1 = reinterpret_cast<volatile uint32_t*>(CH32_CUSTOM_BL_REQ_ADDR1);
    *req0 = CH32_CUSTOM_BL_REQ_MAGIC0;
    *req1 = CH32_CUSTOM_BL_REQ_MAGIC1;
    __asm volatile("fence iorw, iorw" ::: "memory");
    Reset();
  }
};

void JumpToBootloaderThunk(void* ctx) {
  if (ctx == nullptr) {
    return;
  }
  static_cast<LibXR::PowerManager*>(ctx)->JumpToBootloader();
}

void InitAppWatchdog() {
  RCC_LSICmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET) {
  }

  constexpr uint32_t kIwdgTicksPerSecond = 40000u / 64u;
  uint32_t reload = ((kAppWatchdogTimeoutMs * kIwdgTicksPerSecond) + 999u) / 1000u;
  if (reload == 0u) {
    reload = 1u;
  }
  if (reload > 0x0FFFu) {
    reload = 0x0FFFu;
  }

  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(IWDG_Prescaler_64);
  IWDG_SetReload(static_cast<uint16_t>(reload));
  while (IWDG_GetFlagStatus(IWDG_FLAG_PVU) != RESET ||
         IWDG_GetFlagStatus(IWDG_FLAG_RVU) != RESET) {
  }
  IWDG_ReloadCounter();
  IWDG_Enable();
}

inline void FeedAppWatchdog() { IWDG_ReloadCounter(); }
}  // namespace

template <>
LibXR::ErrorCode
LibXR::Debug::SwdGeneralGPIO<LibXR::CH32GPIOSWC, LibXR::CH32GPIOSWD,
                             LibXR::Debug::SwdIoDriveMode::PUSH_PULL>::SetClockHz(
    uint32_t hz)
{
  static constexpr uint32_t kBoardSwdMaxHz = MAX_HZ;

  if (hz == 0u) {
    clock_hz_ = 0u;
    half_period_ns_ = 0u;
    half_period_loops_ = 0u;
    return LibXR::ErrorCode::OK;
  }

  if (hz < MIN_HZ) {
    hz = MIN_HZ;
  }
  if (hz > MAX_HZ) {
    hz = MAX_HZ;
  }
  if (hz > kBoardSwdMaxHz) {
    hz = kBoardSwdMaxHz;
  }

  clock_hz_ = hz;

  const uint32_t den = 2u * hz;
  half_period_ns_ = (NS_PER_SEC + den - 1u) / den;

  if (loops_per_us_ == 0u) {
    half_period_loops_ = 0u;
    return LibXR::ErrorCode::OK;
  }

  const uint64_t loops_num =
      static_cast<uint64_t>(loops_per_us_) * static_cast<uint64_t>(half_period_ns_);
  if (loops_num < LOOPS_SCALE) {
    half_period_loops_ = 0u;
    return LibXR::ErrorCode::OK;
  }

  uint64_t loops_ceil = (loops_num + CEIL_BIAS) / LOOPS_SCALE;
  if (loops_ceil > UINT32_MAX) {
    loops_ceil = UINT32_MAX;
  }
  half_period_loops_ = static_cast<uint32_t>(loops_ceil);
  return LibXR::ErrorCode::OK;
}

extern "C" void app_main() {
  LibXR::CH32Timebase timebase;
  (void)timebase;
  LibXR::PlatformInit();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

  // Keep board control pins low before the CMSIS-DAP SWD bridge starts.
  static LibXR::CH32GPIO force_pb12_low(GPIOB, GPIO_Pin_12);
  static LibXR::CH32GPIO force_pa5_low(GPIOA, GPIO_Pin_5);
  force_pb12_low.Write(false);
  force_pa5_low.Write(false);

  static LibXR::CH32GPIOSWC swclk;
  static LibXR::CH32GPIOSWD swdio;
  static LibXR::CH32GPIO nreset(GPIOC, GPIO_Pin_8);
  nreset.Write(true);

  using SwdGpio = LibXR::Debug::SwdGeneralGPIO<LibXR::CH32GPIOSWC, LibXR::CH32GPIOSWD>;
  using DapLink = LibXR::USB::DapLinkV2Class<SwdGpio, 512u, 4u, 1024u, 2048u, 255u>;

  static SwdGpio swd(swclk, swdio, 0, 500'000u);
  static DapLink dap(swd, &nreset);

  AppBootloaderPowerManager power;
  LibXR::USB::DfuRuntimeClass dfu_runtime(JumpToBootloaderThunk, &power, 50u);

  static constexpr auto kUsbLangPackEnUs = LibXR::USB::DescriptorStrings::MakeLanguagePack(
      LibXR::USB::DescriptorStrings::Language::EN_US,
      "XRobot", "CMSIS-DAP", "XRUSB-DEMO-XRDAP-");

  LibXR::CH32USBOtgHS usb_dev_hs(
      {
          {ep0_buffer_hs},
          {ep1_dap_in_hs, true},
          {ep2_dap_out_hs, false},
      },
      0x0D28, 0x2041, 0x0202, {&kUsbLangPackEnUs}, {{&dap, &dfu_runtime}},
      {reinterpret_cast<void*>(0x1FFFF7E8), 12});

  usb_dev_hs.Init(false);
  usb_dev_hs.Start(false);
  InitAppWatchdog();

  while (1) {
    FeedAppWatchdog();
    dfu_runtime.Process();
    LibXR::Thread::Sleep(1);
  }
}
