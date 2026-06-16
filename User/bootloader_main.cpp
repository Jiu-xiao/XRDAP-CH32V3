#include <cstdint>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <new>

#include "ch32_flash.hpp"
#include "ch32_gpio.hpp"
#include "ch32_timebase.hpp"
#include "ch32_usb.hpp"
#include "ch32_usb_dev.hpp"
#include "ch32v30x.h"
#include "ch32v30x_flash.h"
#include "custom_bootloader_protocol.h"
#include "dfu.hpp"
#include "crc.hpp"
#include "libxr.hpp"
#include "thread.hpp"

namespace
{
static void* BootAlloc(std::size_t size, std::size_t align)
{
  if (size == 0u)
  {
    size = 1u;
  }
  if (align < alignof(void*))
  {
    align = alignof(void*);
  }

  const std::size_t total = size + align - 1u + sizeof(void*);
  void* raw = std::malloc(total);
  if (raw == nullptr)
  {
    std::abort();
  }

  const std::uintptr_t aligned =
      (reinterpret_cast<std::uintptr_t>(raw) + sizeof(void*) + align - 1u) &
      ~(static_cast<std::uintptr_t>(align) - 1u);
  reinterpret_cast<void**>(aligned)[-1] = raw;
  return reinterpret_cast<void*>(aligned);
}

static void BootFree(void* ptr)
{
  if (ptr == nullptr)
  {
    return;
  }
  std::free(reinterpret_cast<void**>(ptr)[-1]);
}
}  // namespace

void* operator new(std::size_t size) { return BootAlloc(size, alignof(std::max_align_t)); }

void* operator new[](std::size_t size) { return BootAlloc(size, alignof(std::max_align_t)); }

void* operator new(std::size_t size, std::align_val_t align)
{
  return BootAlloc(size, static_cast<std::size_t>(align));
}

void* operator new[](std::size_t size, std::align_val_t align)
{
  return BootAlloc(size, static_cast<std::size_t>(align));
}

void operator delete(void* ptr) noexcept { BootFree(ptr); }

void operator delete[](void* ptr) noexcept { BootFree(ptr); }

void operator delete(void* ptr, std::size_t) noexcept { BootFree(ptr); }

void operator delete[](void* ptr, std::size_t) noexcept { BootFree(ptr); }

void operator delete(void* ptr, std::align_val_t) noexcept { BootFree(ptr); }

void operator delete[](void* ptr, std::align_val_t) noexcept { BootFree(ptr); }

void operator delete(void* ptr, std::size_t, std::align_val_t) noexcept { BootFree(ptr); }

void operator delete[](void* ptr, std::size_t, std::align_val_t) noexcept { BootFree(ptr); }

namespace
{
#ifndef CH32_BOOT_APP_BASE
#define CH32_BOOT_APP_BASE 0x00002000u
#endif
#ifndef CH32_BOOT_APP_LIMIT
#define CH32_BOOT_APP_LIMIT 0x00020000u
#endif
#ifndef CH32_BOOT_ENABLE_FORCE_IAP_KEY
#define CH32_BOOT_ENABLE_FORCE_IAP_KEY 1
#endif
#ifndef CH32_BOOT_USB_VID
#define CH32_BOOT_USB_VID 0x1209u
#endif
#ifndef CH32_BOOT_USB_PID
#define CH32_BOOT_USB_PID 0x3072u
#endif
#ifndef CH32_BOOT_USB_BCD
#define CH32_BOOT_USB_BCD 0x0001u
#endif
#ifndef CH32_BOOT_RECLOCK_48M
#define CH32_BOOT_RECLOCK_48M 1
#endif
#ifndef CH32_BOOT_PAGE_READBACK_VERIFY
#define CH32_BOOT_PAGE_READBACK_VERIFY 0
#endif
#ifndef CH32_BOOT_REQUIRE_SEAL
#define CH32_BOOT_REQUIRE_SEAL 1
#endif
#ifndef CH32_BOOT_AUTORUN
#define CH32_BOOT_AUTORUN 0
#endif
#if (CH32_BOOT_AUTORUN != 0) && (CH32_BOOT_AUTORUN != 1)
#error "CH32_BOOT_AUTORUN must be 0 or 1."
#endif
#if (CH32_BOOT_APP_LIMIT <= CH32_BOOT_APP_BASE)
#error "CH32_BOOT_APP_LIMIT must be larger than CH32_BOOT_APP_BASE."
#endif

static constexpr uint32_t kFlashAbsoluteBase = 0x08000000u;
static constexpr uint32_t kAppAliasBase = static_cast<uint32_t>(CH32_BOOT_APP_BASE);
static constexpr uint32_t kAppAliasEnd = static_cast<uint32_t>(CH32_BOOT_APP_LIMIT);
static constexpr uint32_t kAppAbsoluteBase = kFlashAbsoluteBase + kAppAliasBase;
static constexpr uint32_t kAppAbsoluteEnd = kFlashAbsoluteBase + kAppAliasEnd;
static constexpr uint32_t kAppVectorEntryOffset = 0x4u;
static constexpr uint32_t kFastFlashPageSize = 256u;
static constexpr size_t kBootDfuTransferSize = 64u;
static constexpr uint32_t kIapLedBlinkPeriodMs = 250u;
static constexpr uint32_t kAppSealMagic = 0x4C414553u;  // "SEAL"
static constexpr uint32_t kHardFaultTraceBase = 0x20007FC0u;
static constexpr uint32_t kHardFaultTraceMagic = 0x48465231u;  // "HFR1"
static constexpr size_t kHardFaultTraceWords = 12u;
static constexpr uint32_t kBootRuntimeDiagBase = CH32_CUSTOM_BL_RT_DIAG_BASE;
static constexpr uint32_t kBootRuntimeDiagMagic = CH32_CUSTOM_BL_RT_DIAG_MAGIC;
static volatile uint32_t g_boot_reset_flags = 0u;
static constexpr uint32_t kAppSealAbsoluteBase = kAppAbsoluteEnd - kFastFlashPageSize;
static constexpr uint32_t kAppMaxPayloadBytes = kAppAbsoluteEnd - kAppAbsoluteBase - kFastFlashPageSize;
static constexpr size_t kAppAreaBytes = kAppAbsoluteEnd - kAppAbsoluteBase;
static constexpr size_t kAppAreaPageCount = kAppAreaBytes / kFastFlashPageSize;
static_assert((kAppAreaBytes % kFastFlashPageSize) == 0u, "App area must be page-aligned.");
static LibXR::FlashSector g_boot_flash_sectors[kAppAreaPageCount];

alignas(64) static uint8_t ep0_buffer_hs[kBootDfuTransferSize];

static uint32_t GetUptimeMs() { return static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds()); }

static uint32_t GetResetFlagsMask()
{
  return (RCC_PINRSTF | RCC_PORRSTF | RCC_SFTRSTF | RCC_IWDGRSTF | RCC_WWDGRSTF | RCC_LPWRRSTF);
}

struct BootRuntimeDiag
{
  uint32_t magic = 0u;
  uint32_t seq = 0u;
  uint32_t stage = 0u;
  uint32_t aux0 = 0u;
  uint32_t aux1 = 0u;
  uint32_t loop_count = 0u;
  uint32_t rx_bytes = 0u;
  uint32_t cmd_done = 0u;
  uint32_t read_err = 0u;
  uint32_t send_err = 0u;
  uint32_t line_drop = 0u;
  uint32_t write_ok = 0u;
  uint32_t write_err = 0u;
  uint32_t last_ec = 0u;
  uint32_t last_cmd_len = 0u;
  uint32_t last_addr = 0u;
};

static_assert((kBootRuntimeDiagBase + sizeof(BootRuntimeDiag)) <= kHardFaultTraceBase,
              "BL runtime diag overlaps hard-fault trace SRAM.");
static_assert((kHardFaultTraceBase + (kHardFaultTraceWords * sizeof(uint32_t))) <=
                  CH32_CUSTOM_BL_REQ_ADDR0,
              "Hard-fault trace overlaps boot-request mailbox SRAM.");

static volatile BootRuntimeDiag* const g_boot_runtime_diag =
    reinterpret_cast<volatile BootRuntimeDiag*>(kBootRuntimeDiagBase);

static void InitBootRuntimeDiag()
{
  volatile uint32_t* words = reinterpret_cast<volatile uint32_t*>(kBootRuntimeDiagBase);
  for (size_t i = 0; i < (sizeof(BootRuntimeDiag) / sizeof(uint32_t)); ++i)
  {
    words[i] = 0u;
  }
  g_boot_runtime_diag->magic = kBootRuntimeDiagMagic;
  g_boot_runtime_diag->aux0 = g_boot_reset_flags;
}

static inline void RecordBootDiagStage(uint32_t stage, uint32_t aux0 = 0u, uint32_t aux1 = 0u)
{
  g_boot_runtime_diag->seq++;
  g_boot_runtime_diag->stage = stage;
  g_boot_runtime_diag->aux0 = aux0;
  g_boot_runtime_diag->aux1 = aux1;
}

enum class BootDiagCode : uint32_t
{
  kBootEnter = 0xB1000001u,
  kBootSkip = 0xB1000002u,
  kUsbInit = 0xB1000003u,
  kUsbStart = 0xB1000004u,
  kRunAppJump = 0xB1000402u,
  kRunAppRejected = 0xB10004EEu,
};

static inline void RecordBootDiagStage(BootDiagCode stage, uint32_t aux0 = 0u, uint32_t aux1 = 0u)
{
  RecordBootDiagStage(static_cast<uint32_t>(stage), aux0, aux1);
}

static void ReclockBootloaderTo48MHz()
{
  __disable_irq();

  // Move SYSCLK to HSI first so PLL can be safely reconfigured.
  RCC->CTLR |= RCC_HSION;
  while ((RCC->CTLR & RCC_HSIRDY) == 0u)
  {
  }

  RCC->CFGR0 &= static_cast<uint32_t>(~RCC_SW);
  RCC->CFGR0 |= RCC_SW_HSI;
  while ((RCC->CFGR0 & RCC_SWS) != 0x00u)
  {
  }

  // Stop PLL before reprogramming multiplier/source.
  RCC->CTLR &= static_cast<uint32_t>(~RCC_PLLON);
  while ((RCC->CTLR & RCC_PLLRDY) != 0u)
  {
  }

  // Target clock tree for 48MHz:
  // SYSCLK=48MHz, HCLK=48MHz, PCLK2=48MHz, PCLK1=24MHz.
  RCC->CFGR0 &= static_cast<uint32_t>(~(RCC_HPRE | RCC_PPRE2 | RCC_PPRE1));
  RCC->CFGR0 |= (RCC_HPRE_DIV1 | RCC_PPRE2_DIV1 | RCC_PPRE1_DIV2);

  EXTEN->EXTEN_CTR |= EXTEN_PLL_HSI_PRE;
  RCC->CFGR0 &= static_cast<uint32_t>(~(RCC_PLLSRC | RCC_PLLXTPRE | RCC_PLLMULL));
#ifdef CH32V30x_D8
  RCC->CFGR0 |= (RCC_PLLSRC_HSI_Div2 | RCC_PLLMULL6);
#else
  RCC->CFGR0 |= (RCC_PLLSRC_HSI_Div2 | RCC_PLLMULL6_EXTEN);
#endif

  RCC->CTLR |= RCC_PLLON;
  while ((RCC->CTLR & RCC_PLLRDY) == 0u)
  {
  }

  RCC->CFGR0 &= static_cast<uint32_t>(~RCC_SW);
  RCC->CFGR0 |= RCC_SW_PLL;
  while ((RCC->CFGR0 & RCC_SWS) != 0x08u)
  {
  }

  SystemCoreClockUpdate();
  SysTick->CTLR = 0;
  SysTick->SR = 0;
  SysTick->CNT = 0;
  SysTick->CMP = SystemCoreClock / 1000u;
  SysTick->CTLR = 0x0Fu;

  __enable_irq();
}

struct AppSealInfo
{
  uint32_t magic = 0u;
  uint32_t image_size = 0u;
  uint32_t crc32 = 0u;
  uint32_t crc32_inv = 0u;
};

enum class AppSealState : uint8_t
{
  NONE = 0,
  OK = 1,
  BAD = 2,
};

static uint32_t Crc32FlashRegion(uint32_t abs_addr, size_t len)
{
  return LibXR::CRC32::Calculate(reinterpret_cast<const void*>(abs_addr), len);
}

static void ReadAppSeal(AppSealInfo& seal)
{
  auto words = reinterpret_cast<volatile const uint32_t*>(kAppSealAbsoluteBase);
  seal.magic = words[0];
  seal.image_size = words[1];
  seal.crc32 = words[2];
  seal.crc32_inv = words[3];
}

static AppSealState CheckAppSeal(uint32_t* out_size = nullptr, uint32_t* out_expect_crc = nullptr,
                                 uint32_t* out_actual_crc = nullptr)
{
  AppSealInfo seal = {};
  ReadAppSeal(seal);
  if (seal.magic != kAppSealMagic)
  {
    return AppSealState::NONE;
  }

  if (out_size != nullptr)
  {
    *out_size = seal.image_size;
  }
  if (out_expect_crc != nullptr)
  {
    *out_expect_crc = seal.crc32;
  }

  if (seal.image_size == 0u || seal.image_size > kAppMaxPayloadBytes)
  {
    return AppSealState::BAD;
  }
  if ((seal.crc32 ^ seal.crc32_inv) != 0xFFFFFFFFu)
  {
    return AppSealState::BAD;
  }

  uint32_t actual = Crc32FlashRegion(kAppAbsoluteBase, seal.image_size);
  if (out_actual_crc != nullptr)
  {
    *out_actual_crc = actual;
  }
  if (actual != seal.crc32)
  {
    return AppSealState::BAD;
  }
  return AppSealState::OK;
}

static bool IsAppEntryValid()
{
  uint32_t vec0 = *reinterpret_cast<volatile uint32_t*>(kAppAliasBase + 0x0u);
  uint32_t vec1 = *reinterpret_cast<volatile uint32_t*>(kAppAliasBase + kAppVectorEntryOffset);
  uint32_t nmi = *reinterpret_cast<volatile uint32_t*>(kAppAliasBase + 0x0Cu);
  uint32_t hf = *reinterpret_cast<volatile uint32_t*>(kAppAliasBase + 0x10u);

  auto is_erased_or_magic = [](uint32_t v) {
    return (v == 0x00000000u) || (v == 0xFFFFFFFFu) || (v == 0xE339E339u);
  };

  if (is_erased_or_magic(vec0) || is_erased_or_magic(vec1) || is_erased_or_magic(nmi) ||
      is_erased_or_magic(hf))
  {
    return false;
  }

  // CH32 startup layout expects vector[1] to equal image base address.
  if (vec1 != kAppAliasBase)
  {
    return false;
  }

  if ((nmi < kAppAliasBase) || (nmi >= kAppAliasEnd) || (hf < kAppAliasBase) ||
      (hf >= kAppAliasEnd))
  {
    return false;
  }
  return true;
}

static bool IsForceIapKeyPressed()
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitTypeDef gpio = {};
  gpio.GPIO_Pin = GPIO_Pin_7;
  gpio.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &gpio);
  LibXR::Thread::Sleep(2);
  return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7) == Bit_RESET;
}

static void InitIapLed()
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitTypeDef gpio = {};
  gpio.GPIO_Pin = GPIO_Pin_9;
  gpio.GPIO_Speed = GPIO_Speed_10MHz;
  gpio.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &gpio);
  GPIO_ResetBits(GPIOC, GPIO_Pin_9);
}

static void ServiceIapLedBlink(uint32_t now_ms)
{
  static uint32_t s_last_toggle_ms = 0u;
  static bool s_led_on = false;
  if ((now_ms - s_last_toggle_ms) < kIapLedBlinkPeriodMs)
  {
    return;
  }
  s_last_toggle_ms = now_ms;
  s_led_on = !s_led_on;
  if (s_led_on)
  {
    GPIO_SetBits(GPIOC, GPIO_Pin_9);
  }
  else
  {
    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
  }
}

static bool ConsumeBootRequest()
{
  volatile uint32_t* req0 = reinterpret_cast<volatile uint32_t*>(CH32_CUSTOM_BL_REQ_ADDR0);
  volatile uint32_t* req1 = reinterpret_cast<volatile uint32_t*>(CH32_CUSTOM_BL_REQ_ADDR1);

  uint32_t v0 = *req0;
  uint32_t v1 = *req1;
  bool match = (v0 == CH32_CUSTOM_BL_REQ_MAGIC0) && (v1 == CH32_CUSTOM_BL_REQ_MAGIC1);
  if (match)
  {
    *req0 = 0u;
    *req1 = 0u;
    __asm volatile("fence iorw, iorw" ::: "memory");
  }
  return match;
}

static bool ShouldStayInBootloader()
{
  if (ConsumeBootRequest())
  {
    return true;
  }

  if (CH32_BOOT_ENABLE_FORCE_IAP_KEY && IsForceIapKeyPressed())
  {
    return true;
  }

  if (!IsAppEntryValid())
  {
    return true;
  }

  AppSealState seal_state = CheckAppSeal();
  if (seal_state == AppSealState::BAD)
  {
    return true;
  }

  if (CH32_BOOT_REQUIRE_SEAL && (seal_state != AppSealState::OK))
  {
    return true;
  }

  if (!CH32_BOOT_AUTORUN)
  {
    return true;
  }

  return false;
}

static void ShutdownUsbHs()
{
  NVIC_DisableIRQ(USBHS_IRQn);
  NVIC_DisableIRQ(USBHSWakeup_IRQn);
  USBHSD->HOST_CTRL = 0x00;
  USBHSD->CONTROL = 0x00;
  USBHSD->INT_EN = 0x00;
  USBHSD->ENDP_CONFIG = 0xFFFFFFFFu;
  USBHSD->CONTROL &= ~USBHS_UC_DEV_PU_EN;
  USBHSD->CONTROL |= USBHS_UC_CLR_ALL | USBHS_UC_RESET_SIE;
  USBHSD->CONTROL = 0x00;
}

static void ShutdownInterruptState()
{
  SysTick->CTLR = 0u;
  SysTick->SR = 0u;
  SysTick->CNT = 0u;
  SysTick->CMP = 0u;

  for (size_t i = 0u; i < (sizeof(PFIC->IRER) / sizeof(PFIC->IRER[0])); ++i)
  {
    PFIC->IRER[i] = 0xFFFFFFFFu;
    PFIC->IPRR[i] = 0xFFFFFFFFu;
  }
}

[[noreturn]] static void JumpToApp()
{
  __disable_irq();
  ShutdownUsbHs();
  ShutdownInterruptState();
  __set_MTVEC((kAppAliasBase + kAppVectorEntryOffset) | 0x3u);
  auto app_entry = reinterpret_cast<void (*)()>(kAppAliasBase);
  app_entry();
  while (1)
  {
  }
}

void JumpToAppThunk(void*)
{
  JumpToApp();
}

static void InitBootFlashGeometry()
{
  static bool inited = false;
  if (inited)
  {
    return;
  }

  for (size_t i = 0u; i < kAppAreaPageCount; ++i)
  {
    g_boot_flash_sectors[i].address =
        kAppAbsoluteBase + static_cast<uint32_t>(i * kFastFlashPageSize);
    g_boot_flash_sectors[i].size = kFastFlashPageSize;
  }
  inited = true;
}

static LibXR::CH32Flash& BootFlash()
{
  InitBootFlashGeometry();
  static LibXR::CH32Flash flash(g_boot_flash_sectors, kAppAreaPageCount, 1u);
  return flash;
}
}  // namespace

extern "C" void bootloader_main(void)
{
#if CH32_BOOT_RECLOCK_48M
  ReclockBootloaderTo48MHz();
#endif
  g_boot_reset_flags = (RCC->RSTSCKR & GetResetFlagsMask());
  RCC_ClearFlag();
  LibXR::CH32Timebase timebase;
  (void)timebase;
  LibXR::PlatformInit();
  InitBootRuntimeDiag();
  RecordBootDiagStage(BootDiagCode::kBootEnter, g_boot_reset_flags, 0u);

  if (!ShouldStayInBootloader())
  {
    RecordBootDiagStage(BootDiagCode::kBootSkip, 0u, 0u);
    JumpToApp();
  }

  InitIapLed();

  LibXR::USB::DfuBootloaderClass dfu(BootFlash(), 0u, kAppAreaBytes, kAppMaxPayloadBytes,
                                     JumpToAppThunk, nullptr, CH32_BOOT_AUTORUN != 0);
  static constexpr auto kUsbLangPackEnUs = LibXR::USB::DescriptorStrings::MakeLanguagePack(
      LibXR::USB::DescriptorStrings::Language::EN_US, "XRobot", "CH32V3-DFU-BL",
      "XRUSB-BL-");

  LibXR::CH32USBOtgHS usb_dev_hs(
      {
          {ep0_buffer_hs},
      },
      static_cast<uint16_t>(CH32_BOOT_USB_VID), static_cast<uint16_t>(CH32_BOOT_USB_PID),
      static_cast<uint16_t>(CH32_BOOT_USB_BCD), {&kUsbLangPackEnUs}, {{&dfu}},
      {reinterpret_cast<void*>(0x1FFFF7E8), 12});

  usb_dev_hs.Init(false);
  RecordBootDiagStage(BootDiagCode::kUsbInit, 0u, 0u);
  usb_dev_hs.Start(false);
  RecordBootDiagStage(BootDiagCode::kUsbStart, 0u, 0u);

  while (1)
  {
    g_boot_runtime_diag->loop_count++;
    uint32_t now_ms = GetUptimeMs();
    ServiceIapLedBlink(now_ms);
    dfu.Process();
    if (dfu.TryConsumeAppLaunch(now_ms))
    {
      if (IsAppEntryValid() && (CheckAppSeal() == AppSealState::OK))
      {
        RecordBootDiagStage(BootDiagCode::kRunAppJump, now_ms, 0u);
        JumpToApp();
      }
      RecordBootDiagStage(BootDiagCode::kRunAppRejected, now_ms, 0u);
    }

    if (!dfu.HasPendingWork())
    {
      LibXR::Thread::Sleep(1);
    }
  }
}
