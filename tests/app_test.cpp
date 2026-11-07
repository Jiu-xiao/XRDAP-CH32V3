// SPDX-License-Identifier: Apache-2.0
// Tests of the parts of app/ that write registers, on the build machine: tests/fake
// stands in for the WCH headers, LibXR's GPIO driver and XRDAP-Core's vendor extension.
//
// 在编译机上测试 app/ 里写寄存器的部分：tests/fake 代替 WCH 头文件、LibXR 的 GPIO 驱动
// 和 XRDAP-Core 的厂商扩展。

#include <cstdio>

#include "board_pin.hpp"
#include "ch32_gpio.hpp"
#include "ch32v30x.h"
#include "sampler_tick.hpp"
#include "target_reset.hpp"

extern "C" void TIM7_IRQHandler(void);

namespace
{
int failures = 0;
int checks = 0;

#define CHECK(cond)                                               \
  do                                                              \
  {                                                               \
    ++checks;                                                     \
    if (!(cond))                                                  \
    {                                                             \
      ++failures;                                                 \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                             \
  } while (0)

using namespace XrDap;

// The pin is high the moment it becomes an output, on the reset pin of every board (wdap;
// v305 and linkw), on a pin of another port, and on a port whose clock is still off.
void TestTargetResetStaysHigh()
{
  for (const BoardPin pin :
       {BoardPin{kPortB, 12}, BoardPin{kPortC, 1}, BoardPin{kPortC, 8}})
  {
    Fake::Reset();
    const LibXR::CH32GPIO gpio = TargetResetOutput(pin);
    CHECK(gpio.HighAsOutput());
  }
}

// The timer counts at 1 MHz from the core clock: the board's clocks and, with the crystal
// not started, the internal oscillator.
void TestSamplerTickCountsMicroseconds()
{
  DapVendor vendor;
  SamplerTick tick(vendor);
  for (const uint32_t clock_hz : {216000000U, 144000000U, 8000000U})
  {
    SystemCoreClock = clock_hz;
    tick.Start(1000U);
    CHECK(Fake::tim7.PSC == clock_hz / 1000000U - 1U);
    CHECK(Fake::tim7.ATRLR == 999U);
  }
}

// A period longer than the 16-bit counter is split into equal timer periods, and every
// repeat-th interrupt is a tick.
void TestSamplerTickLongPeriod()
{
  DapVendor vendor;
  SamplerTick tick(vendor);
  SystemCoreClock = 216000000U;
  tick.Start(200000U);  // 4 periods of 50 ms
  CHECK(Fake::tim7.ATRLR == 49999U);
  for (int i = 0; i < 8; ++i)
  {
    Fake::tim7_update_pending = true;
    TIM7_IRQHandler();
    CHECK(!Fake::tim7_update_pending);
    CHECK(vendor.ticks == static_cast<uint32_t>((i + 1) / 4));
  }
  TIM7_IRQHandler();  // nothing pending
  CHECK(vendor.ticks == 2U);
}
}  // namespace

int main()
{
  TestTargetResetStaysHigh();
  TestSamplerTickCountsMicroseconds();
  TestSamplerTickLongPeriod();
  std::printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
