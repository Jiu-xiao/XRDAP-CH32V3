// SPDX-License-Identifier: Apache-2.0
// Tests of the parts of app/ that write registers, on the build machine: tests/fake
// stands in for the WCH headers and LibXR's GPIO driver.
//
// 在编译机上测试 app/ 里写寄存器的部分：tests/fake 代替 WCH 头文件和 LibXR 的 GPIO 驱动。

#include <cstdio>

#include "board_pin.hpp"
#include "ch32_gpio.hpp"
#include "ch32v30x.h"
#include "target_reset.hpp"

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

// The pin is high the moment it becomes an output, on the reset pin of every board
// (evt, wdap, v305), and on a port whose clock is still off.
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
}  // namespace

int main()
{
  TestTargetResetStaysHigh();
  std::printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
