// SPDX-License-Identifier: Apache-2.0
// v305 and v305-144: the common probe and the status LED.
//
// v305 与 v305-144：公共探针部分和状态灯。

#include "boot_slot_info.hpp"
#include "probe.hpp"
#include "status_led.hpp"

extern "C" void app_main()
{
  using namespace XrDap;
  static Probe probe;
  static StatusLed led(probe);
  static BootSlotInfo boot_slots;
  probe.Run({&led, &boot_slots});
}
