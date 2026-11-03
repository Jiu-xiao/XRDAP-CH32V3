// SPDX-License-Identifier: Apache-2.0
// wdap and wdap-144: the common probe, the radio and the WS2812 status chain.
//
// wdap 与 wdap-144：公共探针部分、无线和 WS2812 状态灯串。

#include "boot_slot_info.hpp"
#include "probe.hpp"
#include "radio.hpp"
#include "status_lights.hpp"

extern "C" void app_main()
{
  using namespace XrDap;
  static Probe probe;
  static RadioPart radio(probe);
  static StatusLights lights(probe, radio);
  static BootSlotInfo boot_slots;
  probe.Run({&radio, &lights, &boot_slots});
}
