// SPDX-License-Identifier: Apache-2.0
// evt (WCH evaluation board): the common probe only.
//
// evt（WCH 评估板）：只有公共探针部分。

#include "boot_slot_info.hpp"
#include "probe.hpp"

extern "C" void app_main()
{
  using namespace XrDap;
  static Probe probe;
  static BootSlotInfo boot_slots;
  probe.Run({&boot_slots});
}
