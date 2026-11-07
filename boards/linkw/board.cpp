// SPDX-License-Identifier: Apache-2.0
// linkw: the common probe, the chip's radio and the two LEDs: the blue status LED and the
// green radio LED.
//
// linkw：公共探针部分、芯片的射频和两颗灯：蓝色状态灯和绿色无线灯。

#include "boot_slot_info.hpp"
#include "ch32_gpio.hpp"
#include "probe.hpp"
#include "radio_part.hpp"
#include "status_led.hpp"
#include "timebase.hpp"
#include "v208_radio.hpp"

namespace XrDap
{

namespace
{

class RadioLed final : public Part
{
 public:
  explicit RadioLed(const RadioPart<V208Radio>& radio)
      : radio_(radio), led_(Board::kRadioLed.Port(), Board::kRadioLed.Mask())
  {
    led_.Write(true);
  }

  bool Poll() override
  {
    const uint32_t now = static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    const bool lit = radio_.Pairing() ? (now / kBlinkMs) % 2U != 0U : radio_.Wireless();
    led_.Write(!lit);
    return false;
  }

 private:
  static constexpr uint32_t kBlinkMs = 125;

  const RadioPart<V208Radio>& radio_;
  LibXR::CH32GPIO led_;
};

}  // namespace

}  // namespace XrDap

extern "C" void app_main()
{
  using namespace XrDap;
  static Probe probe;
  static V208Radio transceiver;
  static RadioPart<V208Radio> radio(transceiver, probe);
  static StatusLed led(probe);
  static RadioLed radio_led(radio);
  static BootSlotInfo boot_slots;
  probe.Run({&radio, &led, &radio_led, &boot_slots});
}
