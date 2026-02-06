#include <cmath>

#include "cdc_uart.hpp"
#include "ch32_gpio.hpp"
#include "ch32_i2c.hpp"
#include "ch32_timebase.hpp"
#include "ch32_uart.hpp"
#include "ch32_usb.hpp"
#include "ch32_usb_dev.hpp"
#include "ch32v30x_gpio.h"
#include "hid_keyboard.hpp"
#include "libxr.hpp"
#include "semaphore.hpp"
#include "thread.hpp"

static uint8_t ep0_buffer[64];
static uint8_t ep1_buffer[256], ep2_buffer[256];
static uint8_t ep0_buffer_hs[64];
static uint8_t ep1_buffer_tx_hs[1024], ep2_buffer_rx_hs[1024], ep3_buffer_tx_hs[1024];

extern "C" void app_main()
{
  // Initialize timebase
  LibXR::CH32Timebase timebase;

  LibXR::PlatformInit(3, 8192);

  // Initialize GPIO
  LibXR::CH32GPIO led_b(GPIOB, GPIO_Pin_4);
  LibXR::CH32GPIO led_r(GPIOA, GPIO_Pin_15);
  LibXR::CH32GPIO key(GPIOB, GPIO_Pin_3, LibXR::CH32GPIO::Direction::FALL_INTERRUPT,
                      LibXR::CH32GPIO::Pull::UP, EXTI3_IRQn);

  void (*key_cb_fun)(bool, LibXR::GPIO*) = [](bool, LibXR::GPIO* led)
  {
    static bool flag = false;
    flag = !flag;
    led->Write(flag);
  };

  auto key_cb =
      LibXR::GPIO::Callback::Create(key_cb_fun, reinterpret_cast<LibXR::GPIO*>(&led_r));

  key.RegisterCallback(key_cb);

  // Initialize Blink Task
  void (*blink_task)(LibXR::GPIO*) = [](LibXR::GPIO* led)
  {
    static bool flag = false;
    if (flag)
    {
      flag = false;
    }
    else
    {
      flag = true;
    }

    led->Write(flag);
  };

  auto blink_task_handle =
      LibXR::Timer::CreateTask(blink_task, reinterpret_cast<LibXR::GPIO*>(&led_b), 1000);
  LibXR::Timer::Add(blink_task_handle);
  LibXR::Timer::Start(blink_task_handle);

  // Initialize I2C
  static uint8_t i2c_dma_buf[64];

  LibXR::CH32I2C i2c(CH32_I2C1, LibXR::RawData{i2c_dma_buf, sizeof(i2c_dma_buf)}, GPIOB,
                     GPIO_Pin_6, GPIOB, GPIO_Pin_7, 0, 0, 400000);

  // Initialize UART
  uint8_t uart1_tx_buffer[64], uart1_rx_buffer[64];

  static LibXR::CH32UART uart1(CH32_USART2, uart1_rx_buffer, uart1_tx_buffer, GPIOA,
                               GPIO_Pin_2, GPIOA, GPIO_Pin_3, 0, 25);

  uart1.SetConfig({
      .baudrate = 115200,
      .parity = LibXR::UART::Parity::NO_PARITY,
      .data_bits = 8,
      .stop_bits = 1,
  });

  // Initialize USB device
  static constexpr auto LANG_PACK_EN_US = LibXR::USB::DescriptorStrings::MakeLanguagePack(
      LibXR::USB::DescriptorStrings::Language::EN_US, "XRobot", "CDC Demo",
      "XRUSB-DEMO-");

  LibXR::USB::CDCUart cdc(4096, 4096, 8), cdc1;

  LibXR::CH32USBDeviceFS usb_dev(
      /* EP */
      {
          {ep0_buffer},
          {ep1_buffer},
          {ep2_buffer},
      },
      /* packet size */
      LibXR::USB::DeviceDescriptor::PacketSize0::SIZE_64,
      /* vid pid bcd */
      0x1209, 0x0001, 0x0100,
      /* language */
      {&LANG_PACK_EN_US},
      /* config */
      {{&cdc1}}, {reinterpret_cast<void*>(0x1FFFF7E8), 12});

  usb_dev.Init(false);

  usb_dev.Start(false);

  LibXR::CH32USBDeviceHS usb_dev_hs(
      /* EP */
      {
          {ep0_buffer_hs},
          {ep1_buffer_tx_hs, true},
          {ep2_buffer_rx_hs, true},
          {ep3_buffer_tx_hs, false},
      },
      /* vid pid bcd */
      0x1209, 0x0001, 0x0100,
      /* language */
      {&LANG_PACK_EN_US},
      /* config */
      {{&cdc}}, {reinterpret_cast<void*>(0x1FFFF7E8), 12});

  usb_dev_hs.Init(false);

  usb_dev_hs.Start(false);

  // Initialize RamFS and Terminal
  LibXR::RamFS ramfs;

  LibXR::STDIO::read_ = cdc.read_port_;
  LibXR::STDIO::write_ = cdc.write_port_;

  LibXR::Terminal<> terminal(ramfs);

  auto terminal_task_handle1 = LibXR::Timer::CreateTask(terminal.TaskFun, &terminal, 1);
  LibXR::Timer::Add(terminal_task_handle1);
  LibXR::Timer::Start(terminal_task_handle1);

  while (1)
  {
    LibXR::Thread::Sleep(1000);
  }
}
