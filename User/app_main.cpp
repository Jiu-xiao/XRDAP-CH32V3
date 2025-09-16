#include <cmath>

#include "cdc.hpp"
#include "ch32_gpio.hpp"
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
static uint8_t ep1_buffer[256], ep2_buffer[256], ep3_buffer[256], ep4_buffer[256],
    ep5_buffer[256], ep6_buffer[256], ep7_buffer[256];
static uint8_t ep0_buffer_hs[64];
static uint8_t ep1_buffer_tx_hs[1024], ep2_buffer_rx_hs[1024], ep3_buffer_tx_hs[1024],
    ep4_buffer_rx_hs[1024];

extern "C" void app_main()
{
  static constexpr auto LANG_PACK_EN_US = LibXR::USB::DescriptorStrings::MakeLanguagePack(
      LibXR::USB::DescriptorStrings::Language::EN_US, "XRobot", "CDC Demo", "123456789");

  LibXR::USB::CDC cdc(2048, 2048), cdc1, cdc2;
  LibXR::USB::HIDKeyboard hid_keyboard;

  for (int i = 0; i < 256; i++)
  {
    ep0_buffer[i] = 1;
    ep1_buffer[i] = 2;
    ep2_buffer[i] = 3;
    ep3_buffer[i] = 4;
    ep4_buffer[i] = 5;
    ep5_buffer[i] = 6;
    ep6_buffer[i] = 7;
    ep7_buffer[i] = 8;
  }

  for (int i = 256; i < 1024 + 64; i++)
  {
    ep3_buffer[i] = 9;
  }

  LibXR::CH32USBDeviceFS usb_dev(
      /* EP */
      {
          {ep0_buffer},
          {ep1_buffer},
          {ep2_buffer},
          {ep3_buffer},
          {ep4_buffer},
          {ep5_buffer},
          {ep6_buffer},
          {ep7_buffer},
      },
      /* packet size */
      LibXR::USB::DeviceDescriptor::PacketSize0::SIZE_64,
      /* vid pid bcd */
      0x1209, 0x0001, 0x0100,
      /* language */
      {&LANG_PACK_EN_US},
      /* config */
      {{&cdc}});

  LibXR::CH32USBDeviceHS usb_dev_hs(
      /* EP */
      {
          {ep0_buffer_hs},
          {ep1_buffer_tx_hs, true},
          {ep2_buffer_rx_hs, false},
          {ep3_buffer_tx_hs, true},
          {ep4_buffer_rx_hs, false},
      },
      /* vid pid bcd */
      0x1209, 0x0001, 0x0100,
      /* language */
      {&LANG_PACK_EN_US},
      /* config */
      {{&cdc1}});

  usb_dev.Init();

  usb_dev.Start();

  LibXR::CH32Timebase timebase;

  LibXR::PlatformInit(3, 8192);

  LibXR::CH32GPIO led_b(GPIOB, GPIO_Pin_4);
  LibXR::CH32GPIO led_r(GPIOA, GPIO_Pin_15);
  LibXR::CH32GPIO key(GPIOB, GPIO_Pin_3, LibXR::CH32GPIO::Direction::FALL_INTERRUPT,
                      LibXR::CH32GPIO::Pull::UP, EXTI3_IRQn);

  void (*key_cb_fun)(bool, LibXR::GPIO *) = [](bool, LibXR::GPIO *led)
  {
    static bool flag = false;
    flag = !flag;
    led->Write(flag);
  };

  auto key_cb =
      LibXR::GPIO::Callback::Create(key_cb_fun, reinterpret_cast<LibXR::GPIO *>(&led_r));

  key.RegisterCallback(key_cb);

  key.DisableInterrupt();

  uint8_t uart1_tx_buffer[64], uart1_rx_buffer[64];

  static LibXR::CH32UART uart1(CH32_USART2, uart1_rx_buffer, uart1_tx_buffer, GPIOA,
                               GPIO_Pin_2, GPIOA, GPIO_Pin_3, 0, 25);

  uart1.SetConfig({
      .baudrate = 115200,
      .parity = LibXR::UART::Parity::NO_PARITY,
      .data_bits = 8,
      .stop_bits = 1,
  });

  volatile static uint32_t counter = 0, speed = 0;

  void (*blink_task)(LibXR::GPIO *) = [](LibXR::GPIO *led)
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

    speed = counter;
    counter = 0;

    // static char buf[32];
    // LibXR::WriteOperation op;

    // uart1.Write({buf, strlen(buf)}, op);

    led->Write(flag);
  };

  auto blink_task_handle =
      LibXR::Timer::CreateTask(blink_task, reinterpret_cast<LibXR::GPIO *>(&led_b), 1000);
  LibXR::Timer::Add(blink_task_handle);
  LibXR::Timer::Start(blink_task_handle);

#if 0
  LibXR::STDIO::read_ = cdc.read_port_;
  LibXR::STDIO::write_ = cdc.write_port_;

  LibXR::RamFS ramfs;

  LibXR::Terminal<> terminal(ramfs);

  auto terminal_task_handle = LibXR::Timer::CreateTask(terminal.TaskFun, &terminal, 1);
  LibXR::Timer::Add(terminal_task_handle);
  LibXR::Timer::Start(terminal_task_handle);

  LibXR::STDIO::read_ = cdc1.read_port_;
  LibXR::STDIO::write_ = cdc1.write_port_;

  LibXR::Terminal<> terminal1(ramfs);

  auto terminal_task_handle1 = LibXR::Timer::CreateTask(terminal.TaskFun, &terminal1, 1);
  LibXR::Timer::Add(terminal_task_handle1);
  LibXR::Timer::Start(terminal_task_handle1);
#endif
#if 1
  LibXR::WriteOperation op;
  uart1.Write(LibXR::ConstRawData("Hello World!\r\n"), op);
  static uint8_t buffer[512];
  LibXR::Semaphore sem(1000);
  LibXR::ReadOperation read_op(sem), read_op_no_block;
  LibXR::WriteOperation write_op(sem), write_op_no_block;
  LibXR::Thread::Sleep(3000);
  for (int i = 0; i < 512; i++)
  {
    buffer[i] = i;
  }
  while (1)
  {
#if 1
    cdc.Read({buffer, 0}, read_op);
    auto size = cdc.read_port_->Size();
    counter += size;
    cdc.Read({buffer, size}, read_op_no_block);
#else
    cdc.Write({buffer, 1024}, write_op_no_block);
    // LibXR::Thread::Sleep(10);
#endif
  }
#endif

  while (1)
  {
    LibXR::Thread::Sleep(1000);
  }
}
