#include "FreeRTOS.h"
#include "app_main.h"
#include "core_riscv.h"
#include "math.h"
#include "task.h"

static void DefaultTask(void* pvParameters)
{
  (void)(pvParameters);
  app_main();
  while (1)
  {
    vTaskDelay(1000);
  }
}

void USB_RCC_Init(void)
{
  RCC_USBCLK48MConfig(RCC_USBCLK48MCLKSource_PLLCLK);
  RCC_USBFSCLKConfig(RCC_USBFSCLKSource_PLLCLK_Div3);
  RCC_USBHSPLLCLKConfig(RCC_HSBHSPLLCLKSource_HSE);
  RCC_USBHSConfig(RCC_USBPLL_Div2);
  RCC_USBHSPLLCKREFCLKConfig(RCC_USBHSPLLCKREFCLK_4M);
  RCC_USBHSPHYPLLALIVEcmd(ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBFS, ENABLE);
}

int main(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  SystemInit();
  SystemCoreClockUpdate();
  USB_RCC_Init();
  __enable_irq();
  xTaskCreate(DefaultTask, "DefaultTask", 6000, NULL, 3, NULL);
  vTaskStartScheduler();
  return 0;
}
