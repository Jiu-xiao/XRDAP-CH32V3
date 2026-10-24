// SPDX-License-Identifier: Apache-2.0
#include "FreeRTOS.h"
#include "ch32v30x.h"
#include "sysclk.h"
#include "task.h"

void app_main(void);

static void AppTask(void* arg)
{
  (void)arg;
  app_main();
  vTaskDelete(NULL);
}

int main(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  SystemInit();
  (void)XrDapSysClockInit();
  SystemCoreClockUpdate();
  __enable_irq();
  xTaskCreate(AppTask, "app", XRDAP_APP_STACK_WORDS, NULL, 3, NULL);  // per board
  vTaskStartScheduler();
  for (;;)
  {
  }
}
