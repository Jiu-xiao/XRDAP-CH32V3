// SPDX-License-Identifier: Apache-2.0
#include "FreeRTOS.h"
#include "ch32v30x.h"
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
  SystemCoreClockUpdate();
  __enable_irq();
  xTaskCreate(AppTask, "app", 2048, NULL, 3, NULL);  // 8 KB; about 1 KB is used
  vTaskStartScheduler();
  for (;;)
  {
  }
}
