
#include <stdint.h>
#include <stdbool.h>

#include "sleepus.h"

#include "config.h"

//#include "stm32f4xx.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "usec_time.h"

#define TICK_PER_US (FREERTOS_MCU_CLOCK_HZ / (8 * 1e6))

static bool isInit = false;

void sleepus(uint32_t us)
{
  if (!isInit) {
    initUsecTimer();
    isInit = true;
  }

  uint64_t start = usecTimestamp();

  while ((start+us) > usecTimestamp());
}
