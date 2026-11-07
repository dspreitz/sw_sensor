// Host test build: simulated FreeRTOS calls used by the GNSS driver.
#ifndef FREERTOS_WRAPPER_H_
#define FREERTOS_WRAPPER_H_
#include <stdint.h>
#include "sim.h"

typedef long BaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
#define pdTRUE 1
#define pdFALSE 0
#define INFINITE_WAIT 0xffffffff
#define pdMS_TO_TICKS(ms) ((TickType_t) (ms))

inline TaskHandle_t xTaskGetCurrentTaskHandle (void) { return (TaskHandle_t) 1; }
inline TickType_t xTaskGetTickCount (void) { return (TickType_t) (sim_now_us () / 1000); }
inline BaseType_t xTaskNotifyWait (uint32_t, uint32_t, uint32_t *value, TickType_t ticks)
{
  if (value) *value = 0;
  return sim_notify_wait (ticks) ? pdTRUE : pdFALSE;
}
inline uint32_t ulTaskNotifyTake (BaseType_t, TickType_t) { return sim_notify_clear (); }
inline void vTaskNotifyGiveFromISR (TaskHandle_t, BaseType_t *) {}
#define portEND_SWITCHING_ISR(x) (void) (x)
inline void delay (unsigned ms) { sim_delay_ms (ms); }
inline void drop_privileges (void) {}

class Mutex
{
public:
  Mutex (char * = 0) {}
  bool lock (unsigned = INFINITE_WAIT) { return true; }
  void release (void) {}
  void unlock (void) {}
};
#endif
