// Host test build: the FreeRTOS calls used by NMEA_listener.cpp.
#ifndef FREERTOS_WRAPPER_H_
#define FREERTOS_WRAPPER_H_
#include <stdint.h>
#include <string.h>

#define portMAX_DELAY 0xffffffffUL

inline void delay( unsigned) {}
inline void vTaskDelay( unsigned) {}

//! The listener's task object is defined at file scope; the test calls the
//! runnable directly, so the task is never started.
class RestrictedTask
{
public:
  template <typename... Args> RestrictedTask( Args...) {}
};

//! communicator.h (included from the listener's own directory) declares a queue.
template <typename type> class Queue
{
public:
  template <typename... Args> Queue( Args...) {}
  bool send( const type &, unsigned) { return true; }
};
#endif
