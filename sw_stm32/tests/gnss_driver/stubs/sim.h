// Host test build: simulated time, UART3 receive DMA and task notification
// for the GNSS driver (implemented in sim.cpp).
#ifndef SIM_H_
#define SIM_H_
#include <stdint.h>

uint64_t sim_now_us (void);
bool sim_notify_wait (uint32_t ticks);	// true: DMA half/full transfer notification
uint32_t sim_notify_clear (void);
void sim_delay_ms (unsigned ms);
bool sim_dma_start (uint8_t *buffer, unsigned size, bool circular);
void sim_dma_stop (void);
unsigned sim_dma_remaining (void);
void sim_assert (bool ok, const char *what, const char *file, int line);
#endif
