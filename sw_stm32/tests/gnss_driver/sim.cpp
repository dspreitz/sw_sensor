// Simulated time, UART3 receive DMA and task notification for running
// GNSS_driver.cpp on the host. The recorded messages arrive byte by byte at
// 115200 Bd; the DMA raises the half and full transfer notification like
// the STM32, in normal mode (stops when full) or circular mode (wraps). Every 10 ms the "communicator"
// polls GNSS_new_data_ready like communicator.cpp and takes a snapshot of
// the coordinates.
#include "sim_data.h"
#include "stubs/sim.h"
#include <cstdio>
#include <cstdlib>

namespace
{
uint64_t now_us = 0;
uint64_t next_poll_us = 10000;
size_t next_byte = 0;
const std::vector<sim_byte_t> *stream = nullptr;
uint64_t end_us = 0;
uint8_t *dma_buffer = nullptr;
unsigned dma_size = 0, dma_count = 0;
bool dma_active = false;
bool dma_circular = false;
unsigned pending_notifications = 0;
sim_poll_callback_t poll_callback = nullptr;

void deliver (const sim_byte_t &b)
{
  if (! dma_active)
    return; // the UART has nowhere to put it: lost
  dma_buffer[dma_count++] = b.value;
  if (dma_count == dma_size / 2)
    ++pending_notifications; // half transfer
  if (dma_count == dma_size)
    {
      ++pending_notifications; // transfer complete
      if (dma_circular)
	dma_count = 0;
      else
	dma_active = false; // normal mode: the stream stops
    }
}

// advance to 'target', or until a notification is raised if 'stop_on_notify'
void advance (uint64_t target, bool stop_on_notify)
{
  while (true)
    {
      uint64_t next_event = target;
      if (next_byte < stream->size () && (*stream)[next_byte].time_us < next_event)
	next_event = (*stream)[next_byte].time_us;
      if (next_poll_us < next_event)
	next_event = next_poll_us;
      if (next_event > end_us)
	throw sim_end_of_data ();

      now_us = next_event;
      if (now_us == next_poll_us)
	{
	  next_poll_us += 10000;
	  poll_callback (now_us);
	}
      if (next_byte < stream->size () && (*stream)[next_byte].time_us == now_us)
	{
	  deliver ((*stream)[next_byte++]);
	  if (stop_on_notify && pending_notifications)
	    return;
	}
      if (now_us >= target)
	return;
    }
}
}

void sim_reset (const std::vector<sim_byte_t> &bytes, sim_poll_callback_t poll)
{
  stream = &bytes;
  poll_callback = poll;
  now_us = 0;
  next_poll_us = 10000;
  next_byte = 0;
  end_us = bytes.empty () ? 0 : bytes.back ().time_us + 1000000;
  dma_active = false;
  pending_notifications = 0;
}

uint64_t sim_now_us (void) { return now_us; }

bool sim_notify_wait (uint32_t ticks)
{
  if (pending_notifications == 0)
    advance (now_us + 1000ull * ticks, true);
  if (pending_notifications == 0)
    return false;
  pending_notifications = 0; // xTaskNotifyWait() clears the value on exit
  return true;
}

uint32_t sim_notify_clear (void)
{
  uint32_t value = pending_notifications;
  pending_notifications = 0;
  return value;
}

void sim_delay_ms (unsigned ms) { advance (now_us + 1000ull * ms, false); }

bool sim_dma_start (uint8_t *buffer, unsigned size, bool circular)
{
  dma_circular = circular;
  if (dma_active)
    return false;
  dma_buffer = buffer;
  dma_size = size;
  dma_count = 0;
  dma_active = true;
  return true;
}

void sim_dma_stop (void) { dma_active = false; }

unsigned sim_dma_remaining (void) { return dma_size - dma_count; }

void sim_assert (bool ok, const char *what, const char *file, int line)
{
  if (! ok)
    {
      std::fprintf (stderr, "ASSERT(%s) failed at %s:%d\n", what, file, line);
      std::abort ();
    }
}
