// Recorded UBX messages and their replay on the simulated UART3.
#ifndef SIM_DATA_H_
#define SIM_DATA_H_
#include <stdint.h>
#include <string>
#include <vector>

struct sim_byte_t
{
  uint64_t time_us;
  uint8_t value;
};

struct recorded_message_t
{
  uint64_t time_us; // arrival time of the first byte on the host
  std::vector<uint8_t> bytes; // complete UBX frame
};

struct sim_end_of_data {};

typedef void (*sim_poll_callback_t) (uint64_t now_us);

std::vector<recorded_message_t> read_recording (const std::string &path);
std::vector<sim_byte_t> uart_bytes (const std::vector<recorded_message_t> &messages, unsigned baud);
void sim_reset (const std::vector<sim_byte_t> &bytes, sim_poll_callback_t poll);
#endif
