// Runs the real GNSS_driver.cpp + GNSS.cpp on the host against a recording of
// a ZED-X20D (NAV-PVT + NAV-DAHEADING, 10 Hz) and checks what reaches the
// communicator: every NAV-PVT once, nothing stale, heading only from the
// NAV-DAHEADING of the same epoch.
#include "sim_data.h"
#include "GNSS.h"
#include "GNSS_driver.h"
#include "signal_flight_event.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <set>

// as in communicator.cpp
D_GNSS_coordinates_t coordinates;
GNSS_type GNSS (coordinates);

namespace
{
unsigned events_timeout_half, events_timeout_full, events_data_error;
}

void signal_logger_event (uint32_t event)
{
  if ((event & 0xff) != DEBUGGER_DATA)
    return;
  // debug_points in signal_flight_event.h; numeric, as older library
  // versions don't define them (and their drivers don't send them)
  switch ((event >> 8) & 0xff)
    {
    case 1: ++events_timeout_half; break; // GNSS_TIMEOUT_HALF
    case 2: ++events_timeout_full; break; // GNSS_TIMEOUT_FULL
    case 3: ++events_data_error; break;   // GNSS_DATA_ERROR
    }
}

std::vector<recorded_message_t> read_recording (const std::string &path)
{
  std::vector<recorded_message_t> messages;
  std::ifstream in (path, std::ios::binary);
  while (true)
    {
      uint8_t header[6];
      if (! in.read ((char *) header, sizeof(header)))
	break;
      recorded_message_t m;
      m.time_us = header[0] | (header[1] << 8) | (header[2] << 16) | ((uint32_t) header[3] << 24);
      m.bytes.resize (header[4] | (header[5] << 8));
      if (! in.read ((char *) m.bytes.data (), m.bytes.size ()))
	break;
      messages.push_back (m);
    }
  return messages;
}

std::vector<sim_byte_t> uart_bytes (const std::vector<recorded_message_t> &messages, unsigned baud)
{
  std::vector<sim_byte_t> bytes;
  double byte_time_us = 10e6 / baud; // start + 8 data + stop bit
  double t = 0.0;
  for (const recorded_message_t &m : messages)
    {
      if (t < (double) m.time_us)
	t = (double) m.time_us;
      for (uint8_t b : m.bytes)
	{
	  t += byte_time_us;
	  bytes.push_back ({ (uint64_t) std::llround (t), b });
	}
    }
  return bytes;
}

namespace
{
struct published_t
{
  uint64_t time_us;
  uint32_t time_key; // ms of day from the coordinates
  bool heading;
  float heading_rad;
};
std::vector<published_t> published;

uint32_t time_key (unsigned h, unsigned m, unsigned s, int32_t nano)
{
  return ((h * 60 + m) * 60 + s) * 1000 + (int32_t) std::lround (nano / 1e6);
}

void communicator_poll (uint64_t now_us)
{
  if (! GNSS_new_data_ready.test_and_reset ())
    return;
  published.push_back ({ now_us,
      time_key (coordinates.hour, coordinates.minute, coordinates.second, coordinates.nano),
      (coordinates.sat_fix_type & SAT_HEADING) != 0, coordinates.relPosHeading });
}

template <typename T> T get (const std::vector<uint8_t> &b, size_t payload_offset)
{
  T value;
  std::memcpy (&value, b.data () + 6 + payload_offset, sizeof(T));
  return value;
}

unsigned failures;
void check (bool ok, const char *what)
{
  std::printf ("%s %s\n", ok ? "PASS" : "FAIL", what);
  if (! ok)
    ++failures;
}
}

int main (int argc, char **argv)
{
  if (argc < 2)
    {
      std::fprintf (stderr, "usage: %s RECORDING [--heading-delay-ms N] [-v]\n", argv[0]);
      return 2;
    }
  std::vector<recorded_message_t> messages = read_recording (argv[1]);
  bool verbose = false;
  int dah_delay_ms = -1; // < 0: timing as recorded
  for (int i = 2; i < argc; ++i)
    if (std::strcmp (argv[i], "-v") == 0)
      verbose = true;
    else if ((std::strcmp (argv[i], "--heading-delay-ms") == 0) && (i + 1 < argc))
      dah_delay_ms = std::atoi (argv[++i]);
  if (dah_delay_ms >= 0)
    {
      // send each NAV-DAHEADING this long after the start of the NAV-PVT of
      // its epoch, to probe the timeouts of the driver
      uint64_t pvt_time_us = 0;
      uint32_t pvt_itow = 0;
      for (recorded_message_t &m : messages)
	if (m.bytes[3] == 0x07)
	  {
	    pvt_time_us = m.time_us;
	    pvt_itow = get<uint32_t> (m.bytes, 0);
	  }
	else if (get<uint32_t> (m.bytes, 4) == pvt_itow)
	  m.time_us = pvt_time_us + 1000ull * dah_delay_ms;
    }

  // reference: the epochs in the recording
  std::map<uint32_t, uint32_t> pvt_time_key_by_itow;
  std::map<uint32_t, uint32_t> itow_by_time_key;
  std::map<uint32_t, float> valid_heading_by_itow; // rad, as GNSS.cpp converts it
  for (const recorded_message_t &m : messages)
    if (m.bytes[3] == 0x07)
      {
	uint32_t itow = get<uint32_t> (m.bytes, 0);
	uint32_t key = time_key (get<uint8_t> (m.bytes, 8), get<uint8_t> (m.bytes, 9),
				 get<uint8_t> (m.bytes, 10), get<int32_t> (m.bytes, 16));
	pvt_time_key_by_itow[itow] = key;
	itow_by_time_key[key] = itow;
      }
    else if (m.bytes[3] == 0x45 && (get<uint32_t> (m.bytes, 56) & 0x40))
      valid_heading_by_itow[get<uint32_t> (m.bytes, 4)] =
	  (float) get<int32_t> (m.bytes, 24) * 1.745329252e-7f;

  std::vector<sim_byte_t> bytes = uart_bytes (messages, 115200);
  sim_reset (bytes, communicator_poll);
  GNSS_configration_t gnss = GNSS_X20D;
  try
    {
      USART_3_runnable (&gnss);
    }
  catch (const sim_end_of_data &)
    {
    }

  // evaluate
  unsigned unknown = 0, repeated = 0, backwards = 0, heading_wrong = 0, heading_missing = 0;
  std::set<uint32_t> seen;
  uint32_t previous_key = 0;
  for (const published_t &p : published)
    {
      auto it = itow_by_time_key.find (p.time_key);
      if (it == itow_by_time_key.end ())
	{
	  ++unknown;
	  continue;
	}
      if (seen.count (p.time_key))
	++repeated;
      if (p.time_key < previous_key)
	++backwards;
      seen.insert (p.time_key);
      previous_key = p.time_key;

      auto h = valid_heading_by_itow.find (it->second);
      if (p.heading)
	{
	  if (h == valid_heading_by_itow.end () || std::fabs (h->second - p.heading_rad) > 1e-6f)
	    {
	      ++heading_wrong;
	      if (verbose)
		{
		  long source = -1;
		  for (auto &v : valid_heading_by_itow)
		    if (std::fabs (v.second - p.heading_rad) <= 1e-6f)
		      source = v.first;
		  std::printf ("  epoch iTOW %u: heading from iTOW %ld\n", it->second, source);
		}
	    }
	}
      else if (h != valid_heading_by_itow.end ())
	++heading_missing;
    }
  unsigned epochs = pvt_time_key_by_itow.size ();
  unsigned with_heading = valid_heading_by_itow.size ();
  std::printf ("recording: %u NAV-PVT epochs, %u with a valid NAV-DAHEADING\n", epochs, with_heading);
  std::printf ("published: %zu records, %zu different epochs; heading missing in %u, from another epoch in %u\n",
	       published.size (), seen.size (), heading_missing, heading_wrong);
  std::printf ("driver events: %u half timeouts, %u full timeouts, %u data errors\n",
	       events_timeout_half, events_timeout_full, events_data_error);

  char text[120];
  check (unknown == 0, "every published record is a NAV-PVT of the recording");
  std::snprintf (text, sizeof(text), "no stale record published again (%u repeated)", repeated);
  check (repeated == 0, text);
  check (backwards == 0, "records in time order");
  std::snprintf (text, sizeof(text), "at least 98%% of the epochs published (%.1f%%)", 100.0 * seen.size () / epochs);
  check (seen.size () * 100 >= epochs * 98u, text);
  std::snprintf (text, sizeof(text), "heading only from the NAV-DAHEADING of the same epoch (%u wrong)", heading_wrong);
  check (heading_wrong == 0, text);
  std::snprintf (text, sizeof(text), "heading in at least 95%% of the epochs that have one (%.1f%%)",
		 with_heading ? 100.0 * (with_heading - heading_missing) / with_heading : 100.0);
  check ((with_heading - heading_missing) * 100 >= with_heading * 95u, text);
  return failures ? 1 : 0;
}
