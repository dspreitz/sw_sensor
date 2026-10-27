// Host test for UBX_receiver: replays recorded receiver output and checks
// synthetic cases. Build and run (from sw_stm32/):
//   g++ -std=gnu++14 -Wall -Wextra -I Drivers/Custom tests/UBX_receiver_test.cpp Drivers/Custom/UBX_receiver.cpp -o /tmp/ubx_test && /tmp/ubx_test tests/data

#include "UBX_receiver.h"
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf ("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++failures; } } while (0)

typedef std::vector<uint8_t> bytes;

static uint32_t u32 (const uint8_t *p)
{
  return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t) p[3] << 24);
}

static bytes frame (uint8_t cls, uint8_t id, bytes payload)
{
  bytes f = { 0xb5, 'b', cls, id, (uint8_t) payload.size (), (uint8_t) (payload.size () >> 8) };
  f.insert (f.end (), payload.begin (), payload.end ());
  uint8_t a = 0, b = 0;
  for (size_t i = 2; i < f.size (); ++i)
    {
      a += f[i];
      b += a;
    }
  f.push_back (a);
  f.push_back (b);
  return f;
}

static bytes pvt (uint32_t itow)
{
  bytes p (UBX_NAV_PVT_LENGTH, 0);
  memcpy (p.data (), &itow, 4);
  p[20] = 3;
  p[21] = 1;
  return frame (1, UBX_NAV_PVT_ID, p);
}

static bytes heading (uint8_t id, uint32_t itow)
{
  bytes p (id == UBX_NAV_RELPOSNED_ID ? UBX_NAV_RELPOSNED_LENGTH : UBX_NAV_DAHEADING_LENGTH, 0);
  memcpy (p.data () + 4, &itow, 4);
  return frame (1, id, p);
}

struct epoch_t
{
  uint32_t pvt_itow;
  bool second;
  uint32_t second_itow;
};

struct replay_t
{
  UBX_receiver rx;
  std::vector<epoch_t> epochs;

  explicit replay_t (uint8_t id) : rx (id) {}

  void take (void)
  {
    const uint8_t *e = rx.epoch ();
    epoch_t ep = { u32 (e + 6), rx.epoch_has_second (), 0 };
    if (ep.second)
      ep.second_itow = u32 (e + UBX_PVT_FRAME_SIZE + 6 + 4);
    else
      for (int i = 0; i < UBX_SECOND_FRAME_MAX; ++i)
	CHECK (e[UBX_PVT_FRAME_SIZE + i] == 0);
    epochs.push_back (ep);
  }
  void feed (const bytes &b)
  {
    for (uint8_t c : b)
      if (rx.feed (c))
	take ();
  }
  void flush (void)
  {
    if (rx.flush ())
      take ();
  }
};

static bytes cat (std::initializer_list<bytes> parts)
{
  bytes out;
  for (const bytes &p : parts)
    out.insert (out.end (), p.begin (), p.end ());
  return out;
}

static void synthetic (void)
{
  const uint8_t D = UBX_NAV_DAHEADING_ID;
  {
    // normal order, heading first (seen on the X20D), and a missing heading
    replay_t r (D);
    r.feed (cat ({ pvt (100), heading (D, 100), heading (D, 200), pvt (200), pvt (300) }));
    r.flush ();
    CHECK (r.epochs.size () == 3);
    CHECK (r.epochs[0].pvt_itow == 100 && r.epochs[0].second && r.epochs[0].second_itow == 100);
    CHECK (r.epochs[1].pvt_itow == 200 && r.epochs[1].second && r.epochs[1].second_itow == 200);
    CHECK (r.epochs[2].pvt_itow == 300 && !r.epochs[2].second);
  }
  {
    // a heading message of another epoch completes the waiting PVT alone and
    // stays for its own PVT
    replay_t r (D);
    r.feed (cat ({ pvt (100), heading (D, 200), pvt (200) }));
    CHECK (r.epochs.size () == 2);
    CHECK (r.epochs[0].pvt_itow == 100 && !r.epochs[0].second);
    CHECK (r.epochs[1].pvt_itow == 200 && r.epochs[1].second);
  }
  {
    // other UBX messages, NMEA and garbage in between; a false sync with a
    // huge length must not swallow the following epoch
    replay_t r (UBX_NAV_RELPOSNED_ID);
    std::string nmea = "$GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    bytes junk (nmea.begin (), nmea.end ());
    r.feed (cat ({ pvt (100), frame (1, 0x20, bytes (16, 7)), heading (UBX_NAV_RELPOSNED_ID, 100), junk,
		   { 0xb5, 'b', 1, 2, 0xff, 0xff }, pvt (200), heading (UBX_NAV_RELPOSNED_ID, 200) }));
    CHECK (r.epochs.size () == 2);
    CHECK (r.epochs[0].second && r.epochs[1].second && r.epochs[1].pvt_itow == 200);
    CHECK (r.rx.skipped_messages == 1);
  }
  {
    // a corrupted heading message: the PVT comes without it after the line goes idle
    replay_t r (D);
    bytes h = heading (D, 100);
    h[20] ^= 0x10;
    r.feed (cat ({ pvt (100), h }));
    CHECK (r.epochs.empty ());
    r.flush ();
    CHECK (r.epochs.size () == 1 && !r.epochs[0].second);
    CHECK (r.rx.checksum_errors == 1);
  }
  {
    // PVT only (M9N, F9P + F9H): every PVT is an epoch, heading messages are skipped
    replay_t r (0);
    r.feed (cat ({ pvt (100), heading (UBX_NAV_RELPOSNED_ID, 100), pvt (200) }));
    CHECK (r.epochs.size () == 2 && !r.epochs[0].second && !r.epochs[1].second);
  }
}

// all valid UBX frames of a byte stream, independent of UBX_receiver
static std::vector<bytes> frames_in (const bytes &d)
{
  std::vector<bytes> out;
  for (size_t i = 0; i + 8 <= d.size ();)
    {
      if (d[i] != 0xb5 || d[i + 1] != 'b')
	{
	  ++i;
	  continue;
	}
      size_t n = d[i + 4] | (d[i + 5] << 8);
      if (i + n + 8 > d.size ())
	break;
      uint8_t a = 0, b = 0;
      for (size_t k = i + 2; k < i + 6 + n; ++k)
	{
	  a += d[k];
	  b += a;
	}
      if (a != d[i + 6 + n] || b != d[i + 7 + n])
	{
	  ++i;
	  continue;
	}
      out.push_back (bytes (d.begin () + i, d.begin () + i + n + 8));
      i += n + 8;
    }
  return out;
}

static void replay (const std::string &base, uint8_t id)
{
  std::ifstream fu (base + ".ubx", std::ios::binary);
  bytes data ((std::istreambuf_iterator<char> (fu)), std::istreambuf_iterator<char> ());
  std::ifstream fc (base + ".csv");
  CHECK (!data.empty () && fc.good ());
  if (data.empty () || !fc.good ())
    return;

  // expected: every PVT once; with its heading message if one with the same
  // iTOW comes right before (X20D) or after it (F9P), other messages ignored
  std::vector<epoch_t> expected;
  bool heading_before = false;
  uint32_t heading_itow = 0;
  for (const bytes &f : frames_in (data))
    if (f[2] == 1 && f[3] == UBX_NAV_PVT_ID)
      {
	expected.push_back ({ u32 (&f[6]), heading_before && heading_itow == u32 (&f[6]), 0 });
	heading_before = false;
      }
    else if (f[2] == 1 && f[3] == id)
      {
	heading_itow = u32 (&f[10]);
	heading_before = expected.empty () || expected.back ().second || expected.back ().pvt_itow != heading_itow;
	if (!heading_before)
	  expected.back ().second = true;
      }

  // feed the chunks as they arrived, flush when the line was idle for 20 ms (as the driver does)
  replay_t r (id);
  std::string line;
  std::getline (fc, line);
  double last_t = 0;
  while (std::getline (fc, line))
    {
      double t;
      size_t off, len;
      char c1, c2;
      std::istringstream (line) >> t >> c1 >> off >> c2 >> len;
      if (t - last_t > 0.020)
	r.flush ();
      last_t = t;
      r.feed (bytes (data.begin () + off, data.begin () + off + len));
    }
  r.flush ();

  size_t with_second = 0, expected_second = 0;
  CHECK (r.epochs.size () == expected.size ());
  for (size_t i = 0; i < r.epochs.size () && i < expected.size (); ++i)
    {
      CHECK (r.epochs[i].pvt_itow == expected[i].pvt_itow);
      CHECK (r.epochs[i].second == expected[i].second);
      if (r.epochs[i].second)
	CHECK (r.epochs[i].second_itow == r.epochs[i].pvt_itow);
      with_second += r.epochs[i].second;
      expected_second += expected[i].second;
    }
  CHECK (r.rx.checksum_errors == 0);
  std::printf ("%s: %zu epochs (%zu expected), %zu with heading message (%zu expected), %u other messages skipped\n",
	       base.c_str (), r.epochs.size (), expected.size (), with_second, expected_second,
	       (unsigned) r.rx.skipped_messages);
}

int main (int argc, char **argv)
{
  synthetic ();
  std::string dir = argc > 1 ? argv[1] : "tests/data";
  replay (dir + "/f9p_hpg151_baseboard-outputs_10hz_usb_30s", UBX_NAV_RELPOSNED_ID);
  replay (dir + "/f9p_hpg151_pvt-relposned_10hz_usb_30s", UBX_NAV_RELPOSNED_ID);
  replay (dir + "/f9p_hpg151_sapos-rtk-fixed_pvt-relposned_10hz_usb_30s", UBX_NAV_RELPOSNED_ID);
  replay (dir + "/x20d_hdg200_pvt-daheading_10hz_usb_30s", UBX_NAV_DAHEADING_ID);
  std::printf (failures ? "%d FAILED\n" : "all passed\n", failures);
  return failures != 0;
}
