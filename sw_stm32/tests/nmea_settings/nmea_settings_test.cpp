// Host test for the setting values exchanged between a host (XCSoar) and the
// CAN bus (issue #278), checked against the interface definitions in doc_larus:
//
//   documentation/Larus_NMEA_Protocol.md, $PLARS and $g:
//     MC m/s, BAL fraction 0.000-1.000, BUGS % (0-50, example "15"),
//     QNH hPa, CIR 1 = circling, 0 = cruise; $g,s0 = Vario, $g,s1 = SpeedToFly
//   documentation/can_details/object_directory/generic.md, set_config:
//     mac_cready f32 m/s, bugs f32 factor, qnh f32 Pa,
//     vario_mode u8 0 = Vario, 1 = SpeedToFly, water_ballast_fraction f32
//
//   nmea_to_can: real NMEA_listener.cpp, fed byte by byte through UART1,
//                CAN packets captured from CAN_enqueue()
//   can_to_nmea: real format_PLARS() from sw_algorithms_lib, called with the
//                values in CAN units as the CAN listener passes them on

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include "generic_CAN_driver.h"
#include "CAN_output.h"
#include "NMEA_format.h"

void NMEA_listener_task_runnable( void *);

// ---- simulated UART1 and CAN output ---------------------------------------

static std::string uart_input;
static size_t uart_pos;
static std::vector<CANpacket> can_output;

struct input_exhausted {};

bool UART1_Receive( uint8_t *pRxByte, uint32_t)
{
  if( uart_pos >= uart_input.size())
    throw input_exhausted(); // leaves the listener's endless loop
  *pRxByte = (uint8_t) uart_input[uart_pos++];
  return true;
}

bool CAN_enqueue( const CANpacket &p, unsigned)
{
  can_output.push_back( p);
  return true;
}

static std::string with_checksum( const char *body) // body starts with '$'
{
  unsigned char checksum = 0;
  for( const char *p = body + 1; *p; ++p)
    checksum ^= (unsigned char) *p;
  char tail[8];
  snprintf( tail, sizeof tail, "*%02X\r\n", checksum);
  return std::string( body) + tail;
}

static std::vector<CANpacket> listener_output( const char *sentence_body)
{
  uart_input = with_checksum( sentence_body);
  uart_pos = 0;
  can_output.clear();
  try
    {
      NMEA_listener_task_runnable( 0);
    }
  catch( input_exhausted &)
    {
    }
  return can_output;
}

// ---- checks ---------------------------------------------------------------

static int failures;

static void check( bool ok, const char *name, const std::string &detail)
{
  printf( "%s %-44s %s\n", ok ? "PASS" : "FAIL", name, detail.c_str());
  if( not ok)
    ++failures;
}

static std::string fmt( const char *format, ...) __attribute__((format(printf, 1, 2)));
static std::string fmt( const char *format, ...)
{
  char buffer[200];
  va_list args;
  va_start( args, format);
  vsnprintf( buffer, sizeof buffer, format, args);
  va_end( args);
  return buffer;
}

static void expect_float_item( const char *name, const char *sentence,
                               uint16_t config_id, float expected, float tolerance, const char *unit)
{
  std::vector<CANpacket> out = listener_output( sentence);
  if( out.size() != 1)
    {
      check( false, name, fmt( "%s -> %zu CAN packets, expected 1", sentence, out.size()));
      return;
    }
  const CANpacket &p = out[0];
  bool ok = p.dlc == 8 && p.data_h[0] == config_id && fabsf( p.data_f[1] - expected) <= tolerance;
  check( ok, name, fmt( "%s -> config_id 0x%04x, %g %s (expected 0x%04x, %g %s)",
                        sentence, p.data_h[0], p.data_f[1], unit, config_id, expected, unit));
}

static void expect_vario_mode( const char *name, const char *sentence, int expected)
{
  std::vector<CANpacket> out = listener_output( sentence);
  if( out.size() != 1)
    {
      check( false, name, fmt( "%s -> %zu CAN packets, expected vario_mode %d", sentence, out.size(), expected));
      return;
    }
  const CANpacket &p = out[0];
  bool ok = p.data_h[0] == SYSWIDECONFIG_ITEM_ID_VARIO_MODE && p.data_b[2] == expected;
  check( ok, name, fmt( "%s -> config_id 0x%04x, vario_mode %d (expected %d)",
                        sentence, p.data_h[0], p.data_b[2], expected));
}

static void test_nmea_to_can( void)
{
  printf( "NMEA from the host -> CAN set_config (NMEA_listener.cpp)\n");
  expect_float_item( "MC m/s -> mac_cready m/s", "$PLARS,H,MC,2.1",
                     SYSWIDECONFIG_ITEM_ID_MC, 2.1f, 1e-4f, "m/s");
  expect_float_item( "BAL fraction -> water_ballast_fraction", "$PLARS,H,BAL,0.752",
                     SYSWIDECONFIG_ITEM_ID_BALLAST_FRACTION, 0.752f, 1e-4f, "");
  expect_float_item( "BUGS % -> bugs factor", "$PLARS,H,BUGS,15",
                     SYSWIDECONFIG_ITEM_ID_BUGS, 1.15f, 1e-4f, "");
  expect_float_item( "QNH hPa -> qnh Pa", "$PLARS,H,QNH,1031.4",
                     SYSWIDECONFIG_ITEM_ID_QNH, 103140.0f, 1.0f, "Pa");
  expect_vario_mode( "CIR,0 (cruise) -> vario_mode SpeedToFly", "$PLARS,H,CIR,0", 1);
  expect_vario_mode( "CIR,1 (circling) -> vario_mode Vario", "$PLARS,H,CIR,1", 0);
  expect_vario_mode( "$g,s0 (Vario) -> vario_mode Vario", "$g,s0", 0);
  expect_vario_mode( "$g,s1 (SpeedToFly) -> vario_mode SpeedToFly", "$g,s1", 1);
}

static bool valid_nmea( const std::string &line, std::string &body)
{
  size_t star = line.find( '*');
  if( line.empty() || line[0] != '$' || star == std::string::npos || line.size() != star + 5
      || line.compare( star + 3, 2, "\r\n") != 0)
    return false;
  unsigned char checksum = 0;
  for( size_t i = 1; i < star; ++i)
    checksum ^= (unsigned char) line[i];
  char hex[3];
  snprintf( hex, sizeof hex, "%02X", checksum);
  body = line.substr( 0, star);
  return line.compare( star + 1, 2, hex) == 0;
}

static std::string format_setting( float value, parameter_type type)
{
  char buffer[100];
  memset( buffer, 0, sizeof buffer);
  char *p = buffer;
  format_PLARS( value, type, p);
  return buffer;
}

static void expect_plars_value( const char *name, float can_value, parameter_type type,
                                const char *key, double expected, double tolerance, const char *unit)
{
  std::string line = format_setting( can_value, type), body;
  std::string prefix = std::string( "$PLARS,L,") + key + ",";
  bool ok = valid_nmea( line, body) && body.compare( 0, prefix.size(), prefix) == 0;
  double value = ok ? atof( body.c_str() + prefix.size()) : NAN;
  ok = ok && fabs( value - expected) <= tolerance;
  check( ok, name, fmt( "%g -> %s (expected %s%g %s)", can_value,
                        body.empty() ? "invalid sentence" : body.c_str(), prefix.c_str(), expected, unit));
}

static void expect_plars_text( const char *name, float can_value, parameter_type type, const char *expected)
{
  std::string line = format_setting( can_value, type), body;
  bool ok = valid_nmea( line, body) && body == expected;
  check( ok, name, fmt( "%g -> %s (expected %s)", can_value,
                        body.empty() ? "invalid sentence" : body.c_str(), expected));
}

static void test_can_to_nmea( void)
{
  printf( "CAN setting -> NMEA to the host (format_PLARS)\n");
  expect_plars_value( "mac_cready m/s -> MC m/s", 1.3f, MC_CREADY, "MC", 1.3, 0.05, "m/s");
  expect_plars_value( "water_ballast_fraction -> BAL", 0.752f, BALLAST, "BAL", 0.752, 0.0005, "");
  expect_plars_value( "bugs factor 1.15 -> BUGS 15 %", 1.15f, BUGS, "BUGS", 15.0, 0.05, "%");
  expect_plars_value( "bugs factor 1.0 -> BUGS 0 %", 1.0f, BUGS, "BUGS", 0.0, 0.05, "%");
  expect_plars_value( "qnh Pa -> QNH hPa", 101320.0f, QNH, "QNH", 1013.2, 0.05, "hPa");
  expect_plars_text( "vario_mode Vario -> CIR,1 (circling)", 0.0f, VARIO_MODE, "$PLARS,L,CIR,1");
  expect_plars_text( "vario_mode SpeedToFly -> CIR,0 (cruise)", 1.0f, VARIO_MODE, "$PLARS,L,CIR,0");
}

int main( int argc, char **argv)
{
  const char *which = argc > 1 ? argv[1] : "all";
  bool all = strcmp( which, "all") == 0;
  if( all || strcmp( which, "nmea_to_can") == 0)
    test_nmea_to_can();
  if( all || strcmp( which, "can_to_nmea") == 0)
    test_can_to_nmea();
  printf( "%d check(s) failed\n", failures);
  return failures ? 1 : 0;
}
