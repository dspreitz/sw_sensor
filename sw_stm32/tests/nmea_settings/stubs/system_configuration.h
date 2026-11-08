// Host test build: the library's host configuration plus what
// NMEA_listener.cpp needs from the firmware's system_configuration.h.
#ifndef SRC_SYSTEM_CONFIGURATION_H_
#define SRC_SYSTEM_CONFIGURATION_H_

#include <assert.h>

#define GIT_TAG_DEC 0x12345678 // dummy
#define DEVELOPMENT_ADDITIONS		0
#define USE_HARDWARE_EEPROM		0
#define DISABLE_SAT_COMPASS		0
#define PRINT_3D_MAG_PARAMETERS		0
#define LIMIT_DENSITY_CORRECTION( x)
#define ASSERT assert

#define NMEA_LISTEN_PRIORITY		0

#endif
