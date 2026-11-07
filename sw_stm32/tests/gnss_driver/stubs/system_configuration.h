// Host test build: the parts of Core/Inc/system_configuration.h that the
// GNSS driver and GNSS.cpp use.
#ifndef SYSTEM_CONFIGURATION_H_
#define SYSTEM_CONFIGURATION_H_
#include <stdint.h>
#include "sim.h"
#define RUN_GNSS 1
#define MEASURE_GNSS_REFRESH_TIME 0
#define STANDARD_ISR_PRIORITY 14
#define ASSERT(x) sim_assert ((x), #x, __FILE__, __LINE__)
#endif
