#ifndef CPUCORE_H
#define CPUCORE_H

#include "config.h"

#define PS_CARRY       0x01
#define PS_PARITY      0x04
#define PS_HALFCARRY   0x10
#define PS_ZERO        0x40
#define PS_SIGN        0x80

#if CPUCORE_VERSION == 1
#include "cpucore_v1.h"
#elif CPUCORE_VERSION == 2
#include "cpucore_v2.h"
#else
#error "invalid CPU core emulator version"
#endif

#endif
