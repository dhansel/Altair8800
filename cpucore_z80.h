// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef CPUCORE_Z80_H
#define CPUCORE_Z80_H

#include <Arduino.h>
#include "cpucore.h"

extern CPUFUN cpucore_z80_opcodes[256];
void cpucore_z80_print_registers();

#endif
