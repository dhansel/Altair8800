// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef CPUCORE_I8080_H
#define CPUCORE_I8080_H

#include <Arduino.h>
#include "cpucore.h"

extern CPUFUN cpucore_i8080_opcodes[256];
void cpucore_i8080_print_registers();

#endif
