// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef PROG_PS2_H
#define PROG_PS2_H

#include <Arduino.h>

uint16_t prog_ps2_copy_monitor();

void prog_ps2_read_start();
bool prog_ps2_read_next(byte *b);

#endif
