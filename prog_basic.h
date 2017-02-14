// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef PROG_BASIC_H
#define PROG_BASIC_H

#include <Arduino.h>

uint16_t prog_basic_copy_4k(byte *dst);
uint16_t prog_basic_copy_16k(byte *dst);
byte prog_basic_read_16k(uint16_t addr);

#endif
