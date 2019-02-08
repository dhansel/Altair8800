// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef PROG_BASIC_H
#define PROG_BASIC_H

#include <Arduino.h>

uint16_t prog_basic_copy_4k();
uint16_t prog_basic_copy_16k();
bool prog_basic_read_4k(uint16_t addr, byte *b);
byte prog_basic_read_16k(uint16_t addr);

#endif
