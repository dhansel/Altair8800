// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef PROG_TOOLS_H
#define PROG_TOOLS_H

#include <Arduino.h>

uint16_t prog_tools_copy_diag(byte *dst);
uint16_t prog_tools_copy_exerciser(byte *dst);
uint16_t prog_tools_copy_turnmon(byte *dst);
uint16_t prog_tools_copy_multiboot(byte *dst);
uint16_t prog_tools_copy_diskboot(byte *dst);
uint16_t prog_tools_copy_hdbl(byte *dst);
uint16_t prog_tools_copy_calc(byte *dst);
uint16_t prog_tools_copy_statustest(byte *dst);
uint16_t prog_tools_copy_serialirqtest(byte *dst);
uint16_t prog_tools_copy_musicsys(byte *dst);
uint16_t prog_tools_copy_adexer(byte *dst);

#endif
