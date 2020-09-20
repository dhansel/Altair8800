// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef PROG_TOOLS_H
#define PROG_TOOLS_H

#include <Arduino.h>

uint16_t prog_tools_copy_diag();
uint16_t prog_tools_copy_exerciser();
uint16_t prog_tools_copy_turnmon();
uint16_t prog_tools_copy_multiboot();
uint16_t prog_tools_copy_diskboot();
uint16_t prog_tools_copy_tdiskboot();
uint16_t prog_tools_copy_rdos10();
uint16_t prog_tools_copy_hdbl();
uint16_t prog_tools_copy_calc();
uint16_t prog_tools_copy_statustest();
uint16_t prog_tools_copy_serialirqtest();
uint16_t prog_tools_copy_musicsys();
uint16_t prog_tools_copy_adexer();
uint16_t prog_tools_copy_vdmcuter();

#endif
