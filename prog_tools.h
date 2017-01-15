#ifndef PROG_CPUTEST_H
#define PROG_CPUTEST_H

#include <Arduino.h>

uint16_t prog_tools_copy_diag(byte *dst);
uint16_t prog_tools_copy_exerciser(byte *dst);
uint16_t prog_tools_copy_turnmon(byte *dst);
uint16_t prog_tools_copy_diskboot(byte *dst);
uint16_t prog_tools_copy_calc(byte *dst);
uint16_t prog_tools_copy_statustest(byte *dst);
uint16_t prog_tools_copy_serialirqtest(byte *dst);
uint16_t prog_tools_copy_ps2(byte *dst);

#endif
