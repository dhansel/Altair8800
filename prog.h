// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef PROG_H
#define PROG_H

#include <Arduino.h>

#if defined(__AVR_ATmega2560__)
#define FP(x) (__FlashStringHelper *)(x)
#else
#define FP(x) x
#endif


byte  prog_find(const char *name);
PGM_P prog_get_name(byte n);
bool  prog_load(byte n, uint16_t *pc);
uint16_t prog_checksum_loader(const byte *tape, unsigned int tape_length);

bool prog_create_temporary_rom(uint16_t ramdst, const void *src, uint32_t length, const char *name);
bool prog_copy_to_ram(uint16_t ramdst, const void *src, uint32_t length);

#endif
