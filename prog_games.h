// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef PROG_GAMES_H
#define PROG_GAMES_H

#include <Arduino.h>

uint16_t prog_games_copy_killbits(byte *dst);
uint16_t prog_games_copy_pong(byte *dst);
uint16_t prog_games_copy_pongterm(byte *dst);
uint16_t prog_games_copy_daisy(byte *dst);

#endif
