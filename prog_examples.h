// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef PROG_EXAMPLES_H
#define PROG_EXAMPLES_H

#include <Arduino.h>

bool prog_examples_read_start(byte prognum);
bool prog_examples_read_next(byte dev, byte *b);

#endif
