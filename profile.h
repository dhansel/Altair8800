// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef PROFILE_H
#define PROFILE_H

#include "config.h"

#if USE_PROFILING_DETAIL>0
extern unsigned long prof_counter_detail, prof_opcode_count[256];
#define PROFILE_COUNT_OPCODE(n) prof_opcode_count[n]++
#else
#define PROFILE_COUNT_OPCODE(n) while(0)
#endif

void profile_setup();
void profile_reset();
void profile_enable(bool b);

#endif
