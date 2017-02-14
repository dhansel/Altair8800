// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef BREAKPOINT_H
#define BREAKPOINT_H

#include "config.h"
#include "host.h"

#if MAX_BREAKPOINTS > 0

#include <Arduino.h>
extern byte numBreakpoints;
void break_check_do(uint16_t addr);
inline void breakpoint_check(uint16_t addr) { if( numBreakpoints>0 ) break_check_do(addr); }

void breakpoint_add(uint16_t addr);
void breakpoint_remove_last();
void breakpoint_print();

#else

#define breakpoint_check(addr) 0
#define breakpoint_add(addr) while(0);
#define breakpoint_remove_last() while(0);
#define breakpoint_print() while(0);

#endif

#endif
