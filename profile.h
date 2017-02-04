#ifndef PROFILE_H
#define PROFILE_H

#include "config.h"

#if USE_PROFILING>0
extern unsigned long prof_timer, prof_cycle_counter, prof_counter;

#if USE_PROFILING_DETAIL>0
extern unsigned long prof_counter_detail, prof_opcode_count[256];
#define PROF_COUNT_OPCODE(n) prof_opcode_count[n]++
#else
#define PROF_COUNT_OPCODE(n) while(0)
#endif

void prof_print();
void prof_reset();

#define PROF_ADD_CYCLES(n) prof_cycle_counter += n
#define prof_check() {if( config_profiling_enabled() && --prof_counter==0 ) prof_print(); }

#else

#define PROF_ADD_CYCLES(n)   while(0)
#define PROF_COUNT_OPCODE(n) while(0)
#define prof_check()         while(0)
#define prof_toggle()        while(0)

#endif

#endif
