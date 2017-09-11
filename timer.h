// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>

#define MAX_TIMERS 10

// timers 0-4 must represent the four serial devices
#define TIMER_SIO      0
#define TIMER_ACR      1
#define TIMER_2SIO1    2
#define TIMER_2SIO2    3
#define TIMER_DRIVE    4
#define TIMER_RTC      5
#define TIMER_PRINTER  6
#define TIMER_THROTTLE 8
#define TIMER_PROFILE  9


extern uint32_t timer_cycle_counter, timer_cycle_counter_offset, timer_next_expire_cycles;

typedef void (*TimerFnTp)();
void timer_setup(byte tid, uint32_t microseconds, TimerFnTp timer_fn);
void timer_start(byte tid, uint32_t microseconds = 0, bool recurring = false);
void timer_stop(byte tid);
bool timer_running(byte tid);
uint32_t timer_get_period(byte tid);
void timer_check();
void timer_setup();

#define timer_get_cycles() (timer_cycle_counter_offset+timer_cycle_counter)

#define TIMER_ADD_CYCLES(n) if( (timer_cycle_counter+=(n)) >= timer_next_expire_cycles ) timer_check()

#endif
