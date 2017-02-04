#ifndef HOST_H
#define HOST_H

#include "config.h"
#include <Arduino.h>

#if defined(__AVR_ATmega2560__)
#include "host_mega.h"
#elif defined(__SAM3X8E__)
#include "host_due.h"
#elif defined(_WIN32)
#include "host_pc.h"
#else
#error requires Arduino Mega2560, Arduino Due or Windows PC
#endif


void     host_serial_setup(byte iface, unsigned long baud, bool set_primary_interface = true);

bool     host_read_function_switch(byte i);
bool     host_read_function_switch_debounced(byte i);
bool     host_read_function_switch_edge(byte i);
uint16_t host_read_function_switches_edge();
void     host_reset_function_switch_state();

typedef void (*HostTimerFnTp)();
void host_interrupt_timer_setup(byte tid, uint32_t microseconds, HostTimerFnTp timer_fn);
void host_interrupt_timer_start(byte tid);
void host_interrupt_timer_stop(byte tid);
bool host_interrupt_timer_running(byte tid);

void host_copy_flash_to_ram(void *dst, const void *src, uint32_t len);

uint32_t host_read_file(const char *filename, uint32_t offset, uint32_t len, void *buffer);
uint32_t host_write_file(const char *filename, uint32_t offset, uint32_t len, void *buffer);

void host_write_data(const void *data, uint32_t addr, uint32_t len);
void host_read_data(void *data,  uint32_t addr, uint32_t len);
void host_move_data(uint32_t to, uint32_t from, uint32_t len);

uint32_t host_get_random();

void host_setup();


#endif
