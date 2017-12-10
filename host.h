// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef HOST_H
#define HOST_H

#include "config.h"
#include <Arduino.h>

#if defined(__AVR_ATmega2560__)
#include "host_mega.h"
#elif defined(__SAM3X8E__)
#include "host_due.h"
#elif defined(_WIN32) || defined(__linux__)
#include "host_pc.h"
#else
#error requires Arduino Mega2560, Arduino Due or Windows/Linux PC
#endif


bool     host_read_function_switch(byte i);
bool     host_read_function_switch_debounced(byte i);
bool     host_read_function_switch_edge(byte i);
uint16_t host_read_function_switches_edge();
void     host_reset_function_switch_state();

void        host_serial_setup(byte iface, uint32_t baud, uint32_t config, bool set_primary_interface = true);
void        host_serial_end(byte i);
bool        host_serial_ok(byte i);
int         host_serial_available(byte i);
int         host_serial_available_for_write(byte i);
int         host_serial_peek(byte i);
int         host_serial_read(byte i);
void        host_serial_flush(byte i);
size_t      host_serial_write(byte i, uint8_t b);
const char *host_serial_port_name(byte i);
bool        host_serial_port_baud_limits(byte i, uint32_t *min, uint32_t *max);
bool        host_serial_port_has_configs(byte i);

void     host_copy_flash_to_ram(void *dst, const void *src, uint32_t len);

bool     host_file_exists(const char *filename);
int32_t  host_get_file_size(const char *filename);

uint32_t host_read_file(const char *filename, uint32_t offset, uint32_t len, void *buffer);
uint32_t host_write_file(const char *filename, uint32_t offset, uint32_t len, void *buffer);
uint32_t host_set_file(const char *filename, uint32_t offset, uint32_t len, byte b, bool keep_open = false);

void host_write_data(const void *data, uint32_t addr, uint32_t len);
void host_read_data(void *data,  uint32_t addr, uint32_t len);
void host_move_data(uint32_t to, uint32_t from, uint32_t len);

uint32_t host_get_random();

bool host_is_reset();

bool host_have_sd_card();
void host_system_info();

void host_setup();


#endif
