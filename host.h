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
#elif defined(__MK66FX1M0__)
#include "host_teensy36.h"
#else
#error requires Arduino Mega2560, Arduino Due or Windows/Linux PC
#endif

typedef void (*host_serial_receive_callback_tp)(byte iface, byte b);


// handling front panel switches
bool     host_read_function_switch(byte i);
bool     host_read_function_switch_debounced(byte i);
bool     host_read_function_switch_edge(byte i);
uint16_t host_read_function_switches_edge();
void     host_reset_function_switch_state();


// handling serial interfaces
void        host_serial_setup(byte iface, uint32_t baud, uint32_t config, bool set_primary_interface = true);
void        host_serial_end(byte i);
bool        host_serial_ok(byte i);
int         host_serial_available(byte i);
int         host_serial_available_for_write(byte i);
int         host_serial_peek(byte i);
int         host_serial_read(byte i);
void        host_serial_flush(byte i);
size_t      host_serial_write(byte i, uint8_t b);
size_t      host_serial_write(byte i, const char *buf, size_t n);
host_serial_receive_callback_tp host_serial_set_receive_callback(byte iface, host_serial_receive_callback_tp f);
const char *host_serial_port_name(byte i);
bool        host_serial_port_baud_limits(byte i, uint32_t *min, uint32_t *max);
bool        host_serial_port_has_configs(byte i);


// if the host provides a filesystem that the simulator can use then it should define
// HOST_HAS_FILESYS, HOST_FILESYS_FILE_TYPE and HOST_FILESYS_DIR_TYPE and
// all the functions below
#ifdef HOST_HAS_FILESYS
HOST_FILESYS_FILE_TYPE host_filesys_file_open(const char *filename, bool write);
uint32_t host_filesys_file_read(HOST_FILESYS_FILE_TYPE &f, uint32_t len, void *buffer);
uint32_t host_filesys_file_write(HOST_FILESYS_FILE_TYPE &f, uint32_t len, const void *buffer);
uint32_t host_filesys_file_set(HOST_FILESYS_FILE_TYPE &f, uint32_t len, byte b);
void     host_filesys_file_flush(HOST_FILESYS_FILE_TYPE &f);
bool     host_filesys_file_seek(HOST_FILESYS_FILE_TYPE &f, uint32_t pos);
uint32_t host_filesys_file_pos(HOST_FILESYS_FILE_TYPE &f);
bool     host_filesys_file_eof(HOST_FILESYS_FILE_TYPE &f);
void     host_filesys_file_close(HOST_FILESYS_FILE_TYPE &f);
bool     host_filesys_file_exists(const char *filename);
bool     host_filesys_file_remove(const char *filename);
bool     host_filesys_file_rename(const char *from, const char *to);
uint32_t host_filesys_file_size(const char *filename);
HOST_FILESYS_DIR_TYPE host_filesys_dir_open();
const char *host_filesys_dir_nextfile(HOST_FILESYS_DIR_TYPE &d);
void    host_filesys_dir_rewind(HOST_FILESYS_DIR_TYPE &d);
void    host_filesys_dir_close(HOST_FILESYS_DIR_TYPE &d);
bool    host_filesys_ok();
#endif


// at a minimum the host must provide persistent storage memory to which 
// we can write and read, implementing our own mini-filesystem
bool host_storage_init(bool write);
void host_storage_close();
void host_storage_write(const void *data, uint32_t addr, uint32_t len);
void host_storage_read(void *data,  uint32_t addr, uint32_t len);
void host_storage_move(uint32_t to, uint32_t from, uint32_t len);
void host_storage_invalidate();


// miscellaneous
void host_copy_flash_to_ram(void *dst, const void *src, uint32_t len);
uint32_t host_get_random();
void host_system_info();
void host_setup();


#endif
