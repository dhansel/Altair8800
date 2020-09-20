// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef FILESYS
#define FILESYS

#include <Arduino.h>

byte filesys_open_write(char nm1, char nm2);
byte filesys_open_read(char nm1, char nm2);
bool filesys_exists(char nm1, char nm2);

bool filesys_write_char(byte fid, byte c);
bool filesys_write_data(byte fid, const void *data, uint16_t len);

bool     filesys_eof(byte fid);
bool     filesys_is_write(byte fid);
bool     filesys_is_read(byte fid);

uint32_t filesys_getpos(byte fid);
bool     filesys_seek(byte fid, uint32_t pos);

bool     filesys_read_char(byte fid, byte *c);
uint16_t filesys_read_data(byte fid, void *data, uint16_t len);

void filesys_close(byte fid);

bool     filesys_write_file(char nm1, char nm2, const void *data, uint16_t len);
uint16_t filesys_read_file(char nm1, char nm2, void *data, uint16_t len);

void   filesys_print_dir();
void   filesys_manage();

void filesys_setup();

#endif
