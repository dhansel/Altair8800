// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2020 David Hansel
// -----------------------------------------------------------------------------

#ifndef IO_H

#include "config.h"

typedef byte (*IOFUN_INP)(byte port);
typedef void (*IOFUN_OUT)(byte port, byte data);

void io_register_port_inp(byte port, IOFUN_INP f);
void io_register_port_out(byte port, IOFUN_OUT f);

byte io_inp(byte port);
void io_out(byte port, byte data);

void io_print_registered_ports();
void io_setup();

#endif
