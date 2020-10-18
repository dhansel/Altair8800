// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef VDM1_H
#define VDM1_H

#include <Arduino.h>

extern uint16_t vdm1_mem_start, vdm1_mem_end;

#define vdm1_write_mem(a, v) { if( (a)<vdm1_mem_end && (a)>=vdm1_mem_start ) vdm1_write_mem_(a, v); }

void vdm1_write_mem_(uint16_t a, byte v);

void vdm1_receive(int size, byte *data);

void vdm1_set_iface(byte iface);
byte vdm1_get_iface();
void vdm1_set_dip(byte dip);
void vdm1_set_address(uint16_t a);
void vdm1_register_ports();
void vdm1_setup();

#endif
