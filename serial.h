// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef SERIAL_H
#define SERIAL_H

#include "Arduino.h"

#define DBG_FILEOPS_LEVEL 3

#if DBG_FILEOPS_LEVEL>0

#define DBG_FILEOPS(lvl, s) if(DBG_FILEOPS_LEVEL>=lvl) {Serial.print('['); Serial.print(F(s)); Serial.println(']');} else while(0)
#define DBG_FILEOPS2(lvl, s1, s2) if(DBG_FILEOPS_LEVEL>=lvl) { Serial.print('['); Serial.print(F(s1)); Serial.print(s2); Serial.println(']');} else while(0)
#define DBG_FILEOPS3(lvl, s1, s2, s3) if(DBG_FILEOPS_LEVEL>=lvl) { Serial.print('['); Serial.print(F(s1)); Serial.print(s2); Serial.print(s3); Serial.println(']');} else while(0)
#define DBG_FILEOPS4(lvl, s1, s2, s3, s4) if(DBG_FILEOPS_LEVEL>=lvl) { Serial.print('['); Serial.print(F(s1)); Serial.print(s2); Serial.print(s3); Serial.print(s4); Serial.println(']');} else while(0)
#else
#define DBG_FILEOPS(lvl, s) while(0)
#define DBG_FILEOPS2(lvl, s1, s2) while(0)
#define DBG_FILEOPS3(lvl, s1, s2, s3) while(0)
#define DBG_FILEOPS4(lvl, s1, s2, s3, s4) while(0)
#endif


void serial_setup();

void serial_replay_start(byte device, bool example, byte filenum);
void serial_capture_start(byte device, byte filenum);
bool serial_replay_running(byte device);
bool serial_capture_running(byte device);
void serial_stop(byte devoce);

bool serial_acr_mount_ps2();
bool serial_acr_check_cload_timeout();
void serial_close_files();

void serial_timer_interrupt_setup(byte dev = 0xff);
void serial_receive_host_data(byte host_interface, byte b);
bool serial_available();
int  serial_read();
void serial_reset(byte dev = 0xff);
byte serial_last_active_primary_device();

byte serial_2sio1_in_ctrl();
byte serial_2sio1_in_data();
void serial_2sio1_out_ctrl(byte data);
void serial_2sio1_out_data(byte data);

byte serial_2sio2_in_ctrl();
byte serial_2sio2_in_data();
void serial_2sio2_out_ctrl(byte data);
void serial_2sio2_out_data(byte data);

byte serial_2sio3_in_ctrl();
byte serial_2sio3_in_data();
void serial_2sio3_out_ctrl(byte data);
void serial_2sio3_out_data(byte data);

byte serial_2sio4_in_ctrl();
byte serial_2sio4_in_data();
void serial_2sio4_out_ctrl(byte data);
void serial_2sio4_out_data(byte data);

byte serial_sio_in_ctrl();
byte serial_sio_in_data();
void serial_sio_out_ctrl(byte data);
void serial_sio_out_data(byte data);

byte serial_acr_in_ctrl();
byte serial_acr_in_data();
void serial_acr_out_ctrl(byte data);
void serial_acr_out_data(byte data);

void serial_update_hlda_led();

#endif
