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

void serial_replay_start(byte device, byte filenum);
void serial_capture_start(byte device, byte filenum);
bool serial_acr_mount_ps2();
bool serial_acr_check_cload_timeout();
bool serial_file_open();
void serial_close_files();

void serial_timer_interrupt_setup(byte dev = 0xff);
void serial_receive_host_data(byte host_interface, byte b);
bool serial_available();
int  serial_read();
void serial_reset();

byte serial_2sio1_in_ctrl();
byte serial_2sio1_in_data();
void serial_2sio1_out_ctrl(byte data);
void serial_2sio1_out_data(byte data);

byte serial_2sio2_in_ctrl();
byte serial_2sio2_in_data();
void serial_2sio2_out_ctrl(byte data);
void serial_2sio2_out_data(byte data);

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
