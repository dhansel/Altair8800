#ifndef SERIAL_H
#define SERIAL_H

#include "Arduino.h"

#define DBG_FILEOPS_LEVEL 2

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


void serial_replay_start(byte device, byte filenum);
void serial_capture_start(byte device, byte filenum);
bool serial_tape_mount_ps2();
bool serial_tape_check_timeout();
bool serial_file_open();
void serial_close_files();

void serial_timer_interrupt();
void serial_receive_data(byte b);
bool serial_available();
int  serial_read();
void serial_reset();

byte serial_2sio_in_ctrl();
byte serial_2sio_in_data();
void serial_2sio_out_ctrl(byte data);
void serial_2sio_out_data(byte data);

byte serial_sio_in_ctrl();
byte serial_sio_in_data();
void serial_sio_out_ctrl(byte data);
void serial_sio_out_data(byte data);

byte serial_tape_in_ctrl();
byte serial_tape_in_data();
void serial_tape_out_ctrl(byte data);
void serial_tape_out_data(byte data);

void serial_update_hlda_led();

#endif
