#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>

void drive_setup();
void drive_dir();
bool drive_mount(byte drive_num, byte disk_num);
bool drive_unmount(byte drive_num);
void drive_reset();
byte drive_in(byte addr);
void drive_out(byte addr, byte data);

#endif
