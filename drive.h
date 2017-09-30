// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>

void drive_setup();
void drive_dir();
const char *drive_get_image_filename(byte image_num, bool check_exist = true);
const char *drive_get_image_description(byte image_num);
bool drive_mount(byte drive_num, byte image_num);
bool drive_unmount(byte drive_num);
byte drive_get_mounted_image(byte drive_num);
void drive_reset();
void drive_set_realtime(bool b);
byte drive_in(byte addr);
void drive_out(byte addr, byte data);

#endif
