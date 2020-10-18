// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef TDRIVE_H
#define TDRIVE_H

#include <Arduino.h>

void tdrive_setup();
void tdrive_dir();
const char *tdrive_get_image_filename(byte image_num, bool check_exist = true);
const char *tdrive_get_image_description(byte image_num);
bool tdrive_mount(byte drive_num, byte image_num);
bool tdrive_unmount(byte drive_num);
byte tdrive_get_mounted_image(byte drive_num);
void tdrive_reset();

#endif
