// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef CDRIVE_H
#define CDRIVE_H

#include <Arduino.h>

void cdrive_setup();
void cdrive_dir();
const char *cdrive_get_image_filename(byte image_num, bool check_exist = true);
const char *cdrive_get_image_description(byte image_num);
bool cdrive_mount(byte drive_num, byte image_num);
bool cdrive_unmount(byte drive_num);
byte cdrive_get_mounted_image(byte drive_num);
void cdrive_register_ports();
void cdrive_reset();

#define CDRIVE_SWITCH_ROM_ENABLE             0x01
#define CDRIVE_SWITCH_ROM_DISABLE_AFTER_BOOT 0x02
#define CDRIVE_SWITCH_AUTOBOOT               0x04
#define CDRIVE_SWITCH_INHIBIT_INIT           0x08
void cdrive_set_switches(byte switches);
byte cdrive_get_switches();

#endif
