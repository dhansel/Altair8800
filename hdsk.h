#ifndef HDSK_H
#define HDSK_H

#include <Arduino.h>

void hdsk_4pio_out(byte port, byte data);
byte hdsk_4pio_in(byte port);

const char *hdsk_get_image_filename(byte image_num, bool check_exist = true);
const char *hdsk_get_image_description(byte image_num);
bool hdsk_mount(byte unit_num, byte platter_num, byte image_num);
bool hdsk_unmount(byte unit_num, byte platter_num);
byte hdsk_get_mounted_image(byte unit_num, byte platter_num);
void hdsk_dir();
void hdsk_set_realtime(bool b);

void hdsk_reset();
void hdsk_setup();

#endif
