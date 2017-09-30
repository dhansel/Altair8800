// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef IMAGE_H
#define IMAGE_H

#include <Arduino.h>

#define IMAGE_FLOPPY 0
#define IMAGE_HDSK   1

const char *image_get_dir_content(byte image_type);
bool        image_get_filename(byte image_type, byte image_num, char *filename, int buf_len, bool check_exist = true);
const char *image_get_filename(byte image_type, byte image_num, bool check_exist = true);
const char *image_get_description(byte image_type, byte image_num);

#endif

