#ifndef PRINTER_H
#define PRINTER_H

#include <Arduino.h>

void printer_setup();

void printer_out_ctrl(byte data);
void printer_out_data(byte data);

byte printer_in_ctrl();
byte printer_in_data();

#endif
