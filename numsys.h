// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef NUMSYS_H
#define NUMSYS_H

#define NUMSYS_OCT 0
#define NUMSYS_DEC 1
#define NUMSYS_HEX 2

void     numsys_set(byte sys);
byte     numsys_get();
byte     numsys_get_byte_length();
void     numsys_toggle();
void     numsys_print_word(uint16_t w);
void     numsys_print_byte(byte b);
void     numsys_print_byte_bin(byte b);
void     numsys_print_byte_oct(byte b);
void     numsys_print_byte_dec(byte b);
void     numsys_print_byte_hex(byte b);
void     numsys_print_mem(uint16_t addr, byte num, bool printBrackets);
bool     numsys_read_byte(byte *w);
bool     numsys_read_word(uint16_t *w);
bool     numsys_read_dword(uint32_t *w);
uint32_t numsys_read_dword(bool *ESC = NULL);
byte     numsys_read_hex_byte();
uint16_t numsys_read_hex_word();

#ifdef __AVR_ATmega2560__
// on Arduino Mega do not use String as it requires heap space
// and RAM is in short supply. This function is used exclusively for
// logging via Serial.print, so returning the number directly will still
// work, it just will always be printed as decimal
#define  numsys_byte2string(i) i
#else
String   numsys_byte2string(byte b);
#endif

#endif
