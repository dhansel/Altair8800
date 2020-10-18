/*
 * This file is part of uLibx.
 *
 * Copyright (C) 2020  D.Herrendoerfer
 *
 *   uLibx is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   uLibx is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with uLibx.  If not, see <http://www.gnu.org/licenses/>.
 *
*/

//callbacks to host_teensy36.h 
extern void host_vga_write(uint8_t c);
extern void host_vga_setpos(uint8_t x, uint8_t y);
extern void host_vga_clear();

// Buffers
static uint8_t esc_input_buffer[14]; 
static uint8_t buffer_index=0;

// Positions and properties
uint8_t esc_active = 0;

//Helpers:
static boolean uVtIsAlpha(char c) 
{
  return ((c<='z')&&(c>='a'))||((c<='Z')&&(c>='A'));
}

static boolean uVtIsNum(char c)
{
  return (c<='9')&&(c>='0');
}

static short uVtNum(char *c) 
{
  short i=0,out=0;
  for (i=0;uVtIsNum(c[i]);i++)
  {
    out*=10;
    out+=c[i]-'0';
  }
  return out;
}

static short uVtNum2(char *c) 
{
  short i=0;
  while (c[i]!=';') i++;
  i++;
  return uVtNum(&c[i]);
}

static char uVtToUpper(char c)
{
  if ((c<='z')&&(c>='a'))
    return c-'a'+'A';
  return c;
}

//Error Handler
static void err_send_buffer()
{
  for (uint8_t i=0; i<buffer_index;i++)
    host_vga_write(esc_input_buffer[i]);
}


//Escape Code Handler
static uint8_t do_esc()
{
   uint8_t code = esc_input_buffer[buffer_index-1];

  //Note: Simplified ... An escape sequence is always, ESC, TypeCode, Maybe a number and an Alphabetical ESC code. 
  if (buffer_index < 3)
    return 1; // too little info yet.

  if (esc_input_buffer[1] == '[') {
    //CSI
    if (uVtIsAlpha(code)){
      //Found an end code
      switch(code) {
        case 'H': {
                  uint8_t y = uVtNum((char*)&esc_input_buffer[2]);
                  uint8_t x = uVtNum2((char*)&esc_input_buffer[2]);
                  host_vga_setpos(x, y);
                  return 0;
                  break;
        }
        case 'J': {
                  uint8_t x = uVtNum((char*)&esc_input_buffer[2]);
                  host_vga_clear();
                  return 0;
                  break;
        }
      }
    }
  }  

  return 1;
}

//The VT-write
uint8_t uVt_write(uint8_t c)
{    
  if (esc_active) {
    esc_input_buffer[buffer_index++] = c;
    esc_input_buffer[buffer_index] = 0;
    if (buffer_index > 12)  {
      //too big!
      err_send_buffer();
      buffer_index = 0;
      esc_active = 0;
      return 0;
    }
    else {
      if (!do_esc()) {
        // done, esc handled 
        buffer_index = 0;
        esc_active = 0;
      }
    }
    return 0;
  }

  if (c == 0x1B) {
    esc_input_buffer[buffer_index++] = c;
    esc_active = 1;
    return 0;
  }

  //UVGA already handles tabs and stuff
  host_vga_write(c);

}
