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


// must keep track of cursor position here since uvga library has
// no function to query cursor position and current cursor position 
// is needed for some VT100 codes
static const int num_cols = 360/8, num_rows = 300/8;
static int cursor_x = 0, cursor_y = 0;

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
      case 'D': {
                  uint8_t n = uVtNum((char*)&esc_input_buffer[2]);
                  cursor_x -= n;
                  if( cursor_x<0 ) cursor_x = 0;
                  host_vga_setpos(cursor_x, cursor_y);
                  return 0;
                  break;
                  }
        case 'H': {
                  // rows/columns are one-based but 0,0 can also be used for left upper corner
                  // see for example: https://vt100.net/docs/vt100-ug/chapter3.html (CUP command)
                  cursor_y = max(uVtNum((char*)&esc_input_buffer[2])-1, 0);
                  cursor_x = max(uVtNum2((char*)&esc_input_buffer[2])-1, 0);
                  host_vga_setpos(cursor_x, cursor_y);
                  return 0;
                  break;
        }
        case 'J': {
                  uint8_t x = uVtNum((char*)&esc_input_buffer[2]);
                  host_vga_clear();
                  cursor_x = 0;
                  cursor_y = 0;
                  return 0;
                  break;
        }
      case 'K': {
                  uint8_t x = uVtNum((char*)&esc_input_buffer[2]);
                  for(int i=cursor_x; i<num_cols; i++) host_vga_write(' ');
                  host_vga_setpos(cursor_x, cursor_y);
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

  // Only let text run over into next line if simulation is running. 
  // If simulation is not running then we're in the configuration menu
  // which is much more readable if we just cut off lines.
  if( !(status_led_local & ST_WAIT) || c=='\r' || c=='\n' || c=='\b' || cursor_x<num_cols-1 )
    {
      //UVGA already handles tabs and stuff
      host_vga_write(c);
    }
  else
    return 0;
  
  // update cursor position
  switch( c )
    {
    case '\r': cursor_x = 0; break;
    case '\b': cursor_x = max(cursor_x-1, 0); break;
    case '\n': cursor_x = 0; cursor_y = min(cursor_y+1, num_rows-1); break;
    default: 
      {
        cursor_x++; 
        if( cursor_x>=num_cols ) { cursor_x = 0; cursor_y = min(cursor_y+1, num_rows-1); }
        break;
      }
    }
}
