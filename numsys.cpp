// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software Foundation,
// Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
// -----------------------------------------------------------------------------

#include <Arduino.h>
#include "numsys.h"
#include "mem.h"
#include "serial.h"

static byte numsys = NUMSYS_HEX;

static byte hexToDec(int hc)
{
  if( hc>96 ) hc -= 32;

  if( hc >= 65 && hc <= 70 )
    return hc - 65 + 10;
  else if( hc >= 48 && hc <= 57 )
    return hc - 48;

  return 255;
}


void numsys_toggle()
{
  numsys = (numsys+1) % 3;
}

void numsys_set(byte sys)
{
  numsys = sys;
}


void numsys_print_byte_bin(byte b)
{
  for(byte i=0; i<8; i++)
    {
      Serial.print(b & 0x80 ? '1' : '0');
      b = b * 2;
    }
}

void numsys_print_byte(byte b)
{
  if( numsys==NUMSYS_HEX )
    {
      if( b<16 ) Serial.print('0');
      Serial.print(b, HEX);
    }
  else if( numsys==NUMSYS_OCT )
    {
      byte d;
      d = (b&0700) >> 6;
      Serial.print(d);
      d = (b&0070) >> 3;
      Serial.print(d);
      d = (b&0007);
      Serial.print(d);
    }
  else
    Serial.print(b);
}


void numsys_print_word(uint16_t w)
{
  if( numsys==NUMSYS_HEX )
    {
      numsys_print_byte(w >> 8);
      numsys_print_byte(w & 0xff);
    }
  else if( numsys==NUMSYS_OCT )
    {
      Serial.print((w>>15) & 007);
      Serial.print((w>>12) & 007);
      Serial.print((w>> 9) & 007);
      Serial.print((w>> 6) & 007);
      Serial.print((w>> 3) & 007);
      Serial.print( w      & 007);
    }
  else
    {
      Serial.print(w);
    }
}


void numsys_print_mem(uint16_t addr, byte num, bool printBrackets)
{
  byte i;
  if( printBrackets ) Serial.print('['); 
  for(i=0; i<num; i++)
    { numsys_print_byte(MREAD(addr+i)); if(i+1<num) Serial.print(' '); }
  if( printBrackets ) Serial.print(']'); 
}


static byte numsys_read_hex_digit()
{
  while( true )
    {
      char c = serial_read();
      if( c>='0' && c<='9' )
        return c-'0';
      else if( c>='A' && c<='F' )
        return c-'A'+10;
      else if( c>='a' && c<='f' )
        return c-'a'+10;
    }
}


byte numsys_read_hex_byte()
{
  byte b;
  b  = numsys_read_hex_digit() * 16;
  b += numsys_read_hex_digit();
  return b;
}

uint16_t numsys_read_hex_word()
{
  uint16_t w;
  w  = (uint16_t) numsys_read_hex_byte() * 256;
  w += (uint16_t) numsys_read_hex_byte();
  return w;
}


uint16_t numsys_read_word(bool *ESC)
{
  byte b;
  uint16_t w = 0;
  int c = -1;

  if( ESC!=NULL ) *ESC = false;
  while( c!=13 && c!=10 && c!=32 && c!=9 && c!='-' && c!=':')
    {
      c=-1;
      while(c<0) c = serial_read();

      if( numsys==NUMSYS_HEX && (b=hexToDec(c))!=255 )
        {
          Serial.write(c);
          w = w << 4 | b;
        }
      else if( numsys==NUMSYS_OCT && c>=48 && c<=55 )
        {
          Serial.write(c);
          w = w << 3 | (c-48);
        }
      else if( c>=48 && c<=57 )
        {
          Serial.write(c);
          w = w * 10 + (c-48);
        }
      else if( c==27 && ESC!=NULL )
        {
          *ESC = true;
          return 0;
        }
    }

  return w;
}
