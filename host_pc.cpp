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

#if defined(_WIN32) || defined(__linux__)

#include <Arduino.h>
#include <time.h>
#include <string>
#include "Altair8800.h"
#include "mem.h"
#include "serial.h"
#include "cpucore.h"
#include "host_pc.h"
#include "profile.h"
#include "timer.h"

#ifdef _WIN32
#include <Windows.h>
#else
#include <signal.h>
#endif

byte data_leds;
uint16_t status_leds;
uint16_t addr_leds;
byte stop_request;
static FILE *storagefile = NULL;

//#define DEBUG


bool host_read_function_switch(byte i)
{
  return false;
}


bool host_read_function_switch_debounced(byte i)
{
  return false;
}


bool host_read_function_switch_edge(int i)
{
  return false;
}


uint16_t host_read_function_switches_edge()
{
  return 0;
}


void host_reset_function_switch_state()
{
}

// ----------------------------------------------------------------------------------


void host_write_data(const void *data, uint32_t addr, uint32_t len)
{
#ifdef DEBUG
  printf("Writing %i bytes to   0x%04x: ", len, addr);
  for(int i=0; i<len; i++) printf("%02x ", ((byte *) data)[i]);
  printf("\n");
#endif

  if( storagefile )
    {
      fseek(storagefile, addr, SEEK_SET);
      fwrite(data, len, 1, storagefile);
      fflush(storagefile);
    }
}


void host_read_data(void *data, uint32_t addr, uint32_t len)
{
  if( storagefile )
    {
      fseek(storagefile, addr, SEEK_SET);
      uint32_t l = fread(data, 1, len, storagefile);
      if( l<len ) memset(((byte *) data)+l, 0, len-l);
    }

#ifdef DEBUG
  printf("Reading %i bytes from 0x%04x: ", len, addr);
  for(int i=0; i<len; i++) printf("%02x ", ((byte *) data)[i]);
  printf("\n");
#endif
}


void host_move_data(uint32_t to, uint32_t from, uint32_t len)
{
  void *buf = malloc(len);
  host_read_data(buf, from, len);
  host_write_data(buf, to, len);
  free(buf);
}


void host_copy_flash_to_ram(void *dst, const void *src, uint32_t len)
{
  memcpy(dst, src, len);
}


// ----------------------------------------------------------------------------------


static FILE *open_file(const char *filename)
{
  char fnamebuf[30];
  snprintf(fnamebuf, 30, "disks/%s", filename);
  return fopen(fnamebuf, "rb");
}


bool host_file_exists(const char *filename)
{
  FILE *f = open_file(filename);
  if( f )
    {
      fclose(f);
      return true;
    }

  return false;
}


int host_get_file_size(const char *filename)
{
  int res = -1;

  FILE *f = open_file(filename);
  if( f )
    {
      if( fseek(f, 0, SEEK_END)==0 )
        res = ftell(f);
      fclose(f);
    }
  
  return res;
}


uint32_t host_read_file(const char *filename, uint32_t offset, uint32_t len, void *buffer)
{
  uint32_t res = 0;
  FILE *f = open_file(filename);
  if( f )
    {
      if( fseek(f, offset, SEEK_SET)==0 )
        res = fread(buffer, 1, len, f);
      fclose(f);
    }

  //printf("host_read_file('%s', %04x, %04x, %p)=%04x\n", filename, offset, len, buffer, res);
  return res;
}


uint32_t host_write_file(const char *filename, uint32_t offset, uint32_t len, void *buffer)
{
  char fnamebuf[30];
  snprintf(fnamebuf, 30, "disks/%s", filename);
  uint32_t res = 0;
  FILE *f = fopen(fnamebuf, "r+b");
  if( !f ) f = fopen(fnamebuf, "w+b");

  if( f )
    {
      if( fseek(f, offset, SEEK_SET)==0 )
        res = fwrite(buffer, 1, len, f);
      fclose(f);
    }

  //printf("host_write_file('%s', %04x, %04x, %p)=%04x\n", filename, offset, len, buffer, res);
  return res;
}


// ----------------------------------------------------------------------------------

static int ctrlC = 0;

void sig_handler(int signum)
{
  ctrlC++;
}


uint32_t host_get_random()
{
  return rand()*65536l | rand();
}


static uint32_t cycles_per_char = 1, prev_char_cycles = 0;
void host_check_interrupts()
{
  static byte ctr = 0;
  static unsigned long prevCtrlC = 0;

  // Serial.available() is slow, only check it every 256 calls 
  // if program is running
  if( ((host_read_status_led_WAIT() || ++ctr==0) && Serial.available()) || ctrlC>0 )
    {
      int c = -1;

      if( ctrlC>0 )
	{ c = 3; ctrlC--; }
      else if( (host_read_status_led_WAIT() || (timer_get_cycles()-prev_char_cycles) >= cycles_per_char) )
	{
	  c = Serial.read();
	  prev_char_cycles = timer_get_cycles();
	}

      if( c == 3 )
        {
          // CTRL-C was pressed.  If we receive two CTRL-C in short order
          // then we terminate the emulator.
          if( millis()>prevCtrlC+250 )
            prevCtrlC = millis();
          else
            exit(0);
        }
      
      if( c>=0 ) serial_receive_host_data(0, c);
      ctr=0;
    }
}


void host_serial_setup(byte iface, unsigned long baud, bool set_primary_interface)
{
  // assuming 9 bits per character
  if( iface==0 )
    {
      cycles_per_char  = (9*2000000)/baud;
      prev_char_cycles = timer_get_cycles();
    }
}


void host_setup()
{
  data_leds = 0;
  status_leds = 0;
  addr_leds = 0;
  stop_request = 0;
  
  // open storage data file for mini file system
  storagefile = fopen("AltairStorage.dat", "r+b");
  if( storagefile==NULL ) 
    {
      void *chunk = calloc(1024, 1);
      storagefile = fopen("AltairStorage.dat", "wb");
      if( storagefile!=NULL )
        {
          uint32_t size;
          for( size = 0; (size+1024) < HOST_STORAGESIZE; size+=1024 )
            fwrite(chunk, 1024, 1, storagefile);
          fwrite(chunk, HOST_STORAGESIZE-size, 1, storagefile);
          fclose(storagefile);
        }
      
      storagefile = fopen("AltairStorage.dat", "r+b");
    }

#ifdef _WIN32
  // send CTRL-C to input instead of processing it (otherwise the
  // emulator would immediately quit if CTRL-C is pressed) and we
  // could not use CTRL-C to stop a running BASIC example.
  // CTRL-C is handled in host_check_interrupts (above) such that 
  // pressing it twice within 250ms will cause the emulator to terminate.
  DWORD mode;
  HANDLE hstdin = GetStdHandle(STD_INPUT_HANDLE);
  GetConsoleMode(hstdin, &mode);
  SetConsoleMode(hstdin, mode & ~ENABLE_PROCESSED_INPUT);
#else
  signal(SIGINT, sig_handler);
#endif

  // initialize random number generator
  srand((unsigned int) time(NULL));
}


#endif
