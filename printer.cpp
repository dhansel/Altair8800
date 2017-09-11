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


// NOTE: I found almost no documentation on either the Okidata or C700
//       printer. All I have is some information on the Okidata taken
//       from the printer emulation of the MITS Altair Emulator.
//       The rest I had to guess based on issuing actual LPRINT/LLIST
//       commands in BASIC and CP/M and seeing what happens.
//       What I have here works fine for the cases I tried but is
//       certainly nowhere close to being accurate.


#include "printer.h"
#include "config.h"
#include "host.h"
#include "timer.h"
#include "cpucore.h"
#include "Altair8800.h"


#if USE_PRINTER==0

void printer_out_ctrl(byte data) {}
void printer_out_data(byte data) {}
byte printer_in_ctrl() { return 0xff; }
byte printer_in_data() { return 0xff; }
void printer_setup() {}

#else

#define	PST_BUFFER_FULL	0x01
#define	PST_PRINTING	0x02
#define	PST_LINEFEED	0x08

#define STLF_CR   3
#define STLF_CRLF 2
#define STLF_LF   1
#define STLF_DONE 0

static byte status = 0x00;
static bool interrupt_enabled = false;
static byte buffer_size = 0, buffer_counter = 0, linefeed_status = 0;
static byte buffer[80];


static bool print_character(byte c, unsigned long delay)
{
  bool res = false;

  byte ser = config_printer_map_to_host_serial();
  if( ser>0 && host_serial_available_for_write(ser-1) )
    {
      // printer is assigned to serial device and device is ready
      host_serial_write(ser-1, c);
      timer_start(TIMER_PRINTER, delay);
      res = true;
    }
  else
    {
      // serial output not assigned or not ready => check again in 10ms
      timer_start(TIMER_PRINTER, 1000000/100);
    }
  
  return res;
}


static void print_next_character()
{
  status &= ~(PST_PRINTING|PST_LINEFEED);
  bool rt = config_printer_realtime() || interrupt_enabled;

  if( buffer_counter<buffer_size )
    {
      status |= PST_PRINTING;

      // 1000000us/100 delay => 100 characters per second
      if( print_character(buffer[buffer_counter], rt ? 1000000/100 : 1) )
        {
          buffer_counter++;
          if( buffer_counter == buffer_size )
            {
              // done printing the buffer
              buffer_size=0;
              status &= ~PST_BUFFER_FULL;
            }
        }
    }
  else if( linefeed_status==STLF_CRLF )
    {
      status |= PST_LINEFEED;

      // 1000000us/2 delay => 1/10 second for carriage return
      if( print_character(0x0d, rt ? 1000000/10 : 1) )
        linefeed_status=STLF_LF;
    }
  else if( linefeed_status==STLF_CR )
    {
      status |= PST_LINEFEED;

      // 1000000us/2 delay => 1/10 second for carriage return
      if( print_character(0x0d, rt ? 1000000/10 : 1) )
        linefeed_status=STLF_DONE;
    }
  else if( linefeed_status==STLF_LF )
    {
      status |= PST_LINEFEED;

      // 1000000us/10 delay => 1/10 second for line feed
      if( print_character(0x0a, rt ? 1000000/10 : 1) )
        linefeed_status=STLF_DONE;
    }
}


static void printer_interrupt()
{
  if( (status & (PST_PRINTING|PST_LINEFEED)) )
    print_next_character();

  if( !(status & (PST_PRINTING|PST_LINEFEED)) && (status & PST_BUFFER_FULL) )
    {
      buffer_counter  = 0;
      linefeed_status = STLF_CRLF;
      print_next_character();
    }

  if( interrupt_enabled && !(status & (PST_PRINTING|PST_LINEFEED)) )
    altair_interrupt(INT_LPC);
}


// ----- Okidata printer


void printer_oki_out_ctrl(byte data)
{
  if( interrupt_enabled ) 
    altair_interrupt(INT_LPC, false);

  switch( data )
    {
    case 0x01:
      {
        // command 0x01: print whatever is in the buffer (if buffer is not empty)
        if( (status & (PST_PRINTING|PST_LINEFEED))==0 && buffer_size>0 )
          {
            buffer_counter  = 0;
            linefeed_status = STLF_CRLF;
            print_next_character();
          }
        break;
      }

    case 0x02:
      {
        // command 0x02: do a line feed
        if( (status & (PST_PRINTING|PST_LINEFEED))==0 )
          {
            linefeed_status = STLF_LF;
            print_next_character();
          }
        break;
      }

    case 0x04:
      {
        // command 0x04: clear buffer
        buffer_size=0;
        break;
      }
    }
}


void printer_oki_out_data(byte data)
{
  if( interrupt_enabled ) 
    altair_interrupt(INT_LPC, false);

  if( data==0x0d )
    {
      buffer_counter  = 0;
      linefeed_status = STLF_CR;
      print_next_character();
    }
  else if( data==0x0a )
    {
      linefeed_status = STLF_LF;
      print_next_character();
    }
  else if( (status & PST_PRINTING)==0 && buffer_size<80 )
    {
      buffer[buffer_size++] = data;
      if( buffer_size==80 )
        {
          status |= PST_BUFFER_FULL;
          if( (status & PST_LINEFEED)==0 ) 
            {
              buffer_counter  = 0;
              linefeed_status = STLF_CRLF;
              print_next_character();
            }
        }
    }
}


byte printer_oki_in_ctrl()
{
  // inverted logic for status register
  return ~status;
}


void printer_oki_interrupt()
{
  printer_interrupt();
}


void printer_oki_setup()
{
  timer_setup(TIMER_PRINTER, 1000000, printer_oki_interrupt);
  interrupt_enabled = false;
  status = 0x00;
}


// ----- C700 printer


void printer_c700_out_ctrl(byte data)
{
  if( interrupt_enabled ) 
    altair_interrupt(INT_LPC, false);

  switch( data )
    {
    case 0x03:
      {
        // command 0x03: enable interrupts (?)
        interrupt_enabled = true;
        if( !timer_running(TIMER_PRINTER) )
          timer_start(TIMER_PRINTER, 10);

        break;
      }
    }
}


void printer_c700_out_data(byte data)
{
  if( interrupt_enabled ) 
    altair_interrupt(INT_LPC, false);

  if( data==0x0d )
    {
      buffer_counter  = 0;
      linefeed_status = STLF_CRLF;
      print_next_character();
    }
  else if( data==0x0a )
    {
      linefeed_status = STLF_LF;
      print_next_character();
    }
  else if( data==0x11 )
    {
      // 0x11 is DC1 (device control 1)
      // function for C700 is unknown
    }
  else if( (status & PST_PRINTING)==0 && buffer_size<80 )
    {
      buffer[buffer_size++] = data;
      if( buffer_size==80 )
        {
          status |= PST_BUFFER_FULL;
          if( (status & PST_LINEFEED)==0 ) 
            {
              buffer_counter  = 0;
              linefeed_status = STLF_CRLF;
              print_next_character();
            }
        }
    }
}


byte printer_c700_in_ctrl()
{
  // I have no documentation of the C700 printer
  // so I don't really know what the status register is
  // supposed to return when reading.
  // Returning 0xFF when the printer is available and 0x00 when
  // the printer is busy works with BASIC
  return (status==0x00 ? 0xFF : 0x00);
}


void printer_c700_interrupt()
{
  printer_interrupt();
}


void printer_c700_setup()
{
  timer_setup(TIMER_PRINTER, 1000000, printer_c700_interrupt);
  status = 0x00;
}


// ---------------------------------------------------------------------------------------------


void printer_out_ctrl(byte data)
{
  //printf("%04X: OUT 02, %02X\n", regPC, data);

  switch( config_printer_type() )
    {
    case CP_OKI:  printer_oki_out_ctrl(data); break;
    case CP_C700: printer_c700_out_ctrl(data); break;
    }
}


void printer_out_data(byte data)
{
  //printf("%04X: OUT 03, %02X\n", regPC, data);

  switch( config_printer_type() )
    {
    case CP_OKI:  printer_oki_out_data(data); break;
    case CP_C700: printer_c700_out_data(data); break;
    }
}


byte printer_in_ctrl()
{
  byte data = 0xff;

  switch( config_printer_type() )
    {
    case CP_OKI:  data = printer_oki_in_ctrl(); break;
    case CP_C700: data = printer_c700_in_ctrl(); break;
    }

  //printf("%04X: IN 02 <= %02X\n", regPC, data);
  return data;
}


byte printer_in_data()
{
  // can't read data from printer
  return 0xff;
}


void printer_setup()
{
  if( interrupt_enabled ) 
    altair_interrupt(INT_LPC, false);
  interrupt_enabled = false;

  switch( config_printer_type() )
    {
    case CP_OKI:  return printer_oki_setup(); break;
    case CP_C700: return printer_c700_setup(); break;
    }
}


#endif
