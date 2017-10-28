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
static byte buffer[132];


static bool print_character(byte c, unsigned long delay)
{
  bool res = false;

  byte ser = config_printer_map_to_host_serial();
  if( host_serial_available_for_write(ser) )
    {
      // printer is assigned to serial device and device is ready
      host_serial_write(ser, c);
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


// NOTE: I found very little documentation on the Okidata printer. 
//       All I have is some information taken from the printer emulation of 
//       the MITS Altair Emulator.
//       The rest I had to guess based on issuing actual LPRINT/LLIST
//       commands in BASIC and CP/M and seeing what happens.
//       What I have here works fine for the cases I tried but is
//       certainly nowhere close to being accurate.

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

// Documentation for the C700 printer is at:
// http://altairclone.com/downloads/manuals/88-C700%20(Centronics).pdf

static byte printer_c700_selected = false;

void printer_c700_out_ctrl(byte data)
{
  if( interrupt_enabled ) 
    altair_interrupt(INT_LPC, false);

  // bit 0: 0=PRIME (reset print buffer)
  if( (data & 0x01)==0 )
    {
      buffer_counter = 0;
      buffer_size = 0;
    }

  // bit 1: 1=enable interrupts
  interrupt_enabled = (data & 0x02)!=0;
  if( interrupt_enabled )
    {
      if( !timer_running(TIMER_PRINTER) )
        timer_start(TIMER_PRINTER, 10);
    }
  else
    {
      if( timer_running(TIMER_PRINTER) )
        timer_stop(TIMER_PRINTER);
    }
}


void printer_c700_out_data(byte data)
{
  if( data == 0x11 )
    {
      // select printer
      printer_c700_selected = true;
    }
  else if( printer_c700_selected )
    {
      if( data==0x0a )
        {
          // line feed
          linefeed_status = STLF_LF;
          print_next_character();
        }
      else if( data==0x0d )
        {
          // carriage return
          buffer_counter  = 0;
          linefeed_status = STLF_CRLF;
          print_next_character();
        }
      else if( data==0x0e )
        {
          // SO code (wide character printing)
          // currently not implemented
        }
      else if( data == 0x13 )
        {
          // de-select printer
          printer_c700_selected = false;
        }
      else if( data==0x7f )
        {
          // DEL (clears buffer)
          buffer_counter = 0;
          buffer_size = 0;
        }
      else if( (status & PST_PRINTING)==0 && buffer_size<132 )
        {
          buffer[buffer_size++] = data;
          if( buffer_size==132 )
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
}


byte printer_c700_in_ctrl()
{
  byte res = 0x00;

  // bit 0: 0=not ready, 1=ready
  // bit 1: 0=not busy,  1=busy
  if( status & (PST_PRINTING|PST_LINEFEED) ) 
    res |= 0x02;
  else
    res |= 0x01;

  // bit 2: 1=out of paper (we always have paper)

  // bit 3: 0=printer selected, 1=printer not selected
  if( !printer_c700_selected ) res |= 0x08;

  // bit 4: 1=fault (we never have a fault)

  // bit 6: interrupts enabled
  if( interrupt_enabled ) res |= 0x40;

  // bit 7: 1=interrupt has occurred
  if( altair_interrupt_active(INT_LPC) ) res |= 0x80;

  return res;
}


void printer_c700_interrupt()
{
  printer_interrupt();
}


void printer_c700_setup()
{
  timer_setup(TIMER_PRINTER, 1000000, printer_c700_interrupt);
  status = 0x00;
  printer_c700_selected = false;
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
