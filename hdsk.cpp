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

#include "hdsk.h"
#include "config.h"
#include "host.h"
#include "cpucore.h"
#include "Altair8800.h"
#include "timer.h"
#include "image.h"

#define DEBUGLVL 0

#if NUM_HDSK_UNITS == 0

void hdsk_4pio_out(byte port, byte data) {}
byte hdsk_4pio_in(byte port) { return 0xff; }
void hdsk_reset() {}
void hdsk_setup() {}

const char *hdsk_get_image_filename(byte image_num, bool check_exist) { return NULL; }
const char *hdsk_get_image_description(byte image_num) { return NULL; }
bool hdsk_mount(byte unit_num, byte platter_num, byte image_num) { return false; }
bool hdsk_unmount(byte unit_num, byte platter_num) { return false; }
byte hdsk_get_mounted_image(byte unit_num, byte platter_num) { return 0; }
void hdsk_dir() {}
void hdsk_set_realtime(bool b) {}

#else

static byte pio_data[4];
static byte pio_control[4];

#if DEBUGLVL >= 3
const char *signames[8] = { "CREADY", "CSTAT", "ACSTA", "ACMD", "CDSTA", "CDATA", "ADSTA", "ADATA" };
#endif


// ------------------------------------------------------------------------------------
// functions that mimic the control (strobe) signal wiring between 4PIO and controller


// lets the controller know that CSTAT has been read (CA2 of port 1)
static void hdsk_CSTAT_strobe();

// lets the controller know that ACMD was written (CB2 of port 1)
static void hdsk_ACMD_strobe();

// lets the controller know that CDATA has been read (CA2 of port 2)
static void hdsk_CDATA_strobe();

// lets the controller know that ADATA was written (CB2 of port 2)
static void hdsk_ADATA_strobe();


static void hdsk_4pio_CRDY_set(bool set = true)
{
  // called by the controller, set CA1 of 4PIO port 1 (CRDY, bit 7 of CREADY)
  // (controller ready)
  if( set ) 
    {
      pio_control[0] |= 0x80;

      // request interrupt if interrupts are enabled
      if( (pio_control[0] & 1)!=0 ) altair_interrupt(INT_HDSK);
    }
  else 
    pio_control[0] &= 0x7f;
}

static void hdsk_4pio_CMDACK_set(bool set = true)
{
  // called by the controller, set CB1 of 4PIO port 1 (CMDACK, bit 7 of ACSTA)
  // (command acknowledge)
  if( set ) pio_control[1] |= 0x80; else pio_control[1] &= 0x7f;
}

static void hdsk_4pio_CDA_set(bool set = true)
{
  // called by the controller, set CA1 of 4PIO port 2 (CDA, bit 7 of CDSTA)
  // (controller has put data on bus)
  if( set ) pio_control[2] |= 0x80; else pio_control[2] &= 0x7f;
}

static void hdsk_4pio_ADPA_set(bool set = true)
{
  // called by the controller, set CB1 of 4PIO port 2 (ADPA, bit 7 of ADSTA)
  // (controller ready to receive data)
  if( set ) pio_control[3] |= 0x80; else pio_control[3] &= 0x7f;
}


// ----------------------------------------------------------------------------------------


void hdsk_4pio_out(byte addr, byte data) 
{
  if( addr<0240 || addr>0247 )
    return; // invalid register, shouldn't get here

#if DEBUGLVL >= 3
  printf("writing hdsk (%04x) %02x (%s): %02x\n", regPC, addr, signames[addr-0240], data);
#endif
  byte port = (addr-0240) / 2;

  if( (addr & 1)==0 )
    {
      // writing to 4PIO control register
      pio_control[port] = (pio_control[port] & ~0x3f) | (data & 0x3f);
    }
  else if( (pio_control[port] & 4)==0 )
    {
      // writing to 4PIO data direction register (ignored, data directions
      // are hard-wired in the code)
    }
  else
    {
      pio_data[port] = data; 

      // writing to PIO data register strobes connected HDSK signals
      switch( port )
        {
        case 1: hdsk_ACMD_strobe();  break; // PIO port 1 section B: ACMD
        case 3: hdsk_ADATA_strobe(); break; // PIO port 2 section B: ADATA
        }
    }
}


byte hdsk_4pio_in(byte addr) 
{ 
  byte data = 0xff;

  if( addr<0240 || addr>0247 )
    return 0xff; // invalid register, shouldn't get here

  byte port = ((addr & ~1) - 0240) / 2;

  if( (addr & 1)==0 )
    {
      // reading PIO control register
      data = pio_control[port];
    }
  else
    {
      // read data register
      data = pio_data[port];

      // reading the data register clears bit 7 of the control register
      pio_control[port] &= 0x7f;

      // reading the data register of port 0 (CSTAT) clears the interrupt line
      if( port==0 && (pio_control[0] & 1)!=0 ) altair_interrupt(INT_HDSK, false);

      // reading PIO data register strobes connected HDSK signals
      switch( port )
        {
        case 0: hdsk_CSTAT_strobe(); break; // PIO port 1 section A: CSTAT
        case 2: hdsk_CDATA_strobe(); break; // PIO port 2 section A: CDATA
        }
    }
  
#if DEBUGLVL >= 4
  printf("reading hdsk (%04x) %02x (%s): %02x\n", regPC, addr, signames[addr-0240], data);
#endif

  // While the IN instruction in general takes 10 cycles, peripherals can request wait
  // states from the CPU by pulling the READY line low. According to the 88-4PIO manual
  // an IN instruction from the 4PIO uses one wait state.
  TIMER_ADD_CYCLES(1);

  return data; 
}


// -------------------------------------------------------------------------------------------------

static bool hdsk_realtime;
static byte hdsk_buffer_num;
static byte hdsk_buffer_ptr;
static byte hdsk_buffer_ctr;
static byte hdsk_ivbyte_num;
static byte hdsk_unit, hdsk_head, hdsk_sect, hdsk_current_sect;
static byte hdsk_ivbyte_B, hdsk_ivbyte_C, hdsk_ivbyte_E, hdsk_ivbyte_I;
static word hdsk_cyl[4], hdsk_seek;
static byte hdsk_buffer[4][256];
static byte hdsk_mounted_image[NUM_HDSK_UNITS][4];
static char hdsk_file_name[NUM_HDSK_UNITS][4][13];
static uint32_t hdsk_current_sect_cycles;

#define NUM_TRACKS           406
#define NUM_SECTORS           24
#define NUM_BYTES_PER_SECTOR 256
#define NUM_SURFACES           4

// we don't store header data or a CRC checksum in the disk image 
// so error bits 2-6 don't make sense for us. However in the
// event of a read or write error on the hosts' file system we do
// report a "CRC error in sector read" or "CRC error in header read"
// (depending on which command is executed)
#define ERR_DRIVE_NOT_READY  0x01
#define ERR_ILLEGAL_SECTOR   0x02
#define ERR_CRC_SECTOR_READ  0x04
#define ERR_CRC_HEADER_READ  0x08
#define ERR_WRITE_PROTECT    0x80

#define ROT_PER_MINUTE    2400
#define US_PER_ROTATION   (1000000/(ROT_PER_MINUTE/60))
#define CYCLES_PER_SECTOR ((US_PER_ROTATION*2)/NUM_SECTORS)

// according to Martin Eberhard's ADEXER manual, formatting one side of a
// platter takes about 60 seconds. On the other hand, in the ADEXER code
// the timeout for the FORMAT command is 60 seconds. Hard to imagine that
// Martin would have cut the timing so close, so we'll shoot for 55 seconds
// for a format. => 55000000us/406 ~ 135467us per track
// but that includes 10ms seek time to next track, so we take ~125000 per 
// track pure format time.
#define US_PER_TRACK_FORMAT 125000

#define CMD_SEEK      0
#define CMD_WRITESECT 2
#define CMD_READSECT  3
#define CMD_WRITEBUF  4
#define CMD_READBUF   5
#define CMD_READIV    6
#define CMD_WRITEIV   8
#define CMD_READUNF   10
#define CMD_FORMAT    12
#define CMD_INIT      14
#define CMD_NONE      255


#define CSTAT  pio_data[0]
#define ACMD   pio_data[1]
#define CDATA  pio_data[2]
#define ADATA  pio_data[3]
#define CRDY_INTERRUPT ((pio_control[0] & 1)!=0)

static byte hdsk_current_cmd;
static bool hdsk_ACMD_strobed;

void hdsk_ivbyte_set(byte addr, byte value);
byte hdsk_ivbyte_read(byte addr);


static const char *hdsk_cmd_str(byte cmd)
{
  switch( cmd )
    {
    case CMD_NONE:      return "NONE";
    case CMD_SEEK:      return "SEEK";
    case CMD_WRITESECT: return "WRITESECT";
    case CMD_READSECT:  return "READSECT";
    case CMD_WRITEBUF:  return "WRITEBUF";
    case CMD_READBUF:   return "READBUF";
    case CMD_READIV:    return "READIV";
    case CMD_WRITEIV:   return "WRITEIV";
    case CMD_READUNF:   return "READ UNFORMATTED";
    case CMD_FORMAT:    return "FORMAT";
    case CMD_INIT:      return "INITIALIZE";
    default:            return "???";
    }
}


inline const char *mk_filename()
{
  return hdsk_file_name[hdsk_unit][hdsk_head/2];
}


inline uint32_t mk_offset() 
{
  return (hdsk_cyl[hdsk_unit] * 2 * NUM_SECTORS * NUM_BYTES_PER_SECTOR 
          + (hdsk_head&1) * NUM_SECTORS * NUM_BYTES_PER_SECTOR
          + hdsk_sect * NUM_BYTES_PER_SECTOR);
}


inline void hdsk_set_unit()
{
  hdsk_unit = (ACMD & 0x0C) >> 2;
}

inline void hdsk_set_buffer_num()
{
  hdsk_buffer_num = ACMD & 0x03;
}

inline void hdsk_set_sect()
{
  word s = ADATA & 0x1F;
  if( s < NUM_SECTORS )
    hdsk_sect = s;
  else
    CSTAT |= ERR_ILLEGAL_SECTOR;
}

inline void hdsk_set_head()
{
  hdsk_head = (ADATA & 0xE0) >> 5;
}

inline bool hdsk_drive_ready(byte unit, byte head = 0xff)
{
  return unit<NUM_HDSK_UNITS && (head==0xff || hdsk_mounted_image[unit][head/2]>0);
}


static byte hdsk_calc_current_sector()
{                                                               
  uint32_t tdiff = timer_get_cycles()-hdsk_current_sect_cycles;
  hdsk_current_sect = (hdsk_current_sect + (tdiff/CYCLES_PER_SECTOR)) % 24;
  hdsk_current_sect_cycles = timer_get_cycles() - (tdiff%CYCLES_PER_SECTOR);
  return hdsk_current_sect;
}


static uint32_t hdsk_calc_time_to_sector(byte s)
{
  byte sect = hdsk_calc_current_sector();

  if( hdsk_sect>sect )
    sect = hdsk_sect-sect;
  else
    sect = (24-sect)+hdsk_sect;

  // hdsk_current_sect_cycles was the time when we were at the start of
  // the current sector
  return (sect * CYCLES_PER_SECTOR - (timer_get_cycles()-hdsk_current_sect_cycles))/2;
}


static uint32_t hdsk_calc_seek_time(word cyl)
{
  // calculate number of tracks to move
  uint32_t d = abs(int(cyl)-int(hdsk_cyl[hdsk_unit]));

  // this is calculated such that ADEXER reports the expected 
  // seek times corresponding to the HDSK manual:
  // minimum (adjacent track): 10ms, maximum (full stroke): 65ms
  // the 540us is a controller latency for the SEEK command that ADEXER
  // subtracts from the measured time
  return 540 + (d>0 ? 10000ul + ((d-1)*10000ul)/73ul : 0);
}


void hdsk_CSTAT_strobe()
{
  // called when Altair reads CSTAT - not used by controller
}


void check_read_only()
{
  // if we can't write but we can read then the disk image is write-protected
  // otherwise there is some other serious error
  byte dummy;
  if( host_read_file(mk_filename(), mk_offset(), 1, &dummy)>0 )
    CSTAT |= ERR_WRITE_PROTECT;
  else
    CSTAT |= ERR_CRC_HEADER_READ;
}


void hdsk_timer()
{
  switch( hdsk_current_cmd )
    {
    case CMD_SEEK:
      hdsk_cyl[hdsk_unit] = hdsk_seek;
      break;

    case CMD_FORMAT:
      {
        if( hdsk_seek < NUM_TRACKS )
          {
            // reached the next track
            if( hdsk_sect==0 )
              {
                // set the next track
                hdsk_cyl[hdsk_unit] = hdsk_seek;
                
                // seek to the beginning of the new track in the file
                host_set_file(NULL, mk_offset(), 0, 0, true);
              }

#if DEBUGLVL >= 2
            printf("Formatting sector %i head %i track %i of unit %i\n", 
                   hdsk_sect, hdsk_head, hdsk_cyl[hdsk_unit], hdsk_unit);
#endif

            // write two sectors at a time, keeping the file open
            uint32_t t = micros();
            if( host_set_file(NULL, 0xffffffff, 2*NUM_BYTES_PER_SECTOR, 0, true)<2*NUM_BYTES_PER_SECTOR )
              {
                // error => close the file
                host_set_file(NULL, 0xffffffff, 0, 0, false);

                // could not write
                check_read_only();
              }
            else if( hdsk_sect < NUM_SECTORS )
              {
                // wait for the time it would take the real HDSK drive to format
                // two sectors but subtract the time we have spent in host_set_file
                // That way we come out approximately taking the same real time as the 
                // original but also continue running the emulated program
                // (i.e. flicker the lights while waiting for the FORMAT command to finish)
                // This assumes that the emulator is running at 100% speed of the original,
                // which should be the most common case.
                t = micros()-t;
                hdsk_sect += 2;
                timer_start(TIMER_HDSK, t<(2*US_PER_TRACK_FORMAT/NUM_SECTORS) ? (2*US_PER_TRACK_FORMAT/NUM_SECTORS)-t : 1);

                // return from the function immediately
                return;
              }
            else
              {
                // seek to next track
                hdsk_sect = 0;
                hdsk_seek = hdsk_cyl[hdsk_unit] + 1;
                timer_start(TIMER_HDSK, 10000);

                // return from the function immediately
                return;
              }
          }
        else
          {
            // format is finished => close the file
            hdsk_seek = 405;
            host_set_file(NULL, 0xffffffff, 0, 0, false);
          }

        break;
      }
    }

  // ready for next command
  hdsk_current_cmd = CMD_NONE;
  hdsk_4pio_CRDY_set();

  // if ACMD has been strobed then there is a new command 
  // already waiting => execute that now
  if( hdsk_ACMD_strobed ) hdsk_ACMD_strobe();
}


void hdsk_ACMD_strobe()
{
  // called when Altair writes ACMD

  // latch strobe bit (according to HDSK IV byte A11 documentation this set
  // when ACMD is strobed by the Altair)
  hdsk_ACMD_strobed = true;

  // if we are still busy processing a command then don't do anything else
  if( hdsk_current_cmd != CMD_NONE ) return;

  // read command
  hdsk_current_cmd = ACMD / 16;

  // acknowledge command
  hdsk_4pio_CMDACK_set();

  // reset strobe bit (according to HDSK IV byte A11 documentation this reset 
  // when CMDACK is strobed by the controller)
  hdsk_ACMD_strobed = false;

  // reset error flags
  CSTAT = 0;

  switch( hdsk_current_cmd )
    {
    case CMD_SEEK:
      {
        hdsk_set_unit();
#if DEBUGLVL >= 1
        printf("Command: Seek unit %i to cylinder #%i\n", hdsk_unit, hdsk_cyl[hdsk_unit]);
#endif
        hdsk_seek = (ACMD & 0x01) * 256 + ADATA;

        if( !hdsk_drive_ready(hdsk_unit) )
          {
            CSTAT |= ERR_DRIVE_NOT_READY;
            hdsk_current_cmd = CMD_NONE;
            hdsk_4pio_CRDY_set(); // ready for next command
          }
        else if( hdsk_seek > NUM_TRACKS )
          hdsk_current_cmd = CMD_NONE; // illegal cylinder
        else if( hdsk_realtime || CRDY_INTERRUPT )
          timer_start(TIMER_HDSK, hdsk_calc_seek_time(hdsk_seek));
        else
          {
            hdsk_cyl[hdsk_unit] = hdsk_seek;
            hdsk_current_cmd = CMD_NONE;
            hdsk_4pio_CRDY_set(); // ready for next command
          }
        break;
      }

    case CMD_WRITEBUF: 
      {
        hdsk_set_buffer_num();
        hdsk_buffer_ctr = ADATA; 
        hdsk_buffer_ptr = 0;
        hdsk_4pio_ADPA_set(); // ready to receive data
#if DEBUGLVL >= 1
        printf("Command: Write %i bytes to buffer %i\n", hdsk_buffer_ctr==0 ? 256 : hdsk_buffer_ctr, hdsk_buffer_num);
#endif
        break;
      }
      
    case CMD_READBUF: 
      {
        hdsk_set_buffer_num();
        hdsk_buffer_ctr = ADATA; 
        hdsk_buffer_ptr = 0;
#if DEBUGLVL >= 1
        printf("Command: Read %i bytes from buffer %i\n", hdsk_buffer_ctr==0 ? 256 : hdsk_buffer_ctr, hdsk_buffer_num);
#endif
        hdsk_CDATA_strobe(); // read first byte
        break;
      }

    case CMD_READSECT:
    case CMD_READUNF:
      {
        // we don't store information about the header and don't
        // emulate error conditions on the platter, so "Read Unformatted"
        // is the same as "Read Sector"
        hdsk_set_unit();
        hdsk_set_buffer_num();
        hdsk_set_head();
        hdsk_set_sect();
#if DEBUGLVL >= 1
        printf("Command: Read head %i sector %i to buffer #%i\n", hdsk_head, hdsk_sect, hdsk_buffer_num);
#endif
        if( !hdsk_drive_ready(hdsk_unit, hdsk_head) )
          {
            CSTAT |= ERR_DRIVE_NOT_READY;
            hdsk_current_cmd = CMD_NONE;
            hdsk_4pio_CRDY_set(); // ready for next command
          }
        else if( host_read_file(mk_filename(), mk_offset(), NUM_BYTES_PER_SECTOR, hdsk_buffer[hdsk_buffer_num]) < NUM_BYTES_PER_SECTOR )
          {
            // error while reading
            CSTAT |= ERR_CRC_SECTOR_READ;
            hdsk_current_cmd = CMD_NONE;
            hdsk_4pio_CRDY_set(); // ready for next command
          }
        else if( hdsk_realtime || CRDY_INTERRUPT )
          timer_start(TIMER_HDSK, hdsk_calc_time_to_sector(hdsk_sect));
        else
          {
            hdsk_current_cmd = CMD_NONE;
            hdsk_4pio_CRDY_set(); // ready for next command
          }

        break;
      }

    case CMD_WRITESECT:
      {
        hdsk_set_buffer_num();
        hdsk_set_head();
        hdsk_set_sect();
        hdsk_set_unit();
#if DEBUGLVL >= 1
        printf("Command: Write buffer #%i to head %i sector %i\n", hdsk_buffer_num, hdsk_head, hdsk_sect);
#endif
        if( !hdsk_drive_ready(hdsk_unit, hdsk_head) )
          {
            CSTAT |= ERR_DRIVE_NOT_READY;
            hdsk_current_cmd = CMD_NONE;
            hdsk_4pio_CRDY_set(); // ready for next command
          }
        else if( host_write_file(mk_filename(), mk_offset(), NUM_BYTES_PER_SECTOR, hdsk_buffer[hdsk_buffer_num])<NUM_BYTES_PER_SECTOR )
          {
            // can't write
            check_read_only();
            hdsk_current_cmd = CMD_NONE;
            hdsk_4pio_CRDY_set(); // ready for next command
          }
        else if( hdsk_realtime || CRDY_INTERRUPT )
          timer_start(TIMER_HDSK, hdsk_calc_time_to_sector(hdsk_sect));
        else
          {
            hdsk_current_cmd = CMD_NONE;
            hdsk_4pio_CRDY_set(); // ready for next command
          }

        break;
      }

    case CMD_WRITEIV:
      {
        hdsk_ivbyte_num = ADATA;
        hdsk_4pio_ADPA_set(); // ready to receive data
#if DEBUGLVL >= 2
        printf("Command: Write IV byte %02X\n", hdsk_ivbyte_num);
#endif
        break;
      }
      
    case CMD_READIV:
      {
        hdsk_ivbyte_num = ADATA;
        hdsk_set_unit();
        hdsk_CDATA_strobe(); // read byte
        break;
      }

    case CMD_FORMAT:
      {
        hdsk_set_unit();
        hdsk_set_head();
#if DEBUGLVL >= 1
        printf("Command: FORMAT\n");
#endif
        // in a real 88-HDSK, FORMAT only writes the header information, not 
        // the sector data but we don't save header information at all, so in 
        // order to do something useful we just set all sector data to 0.

        if( hdsk_drive_ready(hdsk_unit, hdsk_head) )
          {
            // open the disk image file
            host_set_file(mk_filename(), 0, 0, 0, true);
            
            if( hdsk_realtime || CRDY_INTERRUPT )
              {
                // seek to track 0
                hdsk_sect = 0;
                hdsk_seek = 0;
                timer_start(TIMER_HDSK, hdsk_calc_seek_time(hdsk_seek));
              }
            else
              {
                // write sector data
                hdsk_sect = 0;
                for(hdsk_cyl[hdsk_unit]=0; hdsk_cyl[hdsk_unit]<NUM_TRACKS; hdsk_cyl[hdsk_unit]++)
                  if( host_set_file(NULL, mk_offset(), NUM_BYTES_PER_SECTOR*NUM_SECTORS, 0)<NUM_BYTES_PER_SECTOR*NUM_SECTORS )
                    {
                      // could not write
                      check_read_only();
                      break;
                    }
                
                host_set_file(NULL, 0xffffffff, 0, 0, false);
                
                hdsk_current_cmd = CMD_NONE;
                hdsk_4pio_CRDY_set(); // ready for next command
              }
          }
        else
          {
            CSTAT |= ERR_DRIVE_NOT_READY;
            hdsk_current_cmd = CMD_NONE;
            hdsk_4pio_CRDY_set(); // ready for next command
          }

        break;
      }

    case CMD_INIT:
      {
        // INIT initializes the controller and seeks unit 0 to track 0
        hdsk_unit = 0;
        hdsk_seek = 0;
        if( hdsk_realtime || CRDY_INTERRUPT )
          timer_start(TIMER_HDSK, hdsk_calc_seek_time(hdsk_seek));
        else
          {
            hdsk_cyl[hdsk_unit] = hdsk_seek;
            hdsk_current_cmd = CMD_NONE;
            hdsk_4pio_CRDY_set(); // ready for next command
          }
        break;
      }
      
    default:
      {
#if DEBUGLVL >= 1
        printf("Unknoen command: %s (%02X%02X)\n", hdsk_cmd_str(hdsk_current_cmd), ACMD, ADATA);
#endif
        hdsk_current_cmd = CMD_NONE;
        break;
      }
    }
}


void hdsk_CDATA_strobe()
{
  // called when Altair reads CDATA

  if( hdsk_current_cmd==CMD_READBUF )
    {
      CDATA = hdsk_buffer[hdsk_buffer_num][hdsk_buffer_ptr];
      hdsk_buffer_ptr++;
      if( --hdsk_buffer_ctr==0 ) 
        {
          hdsk_current_cmd = CMD_NONE;
          hdsk_4pio_CRDY_set(); // ready for next command
        }
      else
        hdsk_4pio_CDA_set(); // data ready for reading
    }
  else if( hdsk_current_cmd==CMD_READIV )
    {
      CDATA = hdsk_ivbyte_read(hdsk_ivbyte_num);
      hdsk_current_cmd = CMD_NONE;
      hdsk_4pio_CDA_set(); // data ready for reading
      hdsk_4pio_CRDY_set(); // ready for next command
    }
#if DEBUGLVL >= 2
  else if( hdsk_current_cmd != CMD_NONE )
    printf("hdsk_CDATA_strobe (%s)\n", hdsk_cmd_str(hdsk_current_cmd));
#endif
}


void hdsk_ADATA_strobe()
{
  // called after ALTAIR writes to ADATA
  hdsk_4pio_ADPA_set(false); // not ready to receive data

  if( hdsk_current_cmd==CMD_WRITEBUF )
    {
#if DEBUGLVL >= 3
      printf("buffer[%i][%02x] = %02x\n", hdsk_buffer_num, hdsk_buffer_ptr, ADATA);
#endif
      hdsk_buffer[hdsk_buffer_num][hdsk_buffer_ptr] = ADATA;
      hdsk_buffer_ptr++;
      if( --hdsk_buffer_ctr==0 ) 
        {
          hdsk_current_cmd = CMD_NONE;
          hdsk_4pio_CRDY_set(); // ready for next command
        }
      else
        hdsk_4pio_ADPA_set(); // ready to receive data
    }
  else if( hdsk_current_cmd==CMD_WRITEIV )
    {
      hdsk_ivbyte_set(hdsk_ivbyte_num, ADATA);
      hdsk_current_cmd = CMD_NONE;
      hdsk_4pio_CRDY_set(); // ready for next command
    }
#if DEBUGLVL >= 2
  else if( hdsk_current_cmd != CMD_NONE )
    printf("hdsk_ADATA_strobe (%s)\n", hdsk_cmd_str(hdsk_current_cmd));
#endif
}


void hdsk_ivbyte_set(byte addr, byte value)
{
#if DEBUGLVL >= 2
  printf("Set IV byte #%i to %02x\n", addr, value);
#endif

  switch( addr )
    {
    case 2: // A8 (controller error code)
      CSTAT = value; 
      break;

    case 18: // I (bit 0 = bit 8 for cylinder)
      hdsk_cyl[hdsk_unit] = (hdsk_cyl[hdsk_unit] & 0xff) | ((1-(value & 1)) * 256);
      hdsk_ivbyte_I = value;
      break;

    case 19: // J (cylinder bits 0-7)
      hdsk_cyl[hdsk_unit] = (hdsk_cyl[hdsk_unit] & 0x100) | ((~value)&0xFF);
      break;

    case 34: // B (only supported to pass ADEXER's "IV Address/Data Bus" test)
      hdsk_ivbyte_B = value;
      break;

    case 35: // C (only supported to pass ADEXER's "IV Address/Data Bus" test)
      hdsk_ivbyte_C = value;
      break;

    case 37: // E (only supported so ADEXER recognizes Disk Data Card as present)
      hdsk_ivbyte_E = value;
      break;
    }
}


byte hdsk_ivbyte_read(byte addr)
{
  byte value = 0xff;

  switch( addr )
    {
    case 2: // A8 (controller error code)
      value = CSTAT;
      break;

    case 3: // A9 (command upper byte)
      value = ACMD;
      break;

    case 5: // A11 (bit 1 = LOW if port 0xA3 has been written to by the computer)
      value = 0xFD | (hdsk_ACMD_strobed ? 0x00 : 0x02);
      break;

    case 6: // A12 data from controller)
      value = CDATA;
      break;

    case 7: // A13 (input - command low byte)
      value = ADATA;
      break;

    case 17: // H (bits 0-3 = unit)
      value = 0x0F - (1 << hdsk_unit);
      break;

    case 18: // I (bit 0 = bit 8 for cylinder)
      value = (hdsk_ivbyte_I & 0xFE) | (1-((hdsk_cyl[hdsk_unit] >> 8) & 1));
      break;

    case 19: // J (cylinder bits 0-7)
      value = ~(hdsk_cyl[hdsk_unit] & 0xff);
      break;

    case 20: // K (bit 7: drive malfunction, bit 4: extension present, bit 2: dual platter, bit 0: 200dpi)
      value = 0x10 | 0x04 | 0x01;
      break;

    case 21: // L (bits 0-3: unit 1-4 busy seeking, bit 3: illegal cylinder; bit 6: index pulse bit 7: Ready)
      value = 0x80;
      if( hdsk_seek>NUM_TRACKS ) value |= 0x10; 
      break;

    case 22: // M (bits 0-6: current sector, bit 7: sector pulse)
      value = hdsk_calc_current_sector();
      break;

    case 34: // B (only supported to pass ADEXER's "IV Address/Data Bus" test)
      value = hdsk_ivbyte_B;
      break;

    case 35: // C (only supported to pass ADEXER's "IV Address/Data Bus" test)
      value = hdsk_ivbyte_C;
      break;

    case 37: // E (only supported so ADEXER recognizes Disk Data Card as present)
      value = hdsk_ivbyte_E;
      break;
    }

#if DEBUGLVL >= 2
  printf("\nRead IV byte #%i for unit #%i: %02x", addr, hdsk_unit, value);
#endif
  return value;
}


void hdsk_reset()
{
  hdsk_current_cmd = CMD_NONE;
  hdsk_ACMD_strobed = false;
  hdsk_unit = 0;
  hdsk_head = 0;
  hdsk_sect = 0;

  for(int i=0; i<NUM_HDSK_UNITS; i++)
    hdsk_cyl[i] = 0;

  hdsk_4pio_CRDY_set();        // ready for next command
  hdsk_4pio_CMDACK_set(false); // reset command acknowledge
  hdsk_4pio_CDA_set(false);    // reset "data ready"
  hdsk_4pio_ADPA_set(false);   // reset "ready to receive"
}


void hdsk_setup()
{
  int i, j;

  hdsk_current_sect = 0;
  hdsk_current_sect_cycles = 0;

  for(i=0; i<4; i++)
    {
      pio_data[i]    = 0;
      pio_control[i] = 0;
    }

  for(i=0; i<NUM_HDSK_UNITS; i++)
    for(j=0; j<4; j++)
      {
        hdsk_mounted_image[i][j] = 0;
        hdsk_file_name[i][j][0] = 0;
      }

  hdsk_ivbyte_B = 0xff;
  hdsk_ivbyte_C = 0xff;
  hdsk_ivbyte_E = 0xff;
  hdsk_ivbyte_I = 0xff;

  hdsk_realtime = false;
  timer_setup(TIMER_HDSK, 0, hdsk_timer);
  hdsk_reset();
}


// --------------------------------------------------------------------------------------------------


const char *hdsk_get_image_filename(byte image_num, bool check_exist)
{
  return image_get_filename(IMAGE_HDSK, image_num, check_exist);
}


const char *hdsk_get_image_description(byte image_num)
{
  return image_get_description(IMAGE_HDSK, image_num);
}


bool hdsk_mount(byte unit_num, byte platter_num, byte image_num)
{
  if( unit_num < NUM_HDSK_UNITS && platter_num<4 )
    {
      hdsk_mounted_image[unit_num][platter_num] = image_num;
      image_get_filename(IMAGE_HDSK, image_num, hdsk_file_name[unit_num][platter_num], 13, false);
      return true;
    }
  else
    return false;
}


bool hdsk_unmount(byte unit_num, byte platter_num)
{
  if( unit_num < NUM_HDSK_UNITS && platter_num<4 )
    {
      hdsk_mounted_image[unit_num][platter_num] = 0;
      hdsk_file_name[unit_num][platter_num][0] = 0;
      return true;
    }
  else
    return false;
}


byte hdsk_get_mounted_image(byte unit_num, byte platter_num)
{
  if( unit_num < NUM_HDSK_UNITS && platter_num<4 )
    return hdsk_mounted_image[unit_num][platter_num];
  else
    return 0;
}


void hdsk_dir()
{
  Serial.print(image_get_dir_content(IMAGE_HDSK));
}


void hdsk_set_realtime(bool b)
{
  hdsk_realtime = b;
}


#endif
