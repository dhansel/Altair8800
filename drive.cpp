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

#include "drive.h"
#include "config.h"
#include "host.h"
#include "cpucore.h"
#include "Altair8800.h"
#include "timer.h"

#if NUM_DRIVES == 0

void drive_setup() {}
void drive_dir() {}
const char *drive_disk_filename(byte disk_num, bool check_exist) { return NULL; }
const char *drive_disk_description(byte disk_num) { return NULL; }
bool drive_mount(byte drive_num, byte disk_num) { return false; }
bool drive_unmount(byte drive_num) { return false; }
byte drive_get_mounted_disk(byte drive_num) { return 0; }
void drive_reset() {}
void drive_set_realtime(bool b) {}
byte drive_in(byte addr) { return 0; }
void drive_out(byte addr, byte data) {}

#else

#define DRIVE_SECTOR_LENGTH    137
#define DRIVE_NUM_SECTORS       32
#define DRIVE_NUM_SECTORS_MD    16
#define DRIVE_NUM_TRACKS        77
#define DRIVE_NUM_TRACKS_MD     35

#define DRIVE_STATUS_HAVEDISK    1
#define DRIVE_STATUS_HEADLOAD    2
#define DRIVE_STATUS_WRITE       4
#define DRIVE_STATUS_INT_EN      8
#define DRIVE_STATUS_REALTIME   16

static byte drive_selected = 0xff;
static char drive_file_name[NUM_DRIVES][13];
static byte drive_mounted_disk[NUM_DRIVES];
static byte drive_status[NUM_DRIVES];
static byte drive_current_track[NUM_DRIVES];
static byte drive_current_sector[NUM_DRIVES];
static byte drive_current_byte[NUM_DRIVES];
static byte drive_sector_buffer[NUM_DRIVES][DRIVE_SECTOR_LENGTH];
static byte drive_num_sectors[NUM_DRIVES];
static byte drive_num_tracks[NUM_DRIVES];

#define DRIVE_SECTOR_TRUE_DELAY       5200
#define DRIVE_SECTOR_NOT_TRUE_DELAY     30
#define DRIVE_HEAD_STEP_DELAY        40000
static bool drive_sector_true = false;

static const char *get_dir_content()
{
  const char *dirfilename = "DISKDIR.TXT";
  static char *content = NULL;

  if( content==NULL )
    {
      int dirlen = host_get_file_size(dirfilename);
      if( dirlen>0 )
        {
          content = (char *) malloc(dirlen+1);
          if( content!=NULL )
            {
              if( host_read_file(dirfilename, 0, dirlen, content) )
                content[dirlen] = 0;
              else
                {
                  free(content);
                  content = (char *) "";
                }
            }
          else
            {
              free(content);
              content = (char *) "";
            }
        }
      else
        content = (char *) "";
    }

  return content;
}

static uint32_t drive_get_file_pos(byte drive_num)
{
  return drive_current_track[drive_num] * drive_num_sectors[drive_num] * DRIVE_SECTOR_LENGTH + drive_current_sector[drive_num] * DRIVE_SECTOR_LENGTH;
}


static void drive_flush(byte drive_num)
{
  if( (drive_status[drive_num] & DRIVE_STATUS_WRITE) && drive_current_byte[drive_num]>0 )
    {
      drive_sector_true = false;

      //Serial.print(F("Writing disk: ")); Serial.println(drive_get_file_pos(drive_selected));
      host_write_file(drive_file_name[drive_num], drive_get_file_pos(drive_num), drive_current_byte[drive_num], drive_sector_buffer[drive_num]);
      drive_status[drive_num] &= ~DRIVE_STATUS_WRITE;
      drive_current_byte[drive_num] = 0xff;
    }
}


static void drive_sector_interrupt()
{
  uint32_t d;

  if( drive_sector_true )
    {
      // sector was true => reset to not true
      drive_sector_true = false;
      d = DRIVE_SECTOR_TRUE_DELAY;
    }
  else
    {
      // flush write buffer and end write mode (if enabled)
      if( drive_status[drive_selected] & DRIVE_STATUS_WRITE )
        drive_flush(drive_selected);

      // advance current sector
      drive_current_sector[drive_selected]++;
      if( drive_current_sector[drive_selected] >= drive_num_sectors[drive_selected] )
        drive_current_sector[drive_selected] = 0;
      
      drive_current_byte[drive_selected] = 0xff;
      drive_sector_true = true;
      d = DRIVE_SECTOR_NOT_TRUE_DELAY;
    }
  
  // update Altair interrupt line
  if( drive_status[drive_selected] & DRIVE_STATUS_INT_EN )
    altair_interrupt(INT_DRIVE, drive_sector_true);

  // start timer again with new delay
  timer_start(TIMER_DRIVE, d);
}


void drive_setup()
{
  drive_selected = 0xff;
  for(byte i=0; i<NUM_DRIVES; i++)
    {
      drive_status[i] = 0;
      drive_current_track[i] = 0;
      drive_current_sector[i] = 0;
      drive_current_byte[i] = 0;
      drive_mounted_disk[i] = 0;
      drive_num_tracks[i] = DRIVE_NUM_TRACKS;
      drive_num_sectors[i] = DRIVE_NUM_SECTORS;
    }
  
  // prepare sector change timer interrupt 
  timer_setup(TIMER_DRIVE, DRIVE_SECTOR_TRUE_DELAY, drive_sector_interrupt);
}


void drive_dir()
{
  Serial.print(get_dir_content());
}


bool drive_unmount(byte drive_num)
{
  if( drive_num<NUM_DRIVES && (drive_status[drive_num] & DRIVE_STATUS_HAVEDISK) )
    {
      drive_flush(drive_num);
      drive_status[drive_num] &= DRIVE_STATUS_REALTIME;
      drive_mounted_disk[drive_num] = 0;
      altair_interrupt(INT_DRIVE, false);
    }

  return true;
}


byte drive_get_mounted_disk(byte drive_num)
{
  return drive_mounted_disk[drive_num];
}

static const char *get_filename(byte disk_num, char *filename, int buf_len, bool check_exist)
{
  snprintf(filename, buf_len, "DISK%02X.DSK", disk_num);
  return !check_exist || host_file_exists(filename) ? filename : NULL;
}


const char *drive_disk_description(byte disk_num)
{
  static char *buf = NULL;
  const char *fname = drive_disk_filename(disk_num);

  if( fname!=NULL )
    {
      const char *c = strstr(get_dir_content(), fname);
      if( c )
        {
          if( buf ) free(buf);
          buf = (char *) malloc(strlen(c)+1);
          if( buf )
            {
              char *b = buf;
              while(*c != 13 && *c!=10 && *c!=0) *b++ = *c++;
              *b = 0;
              return buf;
            }
          else
            return fname;
        }
      else
        return fname;
    }
  else
    return fname;
}


const char *drive_disk_filename(byte disk_num, bool check_exist)
{
  static char buf[13];
  return get_filename(disk_num, buf, 13, check_exist);
}


bool drive_mount(byte drive_num, byte disk_num)
{
  if( drive_num<NUM_DRIVES )
    {
      if( drive_status[drive_num] & DRIVE_STATUS_HAVEDISK ) drive_unmount(drive_num);
      if( disk_num>0 )
        {
          get_filename(disk_num, drive_file_name[drive_num], 13, false);
          drive_mounted_disk[drive_num] = disk_num;
          drive_status[drive_num] |= DRIVE_STATUS_HAVEDISK;

          int32_t s = host_get_file_size(drive_file_name[drive_num]);
          if( s>0 && s<100000 )
            {
              // minidisk
              drive_num_tracks[drive_num] = DRIVE_NUM_TRACKS_MD;
              drive_num_sectors[drive_num] = DRIVE_NUM_SECTORS_MD;
            }
          else
            {
              // regular disk or empty disk
              drive_num_tracks[drive_num] = DRIVE_NUM_TRACKS;
              drive_num_sectors[drive_num] = DRIVE_NUM_SECTORS;
            }

          return true;
        }
    }

  return false;
}


void drive_set_realtime(bool b)
{
  if( b && drive_selected<0xff && !(drive_status[drive_selected] & DRIVE_STATUS_INT_EN) )
    {
      // drive interrupts were not enabled before => start timer
      timer_start(TIMER_DRIVE, DRIVE_SECTOR_TRUE_DELAY);
    }
  else if( !b && drive_selected<0xff && !(drive_status[drive_selected] & DRIVE_STATUS_INT_EN) )
    {
      // drive has no interrupts enabled => stop timer
      timer_stop(TIMER_DRIVE);
    }

  for(byte i=0; i<NUM_DRIVES; i++)
    if( b )
      drive_status[i] |=  DRIVE_STATUS_REALTIME;
    else
      drive_status[i] &= ~DRIVE_STATUS_REALTIME;
}


void drive_reset()
{
  for(byte i=0; i<NUM_DRIVES; i++)
    drive_unmount(i);
}


byte drive_in(byte addr)
{
  byte data = 0;

  switch( addr )
    {
    case 0010: 
      {
        /* read drive status
           +---+---+---+---+---+---+---+---+
           | R | Z | I | X | X | H | M | W |
           +---+---+---+---+---+---+---+---+
           W - When 0, write circuit ready to write another byte.
           M - When 0, head movement is allowed
           H - When 0, indicates head is loaded for read/write
           I - When 0, indicates interrupts enabled
           Z - When 0, indicates head is on track 0
           R - When 0, indicates that read circuit has new byte to read
        */

        data = 0;
        if( drive_selected<NUM_DRIVES && (drive_status[drive_selected] & DRIVE_STATUS_HAVEDISK) )
          {
            data |= 0x02; // always ready to move head

            if( drive_status[drive_selected] & DRIVE_STATUS_HEADLOAD )
              {
                data |= 0x04; // head is loaded
                if( drive_status[drive_selected] & DRIVE_STATUS_WRITE )
                  data |= 0x01; // ready to write
                else
                  data |= 0x80; // ready to read

                if( drive_status[drive_selected] & DRIVE_STATUS_INT_EN )
                  data |= 0x20; // interrupts enabled
              }

            if( drive_current_track[drive_selected]==0 )
              data |= 0x40; // on track 0

            // unused bits are 0 if drive is ready
            data |= 0x18;
          }

        data = ~data; // negative logic
        break;
      }

    case 0011: 
      {
        /* read current sector position
           +---+---+---+---+---+---+---+---+
           | X | X |  Sector Number    | T |
           +---+---+---+---+---+---+---+---+
           T = Sector True, 0 if the sector is positioned to read or write.
        */

        if( drive_selected<NUM_DRIVES )
          {
            if( !(drive_status[drive_selected] & (DRIVE_STATUS_INT_EN|DRIVE_STATUS_REALTIME)) )
              {
                // if interrupts are not enabled, alternate "sector true" bit
                // every time this register is queried and advance current sector
                // every time "sector true" goes to true (some applications need
                // "sector true" to go false, e.g. MBASIC under CP/M when saving)
                if( drive_sector_true )
                  {
                    drive_sector_true = false;
                    data |= 0x01;
                  }
                else
                  {
                    // if we were writing then flush the write buffer now
                    if( drive_status[drive_selected]&DRIVE_STATUS_WRITE )
                      drive_flush(drive_selected);

                    drive_current_sector[drive_selected]++;
                    if( drive_current_sector[drive_selected] >= drive_num_sectors[drive_selected] )
                      drive_current_sector[drive_selected] = 0;
                    drive_current_byte[drive_selected] = 0xff;
                    drive_sector_true = true;
                  }
              }
            else if( !drive_sector_true )
              {
                // if interrupts are enabled then the drive_sector_true flag and
                // the drive_current_sector setting are handled in the interrupt
                // routine.
                data |= 0x01;
              }
            
            if( (drive_status[drive_selected] & DRIVE_STATUS_HAVEDISK) && (drive_status[drive_selected] & DRIVE_STATUS_HEADLOAD) )
              data |= 0xC0 | (drive_current_sector[drive_selected] * 2);
          }

        break;
      }
    
    case 0012: 
      {
        // read data from disk
        if( drive_selected<NUM_DRIVES && !(drive_status[drive_selected] & DRIVE_STATUS_WRITE) )
          {
            if( drive_current_byte[drive_selected] >= DRIVE_SECTOR_LENGTH )
              {
                // read new sector from file
                //Serial.print(F("Reading disk: ")); Serial.println(drive_get_file_pos(drive_selected));
                byte n = host_read_file(drive_file_name[drive_selected], drive_get_file_pos(drive_selected), 
                                        DRIVE_SECTOR_LENGTH, drive_sector_buffer[drive_selected]);
                if( n<DRIVE_SECTOR_LENGTH ) memset(drive_sector_buffer[drive_selected]+n, 0, DRIVE_SECTOR_LENGTH-n);
                drive_current_byte[drive_selected] = 0;
              }
            
            data = drive_sector_buffer[drive_selected][drive_current_byte[drive_selected]++];
          }

        break;
      }
    }

  //printf("reading disk %04x: %02x -> %02x\n", regPC, addr, data);
  return data;
}



void drive_out(byte addr, byte data)
{
  //printf("writing disk %04x: %02x -> %02x\n", regPC, addr, data);

  // we were writing and now are doing something else then flush write buffer
  if( addr!=0012 && drive_selected<NUM_DRIVES && (drive_status[drive_selected]&DRIVE_STATUS_WRITE) )
    drive_flush(drive_selected);

  switch( addr )
    {
    case 0010: 
      {
        /* write disk drive select register
           +---+---+---+---+---+---+---+---+
           | C | X | X | X |   Device      |
           +---+---+---+---+---+---+---+---+
           C = If this bit is 1, the disk controller selected by 'device'
               is cleared.  If the bit is zero, 'device' is selected as the
               device being controlled by subsequent I/O operations.
        */
        byte drive = data & 0x0f;

        if( drive < NUM_DRIVES )
          {
            // stop timer interrupts
            timer_stop(TIMER_DRIVE);

            if( data & 0x80 )
              {
                drive_flush(drive);
                drive_status[drive] &= (DRIVE_STATUS_HAVEDISK|DRIVE_STATUS_HEADLOAD|DRIVE_STATUS_REALTIME);
                drive_current_byte[drive] = 0xff;
              }
            else
              drive_selected = drive;

            // start timer interrupts if interrupts for the selected drive are enabled
            if( drive_status[drive_selected]&(DRIVE_STATUS_INT_EN|DRIVE_STATUS_REALTIME) )
              timer_start(TIMER_DRIVE, DRIVE_SECTOR_TRUE_DELAY);
          }

        break;
      }

    case 0011: 
      {
        /* write disk drive control register:
           +---+---+---+---+---+---+---+---+
           | W | C | D | E | U | H | O | I |
           +---+---+---+---+---+---+---+---+
           I - When 1, steps head IN one track
           O - When 1, steps head OUT out track
           H - When 1, loads head to drive surface
           U - When 1, unloads head
           E - When 1, Enables interrupts
           D - When 1, Disables interrupts
           C - When 1, lowers head current (ignored)
           W - When 1, starts Write Enable sequence 
        */

        if( drive_selected < NUM_DRIVES )
          {
            if( (data & 0x01) && drive_current_track[drive_selected]<drive_num_tracks[drive_selected]-1 ) 
              {
                drive_current_track[drive_selected]++;
                drive_current_byte[drive_selected] = 0xff;
                if( timer_running(TIMER_DRIVE) )
                  {
                    drive_sector_true = false;
                    timer_start(TIMER_DRIVE, DRIVE_HEAD_STEP_DELAY);
                  }
              }
            
            if( (data & 0x02) && drive_current_track[drive_selected]>0 ) 
              {
                drive_current_track[drive_selected]--;
                drive_current_byte[drive_selected] = 0xff;
                if( timer_running(TIMER_DRIVE) )
                  {
                    drive_sector_true = false;
                    timer_start(TIMER_DRIVE, DRIVE_HEAD_STEP_DELAY);
                  }
              }
            
            if( data & 0x04 ) drive_status[drive_selected] |=  DRIVE_STATUS_HEADLOAD;
            if( data & 0x08 ) drive_status[drive_selected] &= ~DRIVE_STATUS_HEADLOAD;
            if( data & 0x10 ) drive_status[drive_selected] |=  DRIVE_STATUS_INT_EN;
            if( data & 0x20 ) drive_status[drive_selected] &= ~DRIVE_STATUS_INT_EN;
            
            if( data & 0x80 ) 
              {
                drive_status[drive_selected] |=  DRIVE_STATUS_WRITE;
                drive_current_byte[drive_selected] = 0;
              }

            if( !timer_running(TIMER_DRIVE) && (drive_status[drive_selected]&(DRIVE_STATUS_INT_EN|DRIVE_STATUS_REALTIME)) )
              timer_start(TIMER_DRIVE, DRIVE_SECTOR_TRUE_DELAY);
            else if( timer_running(TIMER_DRIVE) && !(drive_status[drive_selected]&(DRIVE_STATUS_INT_EN|DRIVE_STATUS_REALTIME)) )
              timer_stop(TIMER_DRIVE); altair_interrupt(INT_DRIVE, false); 
          }

        break;
      }

    case 0012: 
      {
        if( drive_selected < NUM_DRIVES )
          {
            // write data to disk drive
            if( drive_status[drive_selected] & DRIVE_STATUS_WRITE )
              {
                if( drive_current_byte[drive_selected] < DRIVE_SECTOR_LENGTH )
                  drive_sector_buffer[drive_selected][drive_current_byte[drive_selected]++] = data;
                else
                  drive_flush(drive_selected);
              }
          }
        
        break;
      }
    }
}


#endif
