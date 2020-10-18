// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2019 David Hansel
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

#include "tdrive.h"
#include "config.h"
#include "host.h"
#include "cpucore.h"
#include "Altair8800.h"
#include "timer.h"
#include "image.h"
#include "io.h"

#if NUM_TDRIVES == 0

void tdrive_setup() {}
void tdrive_dir() {}
const char *tdrive_get_image_filename(byte image_num, bool check_exist) { return NULL; }
const char *tdrive_get_image_description(byte image_num) { return NULL; }
bool tdrive_mount(byte drive_num, byte image_num) { return false; }
bool tdrive_unmount(byte drive_num) { return false; }
byte tdrive_get_mounted_image(byte drive_num) { return 0; }
void tdrive_reset() {}

#elif NUM_TDRIVES>4

#error Tarbell disk controller can only address 4 drives. Set NUM_TDRIVES<=4 in config.h

#elif !defined(HOST_HAS_FILESYS)

#error Disk drive emulation requires host filesystem

#else

#define DEBUGLVL 0

#define DRIVE_SECTOR_LENGTH        128
#define DRIVE_NUM_SECTORS           26
#define DRIVE_NUM_TRACKS            77

#define DRIVE_STATUS_NOTREADY       0x80
#define DRIVE_STATUS_T1_SEEKERR     0x10
#define DRIVE_STATUS_T2_RECNOTFOUND 0x10
#define DRIVE_STATUS_T1_TRACK0      0x04
#define DRIVE_STATUS_T2_DRQ         0x02
#define DRIVE_STATUS_BUSY           0x01


static byte drive_selected = 0;
static byte drive_mounted_disk[NUM_TDRIVES];
static byte drive_current_track[NUM_TDRIVES];
static HOST_FILESYS_FILE_TYPE drive_file[NUM_TDRIVES];
static byte drive_current_sector;

static bool drive_data_request;
static byte drive_track, drive_sector, drive_data, drive_status;
static byte drive_command, drive_aux;
static byte drive_data_idx, drive_data_count;
static byte drive_data_buffer[DRIVE_SECTOR_LENGTH];

static void tdrive_register_ports();


static uint32_t drive_get_file_pos(byte drive_num)
{
  return drive_current_track[drive_num] * DRIVE_NUM_SECTORS * DRIVE_SECTOR_LENGTH + (drive_current_sector-1) * DRIVE_SECTOR_LENGTH;
}


static void drive_read_sector(byte drive_num)
{
  if( drive_num < NUM_TDRIVES && drive_mounted_disk[drive_num]>0 )
    {
#if DEBUGLVL>=1
      printf("read drive %i track %i sector %i\n", drive_num, drive_current_track[drive_num], drive_current_sector);
#endif
      host_filesys_file_seek(drive_file[drive_num], drive_get_file_pos(drive_num));
      byte n = host_filesys_file_read(drive_file[drive_num], DRIVE_SECTOR_LENGTH, drive_data_buffer);
      if( n<DRIVE_SECTOR_LENGTH ) memset(drive_data_buffer+n, 0, DRIVE_SECTOR_LENGTH-n);
    }
}


static void drive_write_sector(byte drive_num)
{
  if( drive_data_idx>0 && drive_num < NUM_TDRIVES && drive_mounted_disk[drive_num]>0 )
    {
#if DEBUGLVL>=1
      printf("write drive %i track %i sector %i\n", drive_num, drive_current_track[drive_num], drive_current_sector);
#endif
      host_filesys_file_seek(drive_file[drive_num], drive_get_file_pos(drive_num));
      host_filesys_file_write(drive_file[drive_num], drive_data_idx, drive_data_buffer);
      host_filesys_file_flush(drive_file[drive_num]);
    }
}


void tdrive_reset()
{
  drive_selected = 0;
  for(byte i=0; i<NUM_DRIVES; i++)
    drive_current_track[i] = 0;

  drive_status = 0;
  drive_data_request = false;
  drive_sector = 0;
  drive_data = 0;
  drive_data_idx = 0;
  drive_command = 0;
  drive_current_sector = 0;
}


void tdrive_dir()
{
  Serial.print(image_get_dir_content(IMAGE_TARBELL));
}


bool tdrive_unmount(byte drive_num)
{
  if( drive_num<NUM_TDRIVES && drive_mounted_disk[drive_num]>0 )
    {
      if( (drive_command&0xE0)==0xA0 || (drive_command&0xF0)==0xF0 ) drive_write_sector(drive_selected);
      drive_mounted_disk[drive_num] = 0;
      host_filesys_file_close(drive_file[drive_num]);
      tdrive_register_ports();
    }

  return true;
}


byte tdrive_get_mounted_image(byte drive_num)
{
  return drive_mounted_disk[drive_num];
}


const char *tdrive_get_image_description(byte disk_num)
{
  return image_get_description(IMAGE_TARBELL, disk_num);
}


const char *tdrive_get_image_filename(byte image_num, bool check_exist)
{
  return image_get_filename(IMAGE_TARBELL, image_num, check_exist);
}


bool tdrive_mount(byte drive_num, byte image_num)
{
  if( drive_num<NUM_TDRIVES )
    {
      tdrive_unmount(drive_num);
      if( image_num>0 )
        {
          char filename[13];
          image_get_filename(IMAGE_TARBELL, image_num, filename, 13, false);
          drive_mounted_disk[drive_num] = image_num;
          drive_file[drive_num] = host_filesys_file_open(filename, true);
          tdrive_register_ports();
          return true;
        }
    }

  return false;
}


static void tdrive_out_data(byte data)
{
  drive_data = data;

  if( (drive_command & 0xE0) == 0xA0 )
    {
      // currently executing write sector command
      drive_data_buffer[drive_data_idx++] = drive_data;
          
      // check if we have received one sector of data
      if( drive_data_idx == DRIVE_SECTOR_LENGTH )
        {
          // write data out to file on host
          drive_write_sector(drive_selected);
              
          if( (drive_command & 0xF0) == 0xB0 && drive_sector<DRIVE_NUM_SECTORS )
            {
              // multi-sector write => wait for more data for next sector
              drive_sector++;
              drive_data_idx=0;
            }
          else
            {
              // done writing sector
              drive_data_request = false;
              drive_status = 0;
              drive_command = 0;
            }
        }
    }
  else if( (drive_command&0xF0) == 0xF0 )
    {
      // currently executing write track command
          
      drive_aux++;
      if( drive_current_sector==0 )
        {
          // lead-in before first sector is 73 bytes long => skip
          if( drive_aux==73 ) { drive_current_sector++; drive_aux = 0; }
        }
      else
        {
          // there are 30 extra bytes before and 28 after the actual data => skip
          const byte skipBefore = 30, skipAfter  = 28;
              
          // drive_data_idx must be consistent with number of data bytes received
          // in case the write track command gets canceled
          if( drive_aux>skipBefore && drive_data_idx<DRIVE_SECTOR_LENGTH )
            drive_data_buffer[drive_data_idx++] = drive_data;
              
          if( drive_aux == skipBefore+DRIVE_SECTOR_LENGTH+skipAfter )
            {
              // sector complete => write data out to host
              drive_data_idx = DRIVE_SECTOR_LENGTH;
              drive_write_sector(drive_selected);
              drive_aux = 0;
              drive_data_idx = 0;
              drive_current_sector++;
                  
              if( drive_current_sector>DRIVE_NUM_SECTORS )
                {
                  // done writing track
                  drive_current_sector = 1;
                  drive_data_request = false;
                  drive_status = 0;
                  drive_command = 0;
                }
            }
        }
    }
}


static void tdrive_out_command(byte cmd)
{
  if( (cmd&0xF0) == 0xD0 )
    {
      // force interrupt (type 4)
      if( drive_command!=0 )
        {
          // if we are currently in write mode we still need to write all received
          // data out to the disk file on the host
          if( (drive_command&0xE0)==0xA0 || (drive_command&0xF0)==0xF0 ) 
            drive_write_sector(drive_selected);
          
          // stop executing current command
          drive_command = 0;
          drive_data_request = false;
        }
      else
        {
          // if not executing a command then clear status
          drive_status = 0;
        }
    }
  else if( drive_command!=0 )
    { /* ignore new commands while busy */ }
  else if( (cmd&0xF0)==0x00 )
    {
      // restore/home (type 1)
      drive_track = 0;
      drive_status = DRIVE_STATUS_T1_TRACK0;
          
      if( drive_selected<NUM_TDRIVES && drive_mounted_disk[drive_selected]>0 )
        drive_current_track[drive_selected] = 0;
      else if( (cmd&0x04)!=0 )
        drive_status = DRIVE_STATUS_T1_SEEKERR;
    }
  else if( (cmd&0xF0)==0x10 )
    {
      // seek (type 1)
      drive_track = drive_data;
      if( drive_track==0 ) drive_status = DRIVE_STATUS_T1_TRACK0;

      if( drive_selected<NUM_TDRIVES && drive_mounted_disk[drive_selected]>0 )
        {
          if( drive_track < DRIVE_NUM_TRACKS )
            drive_current_track[drive_selected] = drive_track;
          else
            drive_current_track[drive_selected] = DRIVE_NUM_TRACKS-1;
        }
      else if( (cmd&0x04)!=0 )
        drive_status = DRIVE_STATUS_T1_SEEKERR;
    }
  else if( (cmd&0xE0)<0x80 )
    {
      // step in/out/again (type 1)
      static bool stepOut = false;
          
      if( (cmd&0xE0)==0x40 )
        stepOut = false;
      else if( (cmd&0xE0)==0x60 )
        stepOut = true;

      // step the drive mechanism if we have a valid drive
      if( drive_selected<NUM_TDRIVES && drive_mounted_disk[drive_selected]>0 )
        {
          if( stepOut && drive_current_track[drive_selected]>0 ) 
            drive_current_track[drive_selected]--;
          else if( !stepOut && drive_current_track[drive_selected]<DRIVE_NUM_TRACKS-1 )
            drive_current_track[drive_selected]++;
        }
          
      // update track register if requested
      if( cmd & 0x10 )
        {
          if( stepOut && drive_track>0 ) 
            drive_track--;
          else if( !stepOut && drive_track<DRIVE_NUM_TRACKS-1 )
            drive_track++;
        }
          
      // update status and track register if at track 0
      if( drive_selected<NUM_TDRIVES && drive_mounted_disk[drive_selected]>0 )
        {
          if( drive_current_track[drive_selected]==0 ) 
            {
              drive_track  = 0;
              drive_status = DRIVE_STATUS_T1_TRACK0;
            }
        }
      else if( (cmd&0x04)!=0 )
        drive_status = DRIVE_STATUS_T1_SEEKERR;
    }
  else if( (cmd&0xE0)==0x80 )
    {
      // read sector (type 2)
      if( drive_selected<NUM_TDRIVES && drive_mounted_disk[drive_selected]>0 )
        {
          drive_current_sector = drive_sector;
          drive_read_sector(drive_selected);
              
          drive_data_idx     = 0;
          drive_data_count   = DRIVE_SECTOR_LENGTH;
          drive_command      = cmd;
          drive_data_request = true;
          drive_status       = DRIVE_STATUS_T2_DRQ;
        }
      else
        drive_status = DRIVE_STATUS_T2_RECNOTFOUND;
    }
  else if( (cmd&0xE0)==0xA0 )
    {
      // write sector (type 2)
      if( drive_selected<NUM_TDRIVES && drive_mounted_disk[drive_selected]>0 )
        {
          drive_current_sector = drive_sector;
          drive_data_idx     = 0;
          drive_command      = cmd;
          drive_data_request = true;
          drive_status       = DRIVE_STATUS_T2_DRQ;
        }
      else
        drive_status = DRIVE_STATUS_T2_RECNOTFOUND;
    }
  else if( cmd == 0xC4 )
    {
      // read address (type 3)
      if( drive_selected<NUM_TDRIVES && drive_mounted_disk[drive_selected]>0 )
        {
          drive_current_sector++;
          if( drive_current_sector>DRIVE_NUM_SECTORS ) drive_current_sector = 1;
              
          drive_data_idx   = 0;
          drive_data_count = 6;
          drive_command    = cmd;
          drive_data_request = true;
          drive_data_buffer[0] = drive_current_track[drive_selected];
          drive_data_buffer[1] = 6;
          drive_data_buffer[2] = drive_current_sector;
          drive_data_buffer[3] = DRIVE_SECTOR_LENGTH;
          drive_data_buffer[4] = 0;
          drive_data_buffer[5] = 0;
        }
      else
        drive_status = DRIVE_STATUS_NOTREADY;
    }
  else if( cmd == 0xF4 )
    {
      // write track (type 3)
      if( drive_selected<NUM_TDRIVES && drive_mounted_disk[drive_selected]>0 )
        {
          drive_current_sector = 0;
          drive_data_idx       = 0;
          drive_aux            = 0;
          drive_command        = cmd;
          drive_data_request   = true;
          drive_status         = DRIVE_STATUS_T2_DRQ;
        }
      else
        drive_status = DRIVE_STATUS_NOTREADY;
    }
  else if( (cmd&0xFE) == 0xE4 )
    {
      // read track (type 3)
      if( drive_selected<NUM_TDRIVES && drive_mounted_disk[drive_selected]>0 )
        {
          // NOT IMPLEMENTED => do not produce data request and 
          // set "RECORD NOT FOUND" status flag to signal error condition
          drive_data_request = false;
          drive_status = DRIVE_STATUS_T2_RECNOTFOUND;
        }
      else
        drive_status = DRIVE_STATUS_NOTREADY;
    }
}


static byte tdrive_in_data()
{
  if( drive_data_idx < drive_data_count )
    {
      drive_data = drive_data_buffer[drive_data_idx++];
      
      // check whether all available data has been read
      if( drive_data_idx == drive_data_count )
        {
          if( (drive_command & 0xF0) == 0x90 && drive_sector<DRIVE_NUM_SECTORS )
            {
              // multi-sector read => read next sector
              drive_sector++;
              drive_read_sector(drive_selected);
              drive_data_idx=0;
            }
          else
            {
              drive_data_request = false;
              drive_status = 0;
              drive_command = 0;
            }
        }
    }
  
  return drive_data;
}


byte tdrive_in(byte addr)
{
  byte data = 0xff;

  switch( addr )
    {
    case 0xf8: // read status register
      data = drive_status;
      if( drive_command!=0 ) data |= DRIVE_STATUS_BUSY;
      break;

    case 0xf9: // read track register
      data = drive_track;
      break;

    case 0xfa: // read sector register
      data = drive_sector;
      break;

    case 0xfb: // read data register
      data = tdrive_in_data();
      break;

    case 0xfc: // wait for interrupt
      data = drive_data_request ? 0x80 : 0x00;
      break;
    }

#if DEBUGLVL>=2
  printf("%04x: tdrive_in(%02x) = %02x\n", regPC-1, addr, data);
#endif

  return data;
}


void tdrive_out(byte addr, byte data)
{
#if DEBUGLVL>=2
  printf("%04x: tdrive_out(%02x, %02x)\n", regPC-1, addr, data);
#endif

  switch( addr )
    {
    case 0xf8: // write command register (execute command)
      tdrive_out_command(data);
      break;

    case 0xf9: // write track register
      drive_track = data;
      break;

    case 0xfa: // write sector register
      drive_sector = data;
      break;

    case 0xfb: // write data register
      tdrive_out_data(data);
      break;

    case 0xfc: // write extended command register
      {
        // if bits 0-2 equal 010 then 
        // update current drive (bits 4-5)
        if( (data & 0x07) == 0x02 )
          drive_selected = (~data & 0x30) >> 4;
        break;
      }
    }
}


void tdrive_register_ports()
{
  bool drive_used = false;
  for(byte i=0; i<NUM_TDRIVES; i++)
    drive_used |= drive_mounted_disk[i]!=0;

  for(byte i=0xf8; i<=0xfd; i++)
    {
      io_register_port_inp(i, drive_used ? tdrive_in : NULL);
      io_register_port_out(i, drive_used ? tdrive_out : NULL);
    }
}


void tdrive_setup()
{
  for(byte i=0; i<NUM_TDRIVES; i++)
    drive_mounted_disk[i] = 0;

  tdrive_register_ports();
  tdrive_reset();
}

#endif
