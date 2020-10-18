// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2020 David Hansel
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

#include "cdrive.h"
#include "config.h"
#include "host.h"
#include "cpucore.h"
#include "Altair8800.h"
#include "timer.h"
#include "image.h"
#include "host.h"
#include "mem.h"
#include "prog_tools.h"
#include "io.h"

#if NUM_CDRIVES == 0

void cdrive_setup() {}
void cdrive_dir() {}
const char *cdrive_get_image_filename(byte image_num, bool check_exist) { return NULL; }
const char *cdrive_get_image_description(byte image_num) { return NULL; }
bool cdrive_mount(byte cdrive_num, byte image_num) { return false; }
bool cdrive_unmount(byte cdrive_num) { return false; }
byte cdrive_get_mounted_image(byte cdrive_num) { return 0; }
void cdrive_reset() {}
void cdrive_set_switches(byte switches) {}
byte cdrive_get_switches() { return 0; }

#elif NUM_CDRIVES>4

#error Maximum of 4 drives supported for Cromemco disk controller. Set NUM_CDRIVES<=4 in config.h

#elif !defined(HOST_HAS_FILESYS)

#error Disk drive emulation requires host filesystem

#else

byte cdrive_switches = (CDRIVE_SWITCH_AUTOBOOT | CDRIVE_SWITCH_ROM_DISABLE_AFTER_BOOT);
void cdrive_set_switches(byte switches) { cdrive_switches = switches; }
byte cdrive_get_switches() { return cdrive_switches; }


struct drive_type_struct
{
  uint16_t sector_length;
  uint8_t  num_sectors;
  uint8_t  num_tracks;
  uint8_t  num_sides;
  char     id[7];
};


static struct drive_type_struct drive_types[8] =
  {
    {128, 26, 77, 1, "LGSSSD"}, // space:  243k, image size:  256256 bytes
    {512, 16, 77, 1, "LGSSDD"}, // space:  600k, image size:  625920 bytes
    {128, 26, 77, 2, "LGDSSD"}, // space:  494k, image size:  512512 bytes
    {512, 16, 77, 2, "LGDSDD"}, // space: 1216k, image size: 1256704 bytes
    {128, 18, 40, 1, "SMSSSD"}, // space:   83k, image size:   92160 bytes
    {512, 10, 40, 1, "SMSSDD"}, // space:  190k, image size:  201984 bytes
    {128, 18, 40, 2, "SMDSSD"}, // space:  173k, image size:  184320 bytes
    {512, 10, 40, 2, "SMDSDD"}  // space:  390k, image size:  406784 bytes
  };


static uint32_t cdrive_get_image_size(uint8_t type)
{
  // track 0 of DD drives is always formatted as SD
  uint32_t len = drive_types[type&0xFE].sector_length * drive_types[type&0xFE].num_sectors;

  return len + (drive_types[type].sector_length *
                drive_types[type].num_sectors *
                ((drive_types[type].num_sides*drive_types[type].num_tracks)-1));
}


static byte cdrive_get_type_id(bool maxi, bool DS, bool DD)
{
  return (maxi ? 0 : 4) + (DS ? 2 : 0) + (DD ? 1 : 0);
}


static byte cdrive_get_type_id(HOST_FILESYS_FILE_TYPE file, uint32_t image_file_size)
{
  char id[6];

  // attempt to identify the disk type by its type id (6 bytes starting at pos 0x0078)
  if( host_filesys_file_seek(file, 0x78) )
    if( host_filesys_file_read(file, 6, id)==6 )
      for(byte i=0; i<8; i++)
        if( strncmp(id, drive_types[i].id, 6)==0 )
          return i;

  // otherwise attempt to identify by file size
  for(byte i=0; i<8; i++)
    if( image_file_size==cdrive_get_image_size(i) )
      return i;

  // default to 8" SS SD
  return 0;
}



#define DRIVE_STATUS_NOTREADY    0x80
#define DRIVE_STATUS_WPROTECT    0x40
#define DRIVE_STATUS_HEADDOWN    0x20
#define DRIVE_STATUS_NOTFOUND    0x10
#define DRIVE_STATUS_CRCERR      0x08
#define DRIVE_STATUS_TRACK0      0x04
#define DRIVE_STATUS_LOSTDATA    0x04 // data was not read/written within 32us (64us for 5" drive) after DRQ
#define DRIVE_STATUS_DRQ         0x02
#define DRIVE_STATUS_BUSY        0x01


#define DRIVE_FLAGS_DRQ          0x80
#define DRIVE_FLAGS_NOTBOOT      0x40
#define DRIVE_FLAGS_HEADLOAD     0x20
#define DRIVE_FLAGS_MOTORON      0x08
#define DRIVE_FLAGS_MOTORTIMEOUT 0x04
#define DRIVE_FLAGS_WAITTIMEOUT  0x02
#define DRIVE_FLAGS_EOJ          0x01


#define DRIVE_CMD_IDLE       0
#define DRIVE_CMD_READADDR   1
#define DRIVE_CMD_READ       2
#define DRIVE_CMD_READMULT   3
#define DRIVE_CMD_WRITE      4
#define DRIVE_CMD_WRITEMULT  5
#define DRIVE_CMD_READTRACK  6
#define DRIVE_CMD_WRITETRACK 7

#define MOTOR_TIME (8000000*2)


static byte drive_selected = 0xff;
static byte drive_mounted_disk[NUM_CDRIVES], drive_mounted_disk_type[NUM_CDRIVES];
static byte drive_track, drive_sector, drive_data, drive_status, drive_flags, drive_config_flags, drive_cmd;
static HOST_FILESYS_FILE_TYPE drive_file[NUM_CDRIVES];
static byte drive_buffer[512];
static uint8_t drive_current_head, drive_current_track[NUM_CDRIVES], drive_current_sector;
static uint16_t drive_current_byte;
static uint32_t drive_drq_timeout, drive_motor_timeout, drive_eoj_timeout;


#define DRIVE_SECTOR_LENGTH    (drive_current_track[drive_selected]==0&&drive_current_head==0 ? drive_types[drive_mounted_disk_type[drive_selected]&0xFE].sector_length : drive_types[drive_mounted_disk_type[drive_selected]].sector_length)
#define DRIVE_NUM_SECTORS      (drive_current_track[drive_selected]==0&&drive_current_head==0 ? drive_types[drive_mounted_disk_type[drive_selected]&0xFE].num_sectors   : drive_types[drive_mounted_disk_type[drive_selected]].num_sectors)
#define DRIVE_NUM_TRACKS       (drive_types[drive_mounted_disk_type[drive_selected]].num_tracks)
#define DRIVE_ROTATION_US      (drive_config_flags & 0x10 ? 166667 : 200000)


static uint32_t cdrive_get_file_pos()
{
  uint32_t pos;
  uint8_t i = drive_mounted_disk_type[drive_selected];

  if( drive_current_track[drive_selected]==0 && drive_current_head==0 )
    {
      // first track is always SD
      pos = (drive_current_sector-1) * drive_types[i&0xFE].sector_length;
    }
  else
    {
      // tracks for heads 0 and 1 are interleaved in image file
      pos = (((drive_current_track[drive_selected] * drive_types[i].num_sides)+drive_current_head) * 
             (drive_types[i].num_sectors * drive_types[i].sector_length) 
             +
             (drive_types[i].sector_length * (drive_current_sector-1)));
      
      // track 0 of DD drives is still formatted as SD => if this is DD and we're
      // not on track 0 then we need to correct the position by the length 
      // difference between a DD track and a SD track
      if( (i&1)!=0 && (drive_current_track[drive_selected]!=0 || drive_current_head!=0) )
        pos -= ((drive_types[i].num_sectors * drive_types[i].sector_length)
                -
                (drive_types[i-1].num_sectors * drive_types[i-1].sector_length));
    }
  
  return pos;
}


static void cdrive_set_current_track(byte drive, byte track)
{
  drive_current_track[drive] = track;

  // If drives A/B (or C/D) are the same type (8" or 5") then consider
  // them dual drives and move the other drive's head with this drive
  // (CDOS does the same)
  byte other = (drive & 0xFE) || (1-(drive&1));
  if( (drive_mounted_disk_type[drive]/4)==(drive_mounted_disk_type[other]/4) )
    drive_current_track[other] = track;
}


static void cdrive_flush()
{
  if( drive_current_byte>0 )
    {
      //printf("writing disk file: %04X\n", cdrive_get_file_pos());
      host_filesys_file_seek(drive_file[drive_selected], cdrive_get_file_pos());
      host_filesys_file_write(drive_file[drive_selected], drive_current_byte, drive_buffer);
      host_filesys_file_flush(drive_file[drive_selected]);
    }
}


void cdrive_dir()
{
  Serial.print(image_get_dir_content(IMAGE_CROMEMCO));
}


bool cdrive_unmount(byte drive_num)
{
  if( drive_num<NUM_CDRIVES && drive_mounted_disk[drive_num]!=0 )
    {
      drive_mounted_disk[drive_num] = 0;
      drive_mounted_disk_type[drive_num] = 0xff;
      host_filesys_file_close(drive_file[drive_num]);
      cdrive_register_ports();
    }

  return true;
}


byte cdrive_get_mounted_image(byte drive_num)
{
  return drive_mounted_disk[drive_num];
}


const char *cdrive_get_image_description(byte disk_num)
{
  return image_get_description(IMAGE_CROMEMCO, disk_num);
}


const char *cdrive_get_image_filename(byte image_num, bool check_exist)
{
  return image_get_filename(IMAGE_CROMEMCO, image_num, check_exist);
}


bool cdrive_mount(byte drive_num, byte image_num)
{
  if( drive_num<NUM_CDRIVES )
    {
      if( drive_mounted_disk[drive_num]>0 ) cdrive_unmount(drive_num);
      if( image_num>0 )
        {
          char filename[13];
          image_get_filename(IMAGE_CROMEMCO, image_num, filename, 13, false);

          drive_mounted_disk[drive_num] = image_num;
          drive_file[drive_num] = host_filesys_file_open(filename, true);
          drive_mounted_disk_type[drive_num] = cdrive_get_type_id(drive_file[drive_num], host_filesys_file_size(filename));

          //printf("mounted disk in drive %i is type = %i: %s\n", drive_num, drive_mounted_disk_type[drive_num], drive_types[drive_mounted_disk_type[drive_num]].id);
          cdrive_register_ports();
          return true;
        }
    }

  return false;
}


void cdrive_reset()
{
  drive_selected = 0xff;
  drive_status = 0x00;
  drive_flags  = DRIVE_FLAGS_NOTBOOT | DRIVE_FLAGS_EOJ;
  drive_track  = 0;
  drive_sector = drive_current_sector = 0;
  drive_current_head = 0;
  drive_cmd = DRIVE_CMD_IDLE;
  drive_drq_timeout = 0;
  drive_motor_timeout = 0;
  drive_eoj_timeout = 0;
  for(byte i=0; i<NUM_CDRIVES; i++) cdrive_set_current_track(i, 0);

  // check whether RDOS ROM is already installed and install if not
  if( (cdrive_switches & CDRIVE_SWITCH_ROM_ENABLE)>0 && mem_find_rom("RDOS")==0xff && cpu_get_processor()==PROC_Z80 )
    {
      prog_tools_copy_rdos10();
      mem_set_rom_flags(mem_find_rom("RDOS"), MEM_ROM_FLAG_TEMP|MEM_ROM_FLAG_AUTOSTART);
    }
}


void cdrive_check_drq_timeout()
{
  if( drive_drq_timeout>0 && timer_get_cycles()>drive_drq_timeout )
    {
      //printf("DRQ TIMEOUT!\n");
      drive_status |= DRIVE_STATUS_LOSTDATA;
      drive_status &= ~DRIVE_STATUS_BUSY;
      //printf("%02x\n", drive_status);
      drive_flags |= DRIVE_FLAGS_EOJ;
      drive_cmd = DRIVE_CMD_IDLE;
      drive_drq_timeout = 0;
    }
}

byte cdrive_in(byte port)
{
  byte data = 0xff;
  cdrive_check_drq_timeout();

  switch( port )
     {
     case 0x04:
       {
         // parallel/aux in
         // bit 7   : DRQ
         // bit 6   : 1=seek in progress (always 0 here)
         // bit 5-4 : unused
         // bit 3   : baud rate preset (unused here)
         // bit 2-0 : sense switches 6-8 (reserved/unused)
         //           (bits 0-1 appear to select boot drive in RDOS 2)
         data = (drive_flags & DRIVE_FLAGS_DRQ) ? 0xBF : 0x3F;
         break;
       }
       
     case 0x30:
       {
         // read drive status
         // Last command        D7          D6              D5            D4          D3          D2          D1      D0
         // SEEK/STEP/RESTORE   not ready   write protect   head down     not found   CRC error   Track 0     index   busy
         // READ RECORD(s)      not ready   record type     record type   not found   CRC error   lost data   DRQ     busy
         // WRITE RECORD(s)     not ready   write protect   write fault   not found   CRC error   lost data   DRQ     busy
         // READ ADDRESS        not ready   0               0             not found   CRC error   lost data   DRQ     busy
         // READ TRACK          not ready   0               0             0           0           lost data   DRQ     busy
         // WRITE TRACK         not ready   write protect   write fault   0           0           lost data   DRQ     busy

         data = drive_status;
         drive_status &= ~(DRIVE_STATUS_HEADDOWN | DRIVE_STATUS_NOTFOUND | DRIVE_STATUS_LOSTDATA | DRIVE_STATUS_DRQ);
         //printf("%04x: STATUS=%02x\n", regPC-1, data);
         break;
       }
       
     case 0x31:
       {
         // read track register
         data = drive_track;
         break;
       }

     case 0x32:
       {
         // read sector register
         data = drive_sector;
         break;
       }

     case 0x33:
       {
         // read data register
         if( (drive_cmd & 0xFE)==DRIVE_CMD_READ )
           {
             drive_data = drive_buffer[drive_current_byte++];
             if( drive_current_byte==DRIVE_SECTOR_LENGTH )
               {
                 if( drive_cmd==DRIVE_CMD_READMULT )
                   {
                     if( drive_current_sector<DRIVE_NUM_SECTORS )
                       {
                         drive_current_sector++;

                         //printf("reading disk file: %04X\n", cdrive_get_file_pos());
                         host_filesys_file_seek(drive_file[drive_selected], cdrive_get_file_pos());
                         uint32_t n = host_filesys_file_read(drive_file[drive_selected], DRIVE_SECTOR_LENGTH, drive_buffer);
                         if( n<DRIVE_SECTOR_LENGTH ) memset(drive_buffer+n, 0, DRIVE_SECTOR_LENGTH-n);
                         drive_current_byte = 0;
                         drive_motor_timeout = timer_get_cycles() + MOTOR_TIME;
                       }
                     else
                       {
                         drive_status = DRIVE_STATUS_NOTFOUND;
                         drive_flags  &= ~DRIVE_FLAGS_DRQ;
                         drive_flags  |= DRIVE_FLAGS_EOJ;
                         drive_cmd = DRIVE_CMD_IDLE;
                         drive_current_sector = 1;
                       }
                   }
                 else
                   {
                     drive_status = 0;
                     drive_flags  &= ~DRIVE_FLAGS_DRQ;
                     drive_flags  |= DRIVE_FLAGS_EOJ;
                     drive_cmd = DRIVE_CMD_IDLE;
                   }
               }
           }
         else if( drive_cmd == DRIVE_CMD_READADDR )
           {
             drive_data = drive_buffer[drive_current_byte++];
             if( drive_current_byte==6 )
               {
                 drive_status = 0;
                 drive_flags  &= ~DRIVE_FLAGS_DRQ;
                 drive_flags  |= DRIVE_FLAGS_EOJ;
                 drive_cmd = DRIVE_CMD_IDLE;
               }
           }

         data = drive_data;
         drive_drq_timeout = drive_flags&DRIVE_FLAGS_DRQ ? timer_get_cycles() + 32 * 2 : 0;

         //printf("%04x:%02x ", regPC-1, data);
         break;
       }

     case 0x34: 
       {
         // read drive flags:
         // D7: DRQ (byte can be read/written)
         // D6: ~BOOT (0="BOOT" switch SW3 is set to boot)
         // D5: HEADLOAD
         // D4: initialization/format inhibit (0="format inhibit" switch SW4 is active)
         // D3: MOTOR ON
         // D2: MOTOR TIMEOUT
         // D1: AUTOWAIT TIMEOUT
         // D0: EOJ (end-of-job)

         static byte prev = 0;
         data = drive_flags & 0x81;
         if( (cdrive_switches & CDRIVE_SWITCH_AUTOBOOT)==0 )     data |= 0x40;
         if( (cdrive_switches & CDRIVE_SWITCH_INHIBIT_INIT)==0 ) data |= 0x10;

         if( drive_eoj_timeout>0 && timer_get_cycles()>=drive_eoj_timeout )
           {
             drive_flags |= DRIVE_FLAGS_EOJ;
             drive_eoj_timeout = 0;
           }

         if( drive_motor_timeout>0 )
           {
             if( timer_get_cycles() > drive_motor_timeout )
               {
                 data |= DRIVE_FLAGS_MOTORTIMEOUT;
                 drive_motor_timeout = 0;
               }
             else
               data |= DRIVE_FLAGS_MOTORON;
           }

         if( (prev & (DRIVE_FLAGS_MOTORON|DRIVE_FLAGS_MOTORTIMEOUT)) != (data & (DRIVE_FLAGS_MOTORON|DRIVE_FLAGS_MOTORTIMEOUT)) )
           {
             //printf("%u: MOTOR %s %s\n", timer_get_cycles(), data & DRIVE_FLAGS_MOTORON ? "ON" : "off", data & DRIVE_FLAGS_MOTORTIMEOUT ? "(TIMEOUT)" : "");
             prev = data;
           }

         break;
       }

     case 0xF0:
       data = 0xFF; 
       break;
    }

  //printf("%04x: cdrive IN  : %02x -> %02x\n", regPC-1, port, data);
  return data;
}



void cdrive_out(byte port, byte data)
{
  int8_t prev_dir = 1;
  //printf("%04x: cdrive OUT : %02x <- %02x\n", regPC-1, port, data);
  cdrive_check_drq_timeout();

  switch( port )
    {
    case 0x04:
      {
        // parallel/aux out
        // D6: EJECT LEFT
        // D5: EJECT RIGHT
        // D4: FAST SEEK
        // D3: RESTORE (negative logic)
        // D2: CONTROL OUT
        // D1: SIDE SELECT (1=side 1, 0=side 2)
        // D0: --
        if( (data & 0x08)==0 ) 
          {
            //printf("%04x: %i:RESTORE-04\n", drive_selected, regPC-1);
            cdrive_set_current_track(drive_selected, 0);
          }
        drive_current_head = (data & 0x02) ? 0 : 1;
        //printf("drive head: %02x %i\n", data, drive_current_head);
        break;
      }

    case 0x30:
      {
        // write command register:
        //
        // Command           D7  D6  D5  D4  D3  D2  D1  D0
        // RESTORE           0   0   0   0   1   v   r1  r0
        // SEEK              0   0   0   1   1   v   r1  r0 
        // STEP              0   0   1   u   1   v   r1  r0
        // STEP IN           0   1   0   u   1   v   r1  r0
        // STEP OUT          0   1   1   u   1   v   r1  r0
        // READ RECORD(s)    1   0   0   m   S   E   0   0
        // WRITE RECORD(s)   1   0   1   m   S   E   a1  a0
        // READ ADDRESS      1   1   0   0   0   1   0   0
        // READ TRACK        1   1   1   0   0   1   0   s
        // WRITE TRACK       1   1   1   1   0   1   0   0
        // FORCE INTERRUPT   1   1   0   1   I3  I2  I1  I0
        //
        // v: 1=verify on last track
        // u: 1=update track register
        // m: 1=multiple records (0=single record)
        // S: 1=compare for side
        // E: 1=enable head load delay (0=assume head is down)
        // s: 1=no synchronization (0=synchronize to address mark)
        // r10: stepping rate

        if( (data & 0xF0)==0x00 )
          {
            // RESTORE
            //printf("%04x: %i:RESTORE\n", regPC-1, drive_selected);
            if( drive_selected<0xff )
              {
                drive_track = 0;
                cdrive_set_current_track(drive_selected, 0);
                drive_status |= DRIVE_STATUS_TRACK0;
              }
            else
              drive_status = DRIVE_STATUS_NOTREADY;

            drive_flags |= DRIVE_FLAGS_EOJ;
          }
        else if( (data & 0xF8)==0x18 )
          {
            // SEEK
            //printf("%04x: %i:SEEK %i\n", regPC-1, drive_selected, drive_data);
            drive_flags |= DRIVE_FLAGS_EOJ;
            if( drive_selected >= NUM_CDRIVES || drive_mounted_disk[drive_selected]==0 )
              drive_status = DRIVE_STATUS_NOTREADY;
            else if( drive_data<DRIVE_NUM_TRACKS )
              {
                drive_track = drive_data;
                cdrive_set_current_track(drive_selected, drive_data);
                drive_status = (drive_status & DRIVE_STATUS_HEADDOWN);
                if( drive_current_track[drive_selected]==0 ) drive_status |= DRIVE_STATUS_TRACK0;
                drive_motor_timeout = timer_get_cycles() + MOTOR_TIME;
              }
            else
              drive_status = (drive_status & DRIVE_STATUS_HEADDOWN) | DRIVE_STATUS_NOTFOUND;
          }
        else if( (data & 0xE8)==0x28 )
          {
            // STEP
            //printf("%04x: %i:STEP\n", regPC-1, drive_selected);
            drive_flags |= DRIVE_FLAGS_EOJ;
            if( drive_selected >= NUM_CDRIVES || drive_mounted_disk[drive_selected]==0 )
              drive_status = DRIVE_STATUS_NOTREADY;
            else if( drive_current_track[drive_selected] + prev_dir <= DRIVE_NUM_TRACKS-1 )
              {
                cdrive_set_current_track(drive_selected, drive_current_track[drive_selected]+prev_dir);
                if( data & 0x10 ) drive_track = drive_current_track[drive_selected];
                drive_status = (drive_status & DRIVE_STATUS_HEADDOWN);
                drive_motor_timeout = timer_get_cycles() + MOTOR_TIME;
              }
            else
              drive_status = (drive_status & DRIVE_STATUS_HEADDOWN) | DRIVE_STATUS_NOTFOUND;
          }
        else if( (data & 0xE8)==0x48 )
          {
            // STEP IN
            prev_dir = 1;
            //printf("%04x: %i:STEP IN\n", regPC-1, drive_selected);
            drive_flags |= DRIVE_FLAGS_EOJ;
            if( drive_selected >= NUM_CDRIVES || drive_mounted_disk[drive_selected]==0 )
              drive_status = DRIVE_STATUS_NOTREADY;
            else if( drive_current_track[drive_selected] < DRIVE_NUM_TRACKS-1 )
              {
                cdrive_set_current_track(drive_selected, drive_current_track[drive_selected]+1);
                if( data & 0x10 ) drive_track = drive_current_track[drive_selected];
                drive_status = (drive_status & DRIVE_STATUS_HEADDOWN);
                drive_motor_timeout = timer_get_cycles() + MOTOR_TIME;
              }
            else
              drive_status = (drive_status & DRIVE_STATUS_HEADDOWN) | DRIVE_STATUS_NOTFOUND;
          }
        else if( (data & 0xE8)==0x68 )
          {
            // STEP OUT
            prev_dir = -1;
            //printf("%04x: %i:STEP OUT\n", regPC-1, drive_selected);
            drive_flags |= DRIVE_FLAGS_EOJ;
            if( drive_selected >= NUM_CDRIVES || drive_mounted_disk[drive_selected]==0 )
              drive_status = DRIVE_STATUS_NOTREADY;
            else if( drive_current_track[drive_selected] > 0 )
              {
                cdrive_set_current_track(drive_selected, drive_current_track[drive_selected]-1);
                if( data & 0x10 ) drive_track = drive_current_track[drive_selected];
                drive_status = (drive_status & DRIVE_STATUS_HEADDOWN);
                drive_motor_timeout = timer_get_cycles() + MOTOR_TIME;
              }
            else
              drive_status = (drive_status & DRIVE_STATUS_HEADDOWN) | DRIVE_STATUS_NOTFOUND;
          }
        else if( (data & 0xE0)==0x80 )
          {
            // READ RECORDs
            if( drive_selected >= NUM_CDRIVES || drive_mounted_disk[drive_selected]==0 )
              drive_status = DRIVE_STATUS_NOTREADY;
            else 
              {
                drive_status = DRIVE_STATUS_BUSY | DRIVE_STATUS_DRQ;
                drive_flags |= DRIVE_FLAGS_DRQ;
                drive_flags &= ~DRIVE_FLAGS_EOJ;
                drive_current_sector = drive_sector;
                drive_cmd = data & 0x10 ? DRIVE_CMD_READMULT : DRIVE_CMD_READ;
                
                //printf("%04x: %i:READ RECORD%s %i/%i/%i\n", regPC-1, drive_selected, data & 0x10 ? "s" : "", drive_current_head, drive_current_track[drive_selected], drive_current_sector);
                //printf("reading disk file: %04X\n", cdrive_get_file_pos());
                host_filesys_file_seek(drive_file[drive_selected], cdrive_get_file_pos());
                uint32_t n = host_filesys_file_read(drive_file[drive_selected], DRIVE_SECTOR_LENGTH, drive_buffer);
                if( n<DRIVE_SECTOR_LENGTH ) memset(drive_buffer+n, 0, DRIVE_SECTOR_LENGTH-n);
                drive_current_byte = 0;
                drive_drq_timeout = timer_get_cycles() + (166667/DRIVE_NUM_SECTORS) * 2;
                drive_motor_timeout = timer_get_cycles() + MOTOR_TIME;
              }
          }
        else if( (data & 0xE0)==0xA0 )
          {
            // WRITE RECORDs
            if( drive_selected >= NUM_CDRIVES || drive_mounted_disk[drive_selected]==0 )
              drive_status = DRIVE_STATUS_NOTREADY;
            else 
              {
                drive_status = DRIVE_STATUS_HEADDOWN | DRIVE_STATUS_BUSY | DRIVE_STATUS_DRQ;
                drive_flags |= DRIVE_FLAGS_DRQ;
                drive_flags &= ~DRIVE_FLAGS_EOJ;
                drive_current_sector = drive_sector;
                drive_cmd = data & 0x10 ? DRIVE_CMD_WRITEMULT : DRIVE_CMD_WRITE;
                
                //printf("%04x: %i:WRITE RECORD%s %i/%i/%i\n", regPC-1, drive_selected, data & 0x10 ? "s" : "", drive_current_head, drive_current_track[drive_selected], drive_current_sector);
                drive_current_byte = 0;
                // spindle speed is 360RPM = 166667us/rotation
                drive_drq_timeout = timer_get_cycles() + (DRIVE_ROTATION_US/DRIVE_NUM_SECTORS) * 2;
                drive_motor_timeout = timer_get_cycles() + MOTOR_TIME;
              }
          }
        else if( data == 0xC4 )
          {
            // READ ADDRESS
            //printf("CMD:READ ADDRESS\n");

            if( drive_selected >= NUM_CDRIVES || drive_mounted_disk[drive_selected]==0 )
              drive_status = DRIVE_STATUS_NOTREADY;
            else             
              {
                drive_cmd    = DRIVE_CMD_READADDR;
                drive_status = DRIVE_STATUS_HEADDOWN | DRIVE_STATUS_BUSY | DRIVE_STATUS_DRQ;
                drive_flags |= DRIVE_FLAGS_DRQ;
                drive_flags &= ~DRIVE_FLAGS_EOJ;
                
                drive_current_sector++;
                if( drive_current_sector>DRIVE_NUM_SECTORS ) drive_current_sector = 1;
                
                drive_buffer[0] = drive_current_track[drive_selected];
                drive_buffer[1] = 0;
                drive_buffer[2] = drive_current_sector;
                drive_buffer[3] = (byte) DRIVE_SECTOR_LENGTH;
                drive_buffer[4] = 0;
                drive_buffer[5] = 0;
                drive_current_byte = 0;
                drive_drq_timeout = timer_get_cycles() + 32 * 2;
                drive_motor_timeout = timer_get_cycles() + MOTOR_TIME;
              }
          }
        else if( data == 0xF4 )
          {
            // WRITE TRACK
            //printf("%i %i %02x\n", drive_current_track[drive_selected], drive_current_head, drive_config_flags);

            if( drive_selected >= NUM_CDRIVES || drive_mounted_disk[drive_selected]==0 )
              drive_status = DRIVE_STATUS_NOTREADY;
            else if( (cdrive_switches & CDRIVE_SWITCH_INHIBIT_INIT)==0 )
              {
                //printf("CMD:WRITE TRACK\n");

                byte ot = drive_mounted_disk_type[drive_selected];
                if( (drive_current_track[drive_selected]==0 && drive_current_head==0) || (drive_current_track[drive_selected]==1 && drive_current_head==0 && (ot&2)==0) )
                  {
                    byte t = cdrive_get_type_id((drive_config_flags&0x10)!=0, false, (drive_config_flags&0x40)!=0);
                    drive_mounted_disk_type[drive_selected] = t;
                  }
                else if( drive_current_track[drive_selected]==0 && drive_current_head==1 )
                  {
                    byte t = cdrive_get_type_id((drive_config_flags&0x10)!=0, true, (drive_config_flags&0x40)!=0);
                    drive_mounted_disk_type[drive_selected] = t;
                  }

                /*if( drive_current_track[drive_selected]==0 && drive_current_head==0 )
                  printf("formatting disk %s as type = %i: %s\n", 
                         cdrive_get_image_filename(drive_mounted_disk[drive_selected], false), 
                         drive_mounted_disk_type[drive_selected], drive_types[drive_mounted_disk_type[drive_selected]].id);
                else if( drive_mounted_disk_type[drive_selected]!=ot )
                  printf("updated format type = %i: %s\n", 
                  drive_mounted_disk_type[drive_selected], drive_types[drive_mounted_disk_type[drive_selected]].id);*/
                
                drive_status = DRIVE_STATUS_HEADDOWN | DRIVE_STATUS_BUSY | DRIVE_STATUS_DRQ;
                drive_flags |= DRIVE_FLAGS_DRQ;
                drive_flags &= ~DRIVE_FLAGS_EOJ;
                drive_cmd = DRIVE_CMD_WRITETRACK;

                drive_current_sector = 1;
                drive_current_byte = 0xffff;
                // spindle speed is 6 rotations/second = 166667us/rotation
                drive_drq_timeout = timer_get_cycles() + (DRIVE_ROTATION_US/DRIVE_NUM_SECTORS) * 2;
                drive_motor_timeout = timer_get_cycles() + MOTOR_TIME;
              }
          }
        else if( (data&0xfe) == 0xE4 )
          {
            // READ TRACK (not implemented)
            drive_status = DRIVE_STATUS_NOTREADY;
            drive_flags |= DRIVE_FLAGS_EOJ;
          }
        else if( (data & 0xF0)==0xd0 )
          {
            // FORCE INTERRUPT
            if( (drive_cmd&0xFE)==DRIVE_CMD_WRITE )
              cdrive_flush();

            drive_flags = 0;
            drive_status = 0;
            drive_drq_timeout = 0;
            drive_eoj_timeout = 0;
            drive_cmd = DRIVE_CMD_IDLE;

            if( (data & 0x0F)==4 )
              {
                // wait for next index pulse to set EOJ, spindle speed is 6 rotations/second
                drive_eoj_timeout = timer_get_cycles() + 166667*2;
              }
            else if( (data & 0x0F)!=0 )
              {
                // just set EOJ immediately
                drive_flags = DRIVE_FLAGS_EOJ;
              }
            
            //printf("%04x: FORCE INTERRUPT %02x => %02x\n", regPC-1, data, drive_flags);
          }

        break;
      }

    case 0x31:
      {
        // write track register
        drive_track = data;
        break;
      }

    case 0x32:
      {
        // write sector register
        drive_sector = data;
        break;
      }
      
    case 0x33:
      {
        // write data register
        drive_data = data;

        if( (drive_cmd&0xFE)==DRIVE_CMD_WRITE )
          {
            //printf("%04x/%02x ", regPC-1, drive_data);
            drive_buffer[drive_current_byte++] = drive_data;
            if( drive_current_byte==DRIVE_SECTOR_LENGTH ) 
              {
                cdrive_flush();
                drive_motor_timeout = timer_get_cycles() + MOTOR_TIME;
                if( drive_cmd==DRIVE_CMD_WRITEMULT )
                  {
                    if( drive_current_sector<DRIVE_NUM_SECTORS )
                      {
                        drive_current_sector++;
                        drive_current_byte = 0;
                      }
                    else
                      {
                        drive_status = DRIVE_STATUS_NOTFOUND;
                        drive_flags  &= ~DRIVE_FLAGS_DRQ;
                        drive_flags  |= DRIVE_FLAGS_EOJ;
                        drive_cmd = DRIVE_CMD_IDLE;
                        drive_current_sector = 1;
                      }
                  }
                else
                  {
                    drive_status = 0;
                    drive_flags  &= ~DRIVE_FLAGS_DRQ;
                    drive_flags  |= DRIVE_FLAGS_EOJ;
                    drive_cmd = DRIVE_CMD_IDLE;
                  }
              }
          }
        else if( drive_cmd==DRIVE_CMD_WRITETRACK )
          {
            if( drive_current_byte == 0xffff )
              {
                // skip until data address mark
                if( data == 0xfb ) drive_current_byte = 0;
              }
            else
              {
                drive_buffer[drive_current_byte++] = drive_data;
                if( drive_current_byte==DRIVE_SECTOR_LENGTH ) 
                  {
                    cdrive_flush();
                    drive_motor_timeout = timer_get_cycles() + MOTOR_TIME;
                    if( drive_current_sector<DRIVE_NUM_SECTORS )
                      {
                        drive_current_byte = 0xffff;
                        drive_current_sector++;
                      }
                    else
                      {
                        drive_status = 0;
                        drive_flags  &= ~DRIVE_FLAGS_DRQ;
                        drive_flags  |= DRIVE_FLAGS_EOJ;
                        drive_cmd = DRIVE_CMD_IDLE;
                        drive_current_sector = 1;
                      }
                  }
              }
          }

        drive_drq_timeout = drive_flags&DRIVE_FLAGS_DRQ ? timer_get_cycles() + 32 * 2 : 0;

        break;
      }

    case 0x34:
      {
        // write disk control register:
        //
        // D7: auto-wait, if 1 then reading port 0x34 will halt CPU until DRQ, EOJ or RESET
        // D6: double density (1=DD/0=SD)
        // D5: motor on (for all drives, set during RESET)
        // D4: MAXI (1=8" drive, 0=5" drive, set during RESET)
        // D3-D0: drive select (drive 4-1) RESET deselects all drives

        //printf("%i %i\n", drive_selected, data & 0x10);
        drive_config_flags = data;

        byte prevdrive = drive_selected;
        if     ( data&0x08 ) drive_selected = 3;
        else if( data&0x04 ) drive_selected = 2;
        else if( data&0x02 ) drive_selected = 1;
        else if( data&0x01 ) drive_selected = 0;
        else                 drive_selected = 0xff;

        if( drive_selected<4 ) drive_track = drive_current_track[drive_selected];

        // turn motor on/off
        if( data & 0x20 )
          drive_motor_timeout = timer_get_cycles() + MOTOR_TIME;
        else
          drive_motor_timeout = 0;

        //if( data&0x80 ) printf ("AUTO-WAIT ON\n"); else printf ("AUTO-WAIT off\n"); 
        //if( drive_selected!=prevdrive ) printf("SELECT DRIVE %i\n", drive_selected+1);
        break;
      }

    case 0x40:
      {
        // bank select register => writing here will turn off ROM if selected by switch

        if( cdrive_switches & CDRIVE_SWITCH_ROM_DISABLE_AFTER_BOOT )
          mem_disable_rom("RDOS");
        
        break;
      }
    }
}


void cdrive_register_ports()
{
  bool drive_used = false;
  for(byte i=0; i<NUM_CDRIVES; i++)
    drive_used |= drive_mounted_disk[i]!=0;

  for(byte i=0x30; i<=0x34; i++)
    {
      io_register_port_inp(i, drive_used ? cdrive_in : NULL);
      io_register_port_out(i, drive_used ? cdrive_out : NULL);
    }

  if( config_vdm1_keyboard_device()==0xFF )
    {
      // both VDM1 keyboard support and Cromemco disk controller use port 4.  
      // We give the VDM1 keyboard priority to use that port.
      io_register_port_inp(0x04, drive_used ? cdrive_in : NULL);
      io_register_port_out(0x04, drive_used ? cdrive_out : NULL);
    }

  io_register_port_inp(0xF0, drive_used ? cdrive_in : NULL);
  io_register_port_out(0x40, drive_used ? cdrive_out : NULL);
}


void cdrive_setup()
{
  for(byte i=0; i<NUM_CDRIVES; i++)
    {
      drive_mounted_disk_type[i] = 0xff;
      drive_mounted_disk[i] = 0;
    }
  
  cdrive_register_ports();
  cdrive_reset();
}


#endif
