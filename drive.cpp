#include "drive.h"
#include "config.h"
#include "host.h"
#include "cpucore.h"

#if NUM_DRIVES == 0

void drive_setup() {}
void drive_dir() {}
bool drive_mount(byte drive_num, byte disk_num) { return false; }
bool drive_unmount(byte drive_num) { return false; }
void drive_reset() {}
byte drive_in(byte addr) { return 0; }
void drive_out(byte addr, byte data) {}

#else

#define DRIVE_SECTOR_LENGTH    137
#define DRIVE_NUM_SECTORS       32
#define DRIVE_NUM_TRACKS       254

#define DRIVE_STATUS_HAVEDISK    1
#define DRIVE_STATUS_HEADLOAD    2
#define DRIVE_STATUS_WRITE       4

static byte drive_selected = 0xff;
static char drive_file_name[NUM_DRIVES][13];
static byte drive_status[NUM_DRIVES];
static byte drive_current_track[NUM_DRIVES];
static byte drive_current_sector[NUM_DRIVES];
static byte drive_current_byte[NUM_DRIVES];
static byte drive_sector_buffer[NUM_DRIVES][DRIVE_SECTOR_LENGTH];


static uint32_t drive_get_file_pos(byte drive_num)
{
  return drive_current_track[drive_num] * DRIVE_NUM_SECTORS * DRIVE_SECTOR_LENGTH + drive_current_sector[drive_num] * DRIVE_SECTOR_LENGTH;
}


static void drive_flush(byte drive_num)
{
  if( (drive_status[drive_num] & DRIVE_STATUS_WRITE) && drive_current_byte[drive_num]>0 )
    {
      host_write_file(drive_file_name[drive_num], drive_get_file_pos(drive_num), drive_current_byte[drive_num], drive_sector_buffer[drive_num]);
      drive_status[drive_num] &= ~DRIVE_STATUS_WRITE;
      drive_current_byte[drive_num] = 0xff;
    }
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
    }
}


void drive_dir()
{
  char buf[33];
  uint32_t offset = 0, len;
  while( (len=host_read_file("DISKDIR.TXT", offset, 32, buf))>0 )
    {
      buf[len]=0;
      Serial.print(buf);
      offset+=len;
    }
}


bool drive_unmount(byte drive_num)
{
  if( drive_num<NUM_DRIVES && (drive_status[drive_num] & DRIVE_STATUS_HAVEDISK) )
    {
      drive_flush(drive_num);
      drive_status[drive_num] = 0;
    }

  return true;
}


bool drive_mount(byte drive_num, byte disk_num)
{
  if( drive_num<NUM_DRIVES )
    {
      if( drive_status[drive_num] & DRIVE_STATUS_HAVEDISK ) drive_unmount(drive_num);
      sprintf(drive_file_name[drive_num], "DISK%02X.DSK", disk_num);
      drive_status[drive_num] |= DRIVE_STATUS_HAVEDISK;
      return true;
    }
  else
    return false;
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
           I - When 0, indicates interrupts enabled (not used)
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
                if( (drive_status[drive_selected] & DRIVE_STATUS_WRITE) )
                  data |= 0x01; // ready to write
                else
                  data |= 0x80; // ready to read
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
            // advance current sector every time this register is queried
            drive_current_sector[drive_selected]++;
            if( drive_current_sector[drive_selected] >= DRIVE_NUM_SECTORS )
              drive_current_sector[drive_selected] = 0;
            
            if( (drive_status[drive_selected] & DRIVE_STATUS_HAVEDISK) && (drive_status[drive_selected] & DRIVE_STATUS_HEADLOAD) )
              data = 0xC0 | (drive_current_sector[drive_selected] * 2);
            
            drive_current_byte[drive_selected] = 0xff;
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
                byte n = host_read_file(drive_file_name[drive_selected], drive_get_file_pos(drive_selected), DRIVE_SECTOR_LENGTH, drive_sector_buffer[drive_selected]);
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
            if( data & 0x80 )
              {
                drive_flush(drive);
                drive_status[drive] &= DRIVE_STATUS_HAVEDISK;
                drive_current_byte[drive] = 0xff;
              }
            else
              drive_selected = drive;
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
            if( (data & 0x01) && drive_current_track[drive_selected]<DRIVE_NUM_TRACKS-1 ) 
              {
                drive_current_track[drive_selected]++;
                drive_current_byte[drive_selected] = 0xff;
              }
            
            if( (data & 0x02) && drive_current_track[drive_selected]>0 ) 
              {
                drive_current_track[drive_selected]--;
                drive_current_byte[drive_selected] = 0xff;
              }
            
            if( data & 0x04 ) drive_status[drive_selected] |=  DRIVE_STATUS_HEADLOAD;
            if( data & 0x08 ) drive_status[drive_selected] &= ~DRIVE_STATUS_HEADLOAD;
            
            if( data & 0x80 ) 
              {
                drive_status[drive_selected] |=  DRIVE_STATUS_WRITE;
                drive_current_byte[drive_selected] = 0;
              }
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
