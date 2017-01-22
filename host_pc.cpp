#ifdef _WIN32

#include <Arduino.h>
#include <time.h>
#include <string>
#include "Altair8800.h"
#include "mem.h"
#include "cpucore.h"
#include "host_pc.h"

byte data_leds;
uint16_t status_leds;
uint16_t addr_leds;
byte stop_request;
static FILE *storagefile = NULL;

//#define DEBUG

void host_setup()
{
  data_leds = 0;
  status_leds = 0;
  addr_leds = 0;
  stop_request = 0;

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

  srand(time(NULL));
}


uint32_t host_get_random()
{
  return (uint32_t) rand();
}


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


void host_check_interrupts()
{
  static unsigned long prevCtrlC = 0;
  if( Serial.available() )
    {
      byte c = Serial.read();
      if( c == 3 )
        {
          // CTRL-C was pressed.  If we receive two CTRL-C in short order
          // then we terminate the emulator.
          if( millis()>prevCtrlC+250 )
            prevCtrlC = millis();
          else
            exit(0);
        }
      
      altair_receive_serial_data(c);
    }
}


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
      fread(data, len, 1, storagefile);
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


uint32_t host_read_file(const char *filename, uint32_t offset, uint32_t len, void *buffer)
{
  char fnamebuf[30];
  sprintf(fnamebuf, "disks\\%s", filename);
  uint32_t res = 0;
  FILE *f = fopen(fnamebuf, "rb");
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
  sprintf(fnamebuf, "disks\\%s", filename);
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


#endif
