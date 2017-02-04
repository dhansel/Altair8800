#ifdef _WIN32

#include <Arduino.h>
#include <time.h>
#include <string>
#include "Altair8800.h"
#include "mem.h"
#include "serial.h"
#include "cpucore.h"
#include "host_pc.h"
#include "Windows.h"

byte data_leds;
uint16_t status_leds;
uint16_t addr_leds;
byte stop_request;
static FILE *storagefile = NULL;

//#define DEBUG

// ----------------------------------------------------------------------------------


struct TimerData {
  HostTimerFnTp timer_fn;
  uint32_t milliseconds;
  DWORD    thread_id;
  bool     go;
} timer_data[9];
  

static DWORD WINAPI timer_interrupt_fn(void *d)
{
  byte tid = (word) d;
  struct TimerData *data = timer_data+tid;

  while( data->go )
    {
      Sleep(data->milliseconds);
      data->timer_fn();
    }
  
  printf("[timer thread exit]\n");
  data->thread_id = 0;
  return 0;
}


void host_interrupt_timer_start(byte tid)
{
  printf("[Starting timer interrupts]\n");
  timer_data[tid].go = true;
  HANDLE h = CreateThread(0, 0, timer_interrupt_fn, (void *) tid, 0, &(timer_data[tid].thread_id));
  if( h )
    CloseHandle(h);
  else
    timer_data[tid].thread_id = 0;
}


void host_interrupt_timer_setup(byte tid, uint32_t microseconds, HostTimerFnTp timer_fn)
{
  timer_data[tid].go = false;
  timer_data[tid].timer_fn = timer_fn;
  timer_data[tid].milliseconds = (microseconds/1000.0)+0.5;
  timer_data[tid].thread_id = 0;
}


void host_interrupt_timer_stop(byte tid)
{
  if( timer_data[tid].thread_id )
    {
      printf("[Stopping timer interrupts]\n");
      timer_data[tid].go = false;
      
      // if this is not the timer thread itself then wait until
      // the timer thread exits
      if( GetCurrentThreadId() != timer_data[tid].thread_id )
        {
          HANDLE h = OpenThread(SYNCHRONIZE, false, timer_data[tid].thread_id);
          if( h!=NULL )
            {
              WaitForSingleObject(h, INFINITE); 
              CloseHandle(h);
            }
        }
      
      printf("[Stopped timer interrupts]\n");
    }
}


bool host_interrupt_timer_running(byte tid)
{
  return timer_data[tid].thread_id!=0;
}


// ----------------------------------------------------------------------------------


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


// ----------------------------------------------------------------------------------


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


// ----------------------------------------------------------------------------------


uint32_t host_get_random()
{
  return rand()*65536l | rand();
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
      
      serial_receive_host_data(0, c);
    }
}


void host_serial_setup(byte iface, unsigned long baud, bool set_primary_interface)
{
  // no setup to be done
}


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

  for(byte tid=0; tid<9; tid++)
    {
      timer_data[tid].go = false;
      timer_data[tid].thread_id = 0;
    }

  srand(time(NULL));
}


#endif
