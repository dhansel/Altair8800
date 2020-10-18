// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// Copyright (C) 2020 Dirk Herrendoerfer
// -----------------------------------------------------------------------------

#ifndef HOST_TEENSY_H
#define HOST_TEENSY_H

#include "config.h"
#include "switch_serial.h"
#include <SdFat.h>

#define strnicmp(a, b, c) strncasecmp(a, b, c)

// LED Colors / Levels
#define LEDCOLOR 0x100000
#define LEDLOW   0x020000  //Comment to disable
#define LEDOFF   0x000000

// Teensy provides a file system (via SD card)
#define HOST_HAS_FILESYS
#define HOST_FILESYS_FILE_TYPE File
#define HOST_FILESYS_DIR_TYPE  File

#define MEMSIZE 0x10000

#define HOST_STORAGESIZE 1024*512
#define HOST_BUFFERSIZE  0x100

#define HOST_PERFORMANCE_FACTOR 1.0

#define HOST_NUM_SERIAL_PORTS   (4)


// ------------------------------------------ switches


uint8_t host_read_sense_switches();

uint16_t host_read_addr_switches();


// ------------------------------------------ status LEDs
extern volatile uint16_t status_led_local;

#ifdef LEDLOW  
  extern volatile uint16_t status_led_local_low;
#endif

inline void set_status_led_local(uint16_t v)
{
  status_led_local |= v;
#ifdef LEDLOW  
  status_led_local_low |= v;
#endif
}

/* reading global variables is faster than reading back the i/o register
   => INTE and WAIT are read often so we keep their state in a global variable */

#define host_set_status_led_INT()     set_status_led_local(ST_INT)
#define host_set_status_led_WO()      set_status_led_local(ST_WO)
#define host_set_status_led_STACK()   set_status_led_local(ST_STACK)
#define host_set_status_led_HLTA()    set_status_led_local(ST_HLTA)
#define host_set_status_led_OUT()     set_status_led_local(ST_OUT)
#define host_set_status_led_M1()      set_status_led_local(ST_M1)
#define host_set_status_led_INP()     set_status_led_local(ST_INP)
#define host_set_status_led_MEMR()    set_status_led_local(ST_MEMR)
#define host_set_status_led_INTE()    set_status_led_local(ST_INTE)
#define host_set_status_led_PROT()    set_status_led_local(ST_PROT)
#define host_set_status_led_WAIT()  { set_status_led_local(ST_WAIT); status_wait = true; }
#define host_set_status_led_HLDA()    set_status_led_local(ST_HLDA)

#define host_clr_status_led_INT()     status_led_local &= ~(ST_INT)
#define host_clr_status_led_WO()      status_led_local &= ~(ST_WO)
#define host_clr_status_led_STACK()   status_led_local &= ~(ST_STACK)
#define host_clr_status_led_HLTA()    status_led_local &= ~(ST_HLTA)
#define host_clr_status_led_OUT()     status_led_local &= ~(ST_OUT)
#define host_clr_status_led_M1()      status_led_local &= ~(ST_M1)
#define host_clr_status_led_INP()     status_led_local &= ~(ST_INP)
#define host_clr_status_led_MEMR()    status_led_local &= ~(ST_MEMR)
#define host_clr_status_led_INTE()    status_led_local &= ~(ST_INTE);
#define host_clr_status_led_PROT()    status_led_local &= ~(ST_PROT)
#define host_clr_status_led_WAIT()  { status_led_local &= ~(ST_WAIT); status_wait = false; }
#define host_clr_status_led_HLDA()    status_led_local &= ~(ST_HLDA)

#define host_read_status_led_WAIT()   status_wait
#define host_read_status_led_M1()     status_led_local & (ST_M1)
#define host_read_status_led_HLTA()   status_led_local & (ST_HLTA)
#define host_read_status_led_INTE()   status_inte

// reading from memory (MEMR on, WO on)
#define host_set_status_leds_READMEM()        set_status_led_local((ST_MEMR | ST_WO))

// reading opcode from memory (MEMR on, M1 on, WO on)
#define host_set_status_leds_READMEM_M1()     set_status_led_local((ST_MEMR | ST_M1 | ST_WO))

// reading from stack (MEMR on, WO on, STACK on)
#define host_set_status_leds_READMEM_STACK()  set_status_led_local((ST_MEMR | ST_WO | ST_STACK))

// writing to memory (MEMR off, WO off)
#define host_set_status_leds_WRITEMEM()       status_led_local &= ~(ST_MEMR | ST_WO)

inline uint16_t host_read_status_leds()
{
  return status_led_local;
}


// ----------------------------------------------------- address bus

extern volatile uint16_t addr_led_local;
#ifdef LEDLOW
extern volatile uint16_t addr_led_local_low;
#endif

inline void host_set_addr_leds(uint16_t v)
{
  addr_led_local = v;
#ifdef LEDLOW  
  addr_led_local_low |= v;
#endif
}

inline uint16_t host_read_addr_leds()
{
  return addr_led_local;
}


// ---------------------------------------------------- data bus
extern volatile uint16_t data_led_local;
#ifdef LEDLOW
extern volatile uint16_t data_led_local_low;
#endif
inline uint16_t host_set_data_leds(uint16_t v)
{
  data_led_local = v;
#ifdef LEDLOW
  data_led_local_low |= v;
#endif
  return v;
}

inline byte host_read_data_leds()
{
  return data_led_local;
}

// ---------------------------------------------------- interrupts

// On the Teensy we are not using real interrupts for serial.
void host_check_interrupts();

void host_serial_interrupts_pause();
void host_serial_interrupts_resume();

// ---------------------------------------------------- serial_available 

// teensy library already defines a serial_available() function which then clashes 
// with our serial_available in serial.h/.cpp => rename it for the teensy platform
#define serial_available serial_avail


// ---------------------------------------------------- external bus I/O

// external bus I/O not supported on Teensy
#define host_read_status_WAIT() 0
#define host_read_data_bus()    0xFF

#endif
