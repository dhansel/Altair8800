#ifdef __AVR_ATmega2560__

#include <Arduino.h>
#include "Altair8800.h"
#include "config.h"
#include "mem.h"
#include "host_mega.h"

#define DEBUG

/*
  Runs emulation at about 0.5 Mhz clock speed (about 1/4 speed of original Altair8800)

  Function switch pin mapping (digital input):
     RUN          => 20
     STOP         => 21 (Port D, bit 0)
     STEP         =>  4
     SLOW         =>  5
     EXAMINE      =>  6
     EXAMINE NEXT =>  7
     DEPOSIT      =>  8
     DEPOSIT NEXT =>  9
     RESET        => 18 (Port D, bit 3)
     CLR          => 19 (Port D, bit 2)
     PROTECT      => 16
     UNPROTECT    => 17
     AUX1 UP      => 14
     AUX1 DOWN    => 15
     AUX2 UP      =>  3 (PORT E, bit 5)
     AUX2 DOWN    =>  2 (PORT E, bit 4)

   Address/Data switch pin mapping (analog input):
     A0...15      => A0...15

   Status LED mapping (digital output):
    +A0..7        => 22, 23, ..., 29  (PORTA)
    +A8..15       => 37, 36, ..., 30  (PORTC)
    +D0..8        => 49, 48, ..., 42  (PORTL)
    *INT          => 53 \
     WO           => 52 |
     STACK        => 51 |
     HLTA         => 50 | STATUS      (PORTB)
     OUT          => 10 |
     M1           => 11 |
     INP          => 12 |
     MEMR         => 13 /
     INTE         => 38
     PROT         => 39
     WAIT         => 40
    *HLDA         => 41

*/


uint16_t host_read_status_leds()
{
  uint16_t res = PORTB;
  res |= PORTD & 0x80 ? ST_INTE : 0;
  res |= PORTG & 0x04 ? ST_PROT : 0;
  res |= PORTG & 0x02 ? ST_WAIT : 0;
  res |= PORTG & 0x01 ? ST_HLDA : 0;
  return res;
}


//------------------------------------------------------------------------------------------------------


static const byte input_switch_pins[16] = {20, 21, 4, 5, 6, 7, 8, 9, 18, 19, 16, 17, 14, 15, 3, 2};

bool host_read_function_switch(byte inputNum)
{
  // digitalRead returns 0 if switch is on
  return !digitalRead(input_switch_pins[inputNum]);
}


//------------------------------------------------------------------------------------------------------


void host_copy_flash_to_ram(void *dst, const void *src, uint32_t len)
{
  for(uint32_t i=0; i<len; i++) 
    ((byte *) dst)[i] = pgm_read_byte(((byte *) src)+i);
}


void host_write_data(const void *data, uint32_t addr, uint32_t len)
{
  byte *b = (byte *) data;
  for(uint32_t i=0; i<len; i++) EEPROM.write(addr+i, b[i]);
}


void host_read_data(void *data, uint32_t addr, uint32_t len)
{
  byte *b = (byte *) data;
  for(uint32_t i=0; i<len; i++) b[i] = EEPROM.read(addr+i);
}


void host_move_data(uint32_t to, uint32_t from, uint32_t len)
{
  for(uint32_t i=0; i<len; i++) EEPROM.write(to+i, EEPROM.read(from+i));
}


// --------------------------------------------------------------------------------------------------


uint32_t host_get_random()
{
  return (uint32_t) random(-2147483647,2147483648);
}


// --------------------------------------------------------------------------------------------------

extern byte host_mega_stop_request = 0;

void host_setup()
{
  int i;
  for(i=0; i<8; i++)
    {
      pinMode(2+i,  INPUT_PULLUP);  
      pinMode(14+i, INPUT_PULLUP);
    }
  for(i=10; i<14; i++) pinMode(i, OUTPUT);
  for(i=22; i<54; i++) pinMode(i, OUTPUT);

  //TODO: Find good way to initialize random number generator
  randomSeed(micros());
}

#endif
