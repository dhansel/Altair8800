// For ATTINY85 at 5V with a 11.0592MHz crystal
//    1: baud select 0 (PB5)
//    2: Crystal
//    3: Crystal
//    4: GND
//    5: baud select 1 (PB1)
//    6: clock output
//    7: baud select 2 (PB2)
//    8: VCC
//
// Attiny85 RSTDSBL fuse (reset disable) must be enabled (set to 0 - negative logic)
// Attiny85 fuses: LOW=0xFF, HIGH=0x57, EXTENDED=0xFF

#include <avr/sleep.h>

#define CLK 11059200ul
const uint32_t baud[8] = {110, 300, 1200, 2400, 9600, 19200, 38400, 57600};

uint32_t getBaudDIP()
{
  byte dip = PINB;
  dip = ((dip & 0x04) ? 0 : 4) | ((dip & 0x01) ? 0 : 2) | ((dip & 0x20) ? 0 : 1);
  return baud[dip];
}


void setBaud(uint32_t baud)
{
  int prescale = 0;
  uint32_t n;

  do 
    {
      prescale++;
      n = (CLK / ((baud * 16) << prescale)) - 1;
    }
  while( n > 255 && prescale < 15 );

  if( n <= 255 )
    {
      // clear on OCR1C compare match, toggle output on OCR1A compare match
      TCCR1 = bit(CTC1) | bit(COM1A0) | prescale;
      OCR1A = OCR1C = n;
    }
}


ISR(PCINT0_vect) 
{
  // pin change detected on input connected to DIP switch
  // => update timer frequency for selected baud rate
  setBaud(getBaudDIP());
}


void setup()
{
  pinMode(0, INPUT_PULLUP);
  pinMode(1, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);

  // set timer frequency for selected baud rate
  setBaud(getBaudDIP());

  // disable some devices we don't need
  ADCSRA &= ~_BV(ADEN);
  ACSR   |= _BV(ACD);
  sleep_bod_disable();

  // when sleeping, go into "idle" mode (keeps timer running)
  set_sleep_mode(SLEEP_MODE_IDLE);

  // enable pin change interrupt for PB0, PB2 and PB5
  GIMSK |= _BV(PCIE);
  PCMSK |= 0x25;
}


void loop()
{  
  // go to sleep (disables CPU clock)
  // pin change interrupt will wake us up
  sleep_mode();
}
