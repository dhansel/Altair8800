#ifndef HOST_MEGA_H
#define HOST_MEGA_H

#include <EEPROM.h>
#include <avr/pgmspace.h>

// Mega2650: 8k SRAM, use 6k for emulated RAM
#define MEMSIZE 4096+2048

#define HOST_STORAGESIZE 4096 // have 4k EEPROM
#define HOST_BUFFERSIZE  0    // have little SRAM so don't buffer

#define host_set_addr_leds(v)  (PORTA=((v) & 0xff), PORTC=((v) / 256))
#define host_read_addr_leds(v) (PORTA | (PORTC * 256))
#define host_set_data_leds(v)  PORTL=(v)
#define host_read_data_leds()  PORTL

extern byte host_mega_stop_request;
#define host_stop_request_check()   (host_mega_stop_request ? host_mega_stop_request : ((PIND&0x0D)<0x0D || (PINE&0x30)<0x30))  /* STOP, RESET, CLR, AUX2 up or AUX2 down */
#define host_stop_request_clear()   host_mega_stop_request = 0
#define host_stop_request_set(w)    host_mega_stop_request = w;

#if USE_IRQ>0
#define host_serial_irq_check()       (host_read_status_led_INTE() && (reg_2SIO_ctrl & 0x80) && (reg_2SIO_status==0) && Serial.available())
#define host_serial_irq_clear()       while(0)
#else
#define host_serial_irq_check()       false
#define host_serial_irq_clear()       while(0)
#endif

#define host_set_status_led_INT()     PORTB |=  0x01
#define host_set_status_led_WO()      PORTB &= ~0x02
#define host_set_status_led_STACK()   PORTB |=  0x04
#define host_set_status_led_HLTA()    PORTB |=  0x08
#define host_set_status_led_OUT()     PORTB |=  0x10
#define host_set_status_led_M1()      PORTB |=  0x20
#define host_set_status_led_INP()     PORTB |=  0x40
#define host_set_status_led_MEMR()    PORTB |=  0x80
#define host_set_status_led_INTE()  { digitalWrite(38, HIGH); status_inte = true; }
#define host_set_status_led_PROT()    digitalWrite(39, HIGH)
#define host_set_status_led_WAIT()  { digitalWrite(40, HIGH); status_wait = true; }
#define host_set_status_led_HLDA()    digitalWrite(41, HIGH)

#define host_clr_status_led_INT()     PORTB &= ~0x01
#define host_clr_status_led_WO()      PORTB |=  0x02
#define host_clr_status_led_STACK()   PORTB &= ~0x04
#define host_clr_status_led_HLTA()    PORTB &= ~0x08
#define host_clr_status_led_OUT()     PORTB &= ~0x10
#define host_clr_status_led_M1()      PORTB &= ~0x20
#define host_clr_status_led_INP()     PORTB &= ~0x40
#define host_clr_status_led_MEMR()    PORTB &= ~0x80
#define host_clr_status_led_INTE()  { digitalWrite(38, LOW); status_inte = false; }
#define host_clr_status_led_PROT()    digitalWrite(39, LOW)
#define host_clr_status_led_WAIT()  { digitalWrite(40, LOW); status_wait = false; }
#define host_clr_status_led_HLDA()    digitalWrite(41, LOW)

#define host_read_status_led_WAIT() status_wait
#define host_read_status_led_M1()   PORTB&0x20
#define host_read_status_led_INTE() PORTD&0x80

#define host_set_status_leds_READMEM()       PORTB |=  0x82
#define host_set_status_leds_READMEM_M1()    PORTB |=  0xA2; 
#define host_set_status_leds_READMEM_STACK() PORTB |=  0x86; 
#define host_set_status_leds_WRITEMEM()      PORTB &= ~0x82

uint16_t host_read_status_leds();


inline byte host_mega_read_switches(byte highlow)
{
  byte b = 0;
  ADCSRB = highlow ? 0x08 : 0x00; // MUX5
  ADMUX = 0x40; ADCSRA = 0xD4; while( ADCSRA&0x40 ); if( ADCH != 0 ) b |= 0x01;
  ADMUX = 0x41; ADCSRA = 0xD4; while( ADCSRA&0x40 ); if( ADCH != 0 ) b |= 0x02;
  ADMUX = 0x42; ADCSRA = 0xD4; while( ADCSRA&0x40 ); if( ADCH != 0 ) b |= 0x04;
  ADMUX = 0x43; ADCSRA = 0xD4; while( ADCSRA&0x40 ); if( ADCH != 0 ) b |= 0x08;
  ADMUX = 0x44; ADCSRA = 0xD4; while( ADCSRA&0x40 ); if( ADCH != 0 ) b |= 0x10;
  ADMUX = 0x45; ADCSRA = 0xD4; while( ADCSRA&0x40 ); if( ADCH != 0 ) b |= 0x20;
  ADMUX = 0x46; ADCSRA = 0xD4; while( ADCSRA&0x40 ); if( ADCH != 0 ) b |= 0x40;
  ADMUX = 0x47; ADCSRA = 0xD4; while( ADCSRA&0x40 ); if( ADCH != 0 ) b |= 0x80;
  return b;
}

#define host_read_sense_switches() host_mega_read_switches(true)

#define host_read_addr_switches() (host_mega_read_switches(true) * 256 | host_mega_read_switches(false))

bool host_read_function_switch(byte inputNum);


#endif
