// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef HOST_DUE_H
#define HOST_DUE_H

#define MEMSIZE 0x10000

#define HOST_STORAGESIZE due_storagesize
#define HOST_BUFFERSIZE  0x100

#define HOST_PERFORMANCE_FACTOR 1.0

extern uint32_t due_storagesize;

// ------------------------------------------ serial interface

class SwitchSerialClass : public Stream
{
  public:
    SwitchSerialClass();
    virtual void begin(unsigned long);
    virtual void end();
    virtual int available(void);
    virtual int availableForWrite(void);
    virtual int peek(void);
    virtual int read(void);
    virtual void flush(void);
    virtual size_t write(uint8_t);
    using Print::write; // pull in write(str) and write(buf, size) from Print
    virtual operator bool();

    void    select(uint8_t n) { m_selected = n; }
    uint8_t getSelected() { return m_selected; }

 private:
    uint8_t m_selected;
};

extern SwitchSerialClass SwitchSerial;
#define Serial SwitchSerial

bool host_serial_available_for_write(byte iface);
void host_serial_write(byte iface, byte data);

// ------------------------------------------ switches


inline byte host_read_sense_switches()
{
  // SW8...15  => PIOA, bits 12-15,17-20 (negative logic)
  word w = ~REG_PIOA_PDSR;
  return ((w & 0xF000) / (1<<12)) | ((w & 0x1E0000) / (1<<13));
}

uint16_t host_read_addr_switches();


// ------------------------------------------ status LEDs

/* reading global variables is faster than reading back the i/o register
   => INTE and WAIT are read often so we keep their state in a global variable */

#define host_set_status_led_INT()     REG_PIOB_SODR = 1<<25
#define host_set_status_led_WO()      REG_PIOC_CODR = 1<<28
#define host_set_status_led_STACK()   REG_PIOC_SODR = 1<<26
#define host_set_status_led_HLTA()    REG_PIOC_SODR = 1<<25
#define host_set_status_led_OUT()     REG_PIOC_SODR = 1<<24
#define host_set_status_led_M1()      REG_PIOC_SODR = 1<<23
#define host_set_status_led_INP()     REG_PIOC_SODR = 1<<22
#define host_set_status_led_MEMR()    REG_PIOC_SODR = 1<<21
#define host_set_status_led_INTE()    REG_PIOD_SODR = 1<<8;
#define host_set_status_led_PROT()    REG_PIOB_SODR = 1<<27
#define host_set_status_led_WAIT()  { REG_PIOC_SODR = 1<<29; status_wait = true; }
#define host_set_status_led_HLDA()    REG_PIOB_SODR = 1<<26

#define host_clr_status_led_INT()     REG_PIOB_CODR = 1<<25
#define host_clr_status_led_WO()      REG_PIOC_SODR = 1<<28
#define host_clr_status_led_STACK()   REG_PIOC_CODR = 1<<26
#define host_clr_status_led_HLTA()    REG_PIOC_CODR = 1<<25
#define host_clr_status_led_OUT()     REG_PIOC_CODR = 1<<24
#define host_clr_status_led_M1()      REG_PIOC_CODR = 1<<23
#define host_clr_status_led_INP()     REG_PIOC_CODR = 1<<22
#define host_clr_status_led_MEMR()    REG_PIOC_CODR = 1<<21
#define host_clr_status_led_INTE()    REG_PIOD_CODR = 1<<8;
#define host_clr_status_led_PROT()    REG_PIOB_CODR = 1<<27
#define host_clr_status_led_WAIT()  { REG_PIOC_CODR = 1<<29; status_wait = false; }
#define host_clr_status_led_HLDA()    REG_PIOB_CODR = 1<<26

#define host_read_status_led_WAIT()   status_wait
#define host_read_status_led_M1()     (REG_PIOC_PDSR & (1<<23))
#define host_read_status_led_HLTA()   (REG_PIOC_PDSR & (1<<25))
#define host_read_status_led_INTE()   status_inte

// reading from memory (MEMR on, WO on)
#define host_set_status_leds_READMEM()        REG_PIOC_SODR = 0x10200000

// reading opcode from memory (MEMR on, M1 on, WO on)
#define host_set_status_leds_READMEM_M1()     REG_PIOC_SODR = 0x10A00000

// reading from stack (MEMR on, WO on, STACK on)
#define host_set_status_leds_READMEM_STACK()  REG_PIOC_SODR = 0x10200000

// writing to memory (MEMR off, WO off)
#define host_set_status_leds_WRITEMEM()       REG_PIOC_CODR = 0x10200000

uint16_t host_read_status_leds();


// ----------------------------------------------------- address bus


inline void host_set_addr_leds(uint16_t v)
{
  // A0..7  => 34, 35, ..., 41 (PIOC, bits 2-9)
  // A8..15 => 51, 50, ..., 44 (PIOC, bits 12-19)
  REG_PIOC_ODSR = (v & 0x00ff) * 4 + (v & 0xff00) * 16;
}

uint16_t host_read_addr_leds();


// ---------------------------------------------------- data bus


// D0..8 => 25,26,27,28,14,15,29,11  (PIOD, bits 0-7)
#define host_set_data_leds(v) REG_PIOD_ODSR = v

byte host_read_data_leds();


// ---------------------------------------------------- interrupts

// On the Due we are using real interrupts so nothing needs o be done here
#define host_check_interrupts() while(0)

#endif
