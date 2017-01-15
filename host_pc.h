#ifndef HOST_PC_H
#define HOST_PC_H

#define MEMSIZE          0x10000 /* 64k minus */
#define HOST_STORAGESIZE 0x80000 /* 512k */
#define HOST_BUFFERSIZE  0x400   /* 1k */

#define PROF_DISPLAY_INTERVAL 50000000

// PC host is always standalone
#undef  STANDALONE
#define STANDALONE 1

#undef  MAX_BREAKPOINTS
#define MAX_BREAKPOINTS 10

extern byte data_leds;
extern uint16_t status_leds;
extern uint16_t addr_leds;
extern byte stop_request;

#define host_read_sense_switches()           0
#define host_read_addr_switches()            0
#define host_read_function_switch(switchNum) false

#define host_set_status_led_INT()     status_leds |= ST_INT
#define host_set_status_led_WO()      status_leds &= ~ST_WO
#define host_set_status_led_STACK()   status_leds |= ST_STACK
#define host_set_status_led_HLTA()    status_leds |= ST_HLTA
#define host_set_status_led_OUT()     status_leds |= ST_OUT
#define host_set_status_led_M1()      status_leds |= ST_M1
#define host_set_status_led_INP()     status_leds |= ST_INP
#define host_set_status_led_MEMR()    status_leds |= ST_MEMR
#define host_set_status_led_INTE()  { status_leds |= ST_INTE; status_inte = true; }
#define host_set_status_led_PROT()    status_leds |= ST_PROT
#define host_set_status_led_WAIT()  { status_leds |= ST_WAIT; status_wait = true; }
#define host_set_status_led_HLDA()    status_leds |= ST_HLDA

#define host_clr_status_led_INT()     status_leds &= ~ST_INT
#define host_clr_status_led_WO()      status_leds |=  ST_WO
#define host_clr_status_led_STACK()   status_leds &= ~ST_STACK
#define host_clr_status_led_HLTA()    status_leds &= ~ST_HLTA
#define host_clr_status_led_OUT()     status_leds &= ~ST_OUT
#define host_clr_status_led_M1()      status_leds &= ~ST_M1
#define host_clr_status_led_INP()     status_leds &= ~ST_INP
#define host_clr_status_led_MEMR()    status_leds &= ~ST_MEMR
#define host_clr_status_led_INTE()  { status_leds &= ~ST_INTE;  status_inte = false; }
#define host_clr_status_led_PROT()    status_leds &= ~ST_PROT
#define host_clr_status_led_WAIT()  { status_leds &= ~ST_WAIT; status_wait = false; }
#define host_clr_status_led_HLDA()    status_leds &= ~ST_HLDA

#define host_read_status_led_WAIT()   status_wait
#define host_read_status_led_M1()     (status_leds & ST_M1)
#define host_read_status_led_INTE()   status_inte

// reading from memory (MEMR on, WO on)
#define host_set_status_leds_READMEM()     status_leds |= ST_MEMR | ST_WO

// reading opcode from memory (MEMR on, M1 on, WO on)
#define host_set_status_leds_READMEM_M1()  status_leds |= ST_MEMR | ST_WO | ST_M1

// reading from stack (MEMR on, STACK on, WO on)
#define host_set_status_leds_READMEM_STACK()  status_leds |= ST_MEMR | ST_WO | ST_STACK

// writing to memory (MEMR off, WO off)
#define host_set_status_leds_WRITEMEM()    status_leds &= ~(ST_MEMR | ST_WO)

#define host_read_status_leds()  status_leds


#define host_set_addr_leds(v) addr_leds = v
#define host_read_addr_leds() addr_leds

#define host_set_data_leds(v) data_leds = v
#define host_read_data_leds() data_leds

#define host_stop_request_check()     stop_request
#define host_stop_request_clear()     stop_request = 0
#define host_stop_request_set(w)      stop_request = w

#define host_serial_irq_check()       (host_read_status_led_INTE() && (reg_2SIO_ctrl & 0x80) && (reg_2SIO_status==0) && Serial.available())
#define host_serial_irq_clear()       while(0)

void host_init();

#endif
