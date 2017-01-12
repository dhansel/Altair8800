#ifndef ALTAIR8800_H
#define ALTAIR8800_H

#include <Arduino.h>

#define SW_RUN        0
#define SW_STOP       1
#define SW_STEP       2
#define SW_SLOW       3
#define SW_EXAMINE    4
#define SW_EXNEXT     5
#define SW_DEPOSIT    6
#define SW_DEPNEXT    7
#define SW_RESET      8
#define SW_CLR        9
#define SW_PROTECT   10
#define SW_UNPROTECT 11
#define SW_AUX1UP    12
#define SW_AUX1DOWN  13
#define SW_AUX2UP    14
#define SW_AUX2DOWN  15

#define ST_INT     0x0001
#define ST_WO      0x0002
#define ST_STACK   0x0004
#define ST_HLTA    0x0008
#define ST_OUT     0x0010
#define ST_M1      0x0020
#define ST_INP     0x0040
#define ST_MEMR    0x0080
#define ST_PROT    0x0100
#define ST_INTE    0x0200
#define ST_HLDA    0x0400
#define ST_WAIT    0x0800

extern word status_wait;
extern word status_inte;
extern byte reg_2SIO_ctrl, reg_2SIO_status;

byte altair_in(byte addr);
void altair_out(byte addr, byte val);
void altair_hlt();
bool altair_isreset();
void altair_wait_step();
void altair_set_outputs(uint16_t a, byte v);

#endif
