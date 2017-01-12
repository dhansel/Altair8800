#include <Arduino.h>

#if defined(__AVR_ATmega2560__)

// not enough room in MEGA to store ASM examples (or PS2 assembler anyways)
#include "prog_examples_basic_mega.h"
const char * const asm_programs[] = {};
#define READ_EX_BYTE(pv,pi,pc) pgm_read_byte(((const char*) pgm_read_word(pv+(pi)))+(pc))

#else

#include "prog_examples_basic_due.h"
#include "prog_examples_asm.h"
#define READ_EX_BYTE(pv,pi,pc) ((byte) pv[pi][pc])

#endif

static byte     prog_idx = 0;
static uint16_t prog_ctr = 0;


bool prog_examples_read_start(byte idx)
{
  if( idx < sizeof(basic_programs)/sizeof(char *) ||
      idx >= 0x80 && idx < 0x80+(sizeof(asm_programs)/sizeof(char *)) )
    {
      prog_idx = idx;
      prog_ctr = 0;
      return true;
    }
  else
    return false;
}


bool prog_examples_read_next(byte *b)
{
  if( prog_idx < 0x80 )
    *b = READ_EX_BYTE(basic_programs, prog_idx, prog_ctr);
  else
    *b = READ_EX_BYTE(asm_programs, prog_idx-0x80, prog_ctr);

  if( *b>0 ) prog_ctr++;
  return *b != 0;
}
