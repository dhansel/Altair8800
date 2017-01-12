#include "Altair8800.h"
#include "mem.h"
#include "host.h"

word mem_have_rom_basic = false;

byte Mem[MEMSIZE];


byte MEM_READ_STEP(uint16_t a)
{
  if( altair_isreset() )
    {
      byte v = MREAD(a);
      host_set_status_leds_READMEM();
      altair_set_outputs(a, v);
      altair_wait_step();
      v = host_read_data_leds(); // CPU reads whatever is on the data bus at this point
      host_clr_status_led_MEMR();
      return v;
    }
  else
    return 0x00;
}


void MEM_WRITE_STEP(uint16_t a, byte v)
{ 
  if( altair_isreset() )
    {
      MWRITE(a, v);
      host_set_status_leds_WRITEMEM();
      altair_set_outputs(a, 0xff);
      altair_wait_step();
      host_clr_status_led_WO();
    }
}


#if USE_PROTECT>0

word protected_flag = 0;
byte protected_flags[32];

void mem_protect(uint16_t a)
{
  protected_flags[((a)>>8)/8] |= (1<<(((a)>>8)&0x07));
  protected_flag = 1;
}

void mem_unprotect(uint16_t a)
{
  byte i;
  protected_flags[((a)>>8)/8] &= ~(1<<(((a)>>8)&0x07));
  for(i=0; i<32 && protected_flags[i]==0; i++);
  protected_flag = i<32;
}


#endif


void mem_setup()
{
#if USE_PROTECT>0
  for(int i=0; i<32; i++) protected_flags[i]=0x00;
  protected_flag = 0;
#endif
}
