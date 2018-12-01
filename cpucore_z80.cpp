// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2018 David Hansel
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software Foundation,
// Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
// -----------------------------------------------------------------------------

#include "cpucore.h"
#include "timer.h"
#include "mem.h"
#include "numsys.h"
#include "disassembler.h"
#include "Altair8800.h"

#if USE_Z80 != 0

// additional Z80 status flags 
// (proper PS_UNUSED flag handling is necessary to pass the "zexall" test)
#define PS_ADDSUB      0x02
#define PS_UNUSED08    0x08
#define PS_OVERFLOW    PS_PARITY
#define PS_UNUSED20    0x20
#define PS_UNUSED      (PS_UNUSED08 | PS_UNUSED20)

extern union unionIXY
{
  struct { byte L, H; };
  uint16_t HL;
} regIX, regIY;


// additional Z80 registers
union unionAF regAF_;
union unionBC regBC_;
union unionDE regDE_;
union unionHL regHL_;
union unionIXY regIX, regIY;
byte regRL, regRH, regI;

byte     *registers[8]      = {&regB, &regC, &regD, &regE, &regH, &regL, NULL, &regA };
uint16_t *registers_wide[4] = {&regBC.BC, &regDE.DE, &regHL.HL, &regSP};

#define setCarryBit(v) if(v) regS |= PS_CARRY; else regS &= ~PS_CARRY

static const byte parity_table[256] = 
  {1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
   0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
   0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
   1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
   0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
   1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
   1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
   0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1};


inline uint16_t MEM_READ_WORD(uint16_t addr)
{
  if( host_read_status_led_WAIT() )
    {
      byte l, h;
      l = MEM_READ_STEP(addr);
      addr++;
      h = MEM_READ_STEP(addr);
      return l | (h * 256);
    }
  else
    {
      byte l, h;
#if USE_REAL_MREAD_TIMING>0
      l = MEM_READ(addr);
      addr++;
      h = MEM_READ(addr);
#else
      host_set_status_leds_READMEM();
      host_set_addr_leds(addr);
      l = MREAD(addr);
      addr++;
      host_set_addr_leds(addr);
      h = MREAD(addr);
      host_set_data_leds(h);
#endif
      return l | (h * 256);
    }
}


inline void MEM_WRITE_WORD(uint16_t addr, uint16_t v)
{
  if( host_read_status_led_WAIT() )
    {
      MEM_WRITE_STEP(addr, v & 255);
      addr++;
      MEM_WRITE_STEP(addr, v / 256);
    }
  else
    {
      byte b;
#if SHOW_BUS_OUTPUT>0
      b = v & 255;
      MEM_WRITE(addr, b);
      b = v / 256;
      addr++;
      MEM_WRITE(addr, b);
#else
      host_set_status_leds_WRITEMEM();
      host_set_data_leds(0xff);
      host_set_addr_leds(addr);
      b = v & 255;
      MWRITE(addr, b);
      addr++;
      host_set_addr_leds(addr);
      b = v / 256;
      MWRITE(addr, b);
#endif
    }
}


static void pushStackSlow(byte valueH, byte valueL)
{
  if( altair_isreset() )
    {
      host_set_status_led_STACK();
      regSP--;
      MEM_WRITE_STEP(regSP, valueH);
      if( altair_isreset() )
        {
          regSP--;
          MEM_WRITE_STEP(regSP, valueL);
        }
      host_clr_status_led_STACK();
    }
}

static void popStackSlow(byte *valueH, byte *valueL)
{
  if( altair_isreset() )
    {
      host_set_status_led_STACK();
      *valueL = MEM_READ_STEP(regSP);
      if( altair_isreset() )
        {
          regSP++;
          *valueH = MEM_READ_STEP(regSP);
          if( altair_isreset() ) regSP++;
        }
      
      host_clr_status_led_STACK();
    }
}


#if SHOW_BUS_OUTPUT>0

#define pushStack(valueH, valueL)               \
  if( !host_read_status_led_WAIT() )            \
    {                                           \
      host_set_status_led_STACK();              \
      regSP--;                                  \
      MEM_WRITE(regSP, valueH);                 \
      regSP--;                                  \
      MEM_WRITE(regSP, valueL);                 \
      host_clr_status_led_STACK();              \
    }                                           \
  else pushStackSlow(valueH, valueL);

#else

#define pushStack(valueH, valueL)               \
  if( !host_read_status_led_WAIT() )            \
    {                                           \
      host_set_status_led_STACK();              \
      host_set_status_leds_WRITEMEM();          \
      regSP--;                                  \
      host_set_addr_leds(regSP);                \
      MWRITE(regSP, valueH);                    \
      regSP--;                                  \
      host_set_addr_leds(regSP);                \
      MWRITE(regSP, valueL);                    \
      host_clr_status_led_STACK();              \
    }                                           \
  else pushStackSlow(valueH, valueL);

#endif

#if USE_REAL_MREAD_TIMING>0

#define popStack(valueH, valueL)                \
  if( !host_read_status_led_WAIT() )            \
    {                                           \
      host_set_status_led_STACK();              \
      valueL = MEM_READ(regSP);                 \
      regSP++;                                  \
      valueH = MEM_READ(regSP);                 \
      regSP++;                                  \
      host_clr_status_led_STACK();              \
    }                                           \
  else popStackSlow(&valueH, &valueL);

#else

#define popStack(valueH, valueL)                     \
  if( !host_read_status_led_WAIT() )                 \
    {                                                \
      host_set_status_leds_READMEM_STACK();          \
      host_set_addr_leds(regSP);                     \
      valueL = MREAD(regSP);                         \
      regSP++;                                       \
      host_set_addr_leds(regSP);                     \
      valueH = host_set_data_leds(MREAD(regSP));     \
      regSP++;                                       \
      host_clr_status_led_STACK();                   \
    }                                                \
  else popStackSlow(&valueH, &valueL);


#endif


inline void pushStackWord(uint16_t v)
{
  host_set_status_led_STACK();
  regSP -= 2;
  MEM_WRITE_WORD(regSP, v);
  host_clr_status_led_STACK();
}


inline uint16_t popStackWord()
{
  uint16_t v;
  host_set_status_led_STACK();
  v = MEM_READ_WORD(regSP);
  regSP += 2;
  host_clr_status_led_STACK();
  return v;
}


#define pushPC() pushStack(regPCU.H, regPCU.L)
#define popPC()  regPC = popStackWord()


inline void setStatusBitsLogic(byte value, byte f)
{
  regS = (value & (PS_SIGN|PS_UNUSED)) | f;
  if( value==0 ) regS |= PS_ZERO;
  if( parity_table[value] ) regS |= PS_PARITY;
}


inline byte inc(byte n)
{
  n++;
  regS = (regS & PS_CARRY) | (n & (PS_SIGN|PS_UNUSED));
  if( n == 0x00 ) regS |= PS_ZERO;
  if( (n&0x0F)==0 ) regS |= PS_HALFCARRY;
  if( n == 0x80 ) regS |= PS_OVERFLOW;
  return n;
}


inline byte dec(byte n)
{
  n--;
  regS = (regS & PS_CARRY) | (n & (PS_SIGN|PS_UNUSED)) | PS_ADDSUB;
  if( n == 0x00 ) regS |= PS_ZERO;
  if( (n&0x0F)==0x0F ) regS |= PS_HALFCARRY;
  if( n == 0x7F ) regS |= PS_OVERFLOW;
  return n;
}


inline byte add(byte opd1, byte opd2, byte c)
{
  byte res;

  uint16_t w = opd1 + opd2 + c;
  uint16_t b = opd1 ^ opd2 ^ w;

  res  = (byte) w;
  regS = (w & (PS_SIGN|PS_UNUSED)) | (b & PS_HALFCARRY);
  if( res == 0 ) regS |= PS_ZERO;
  if( w&0x100 ) regS |= PS_CARRY;
  if( (((b >> 1) ^ b) & 0x80) ) regS |= PS_OVERFLOW;

  return res;
}


inline byte sub(byte opd1, byte opd2, byte c)
{
  byte res;

  uint16_t w = opd1 - opd2 - c;
  uint16_t b = opd1 ^ opd2 ^ w;

  res  = (byte) w;
  regS = (w & (PS_SIGN|PS_UNUSED)) | (b & PS_HALFCARRY) | PS_ADDSUB;
  if( res==0 ) regS |= PS_ZERO;
  if( w&0x100 ) regS |= PS_CARRY;
  if( (((b >> 1) ^ b) & 0x80) ) regS |= PS_OVERFLOW;

  return res;
}


inline void cp(byte opd1, byte opd2)
{
  uint16_t r = opd1 - opd2;
  uint16_t b = opd1 ^ opd2 ^ r;
  regS = (r & PS_SIGN) | (opd2 & PS_UNUSED) | (b & PS_HALFCARRY) | PS_ADDSUB;
  if( r==0 ) regS |= PS_ZERO;
  if( r&0x100 ) regS |= PS_CARRY;
  if( (((b >> 1) ^ b) & 0x80) ) regS |= PS_OVERFLOW;
}


inline void cpnc(byte opd1, byte opd2)
{
  // similar to cp() but does not affect carry (used for cpi/cpir/cpdr)
  byte r = opd1 - opd2;
  byte b = opd1 ^ opd2 ^ r;
  regS = (regS & PS_CARRY) | (r & PS_SIGN) | (b & PS_HALFCARRY) | PS_ADDSUB;
  if( r==0 ) regS |= PS_ZERO;
  r -= (regS & PS_HALFCARRY)/PS_HALFCARRY;
  regS |= (r & PS_UNUSED08) | ((r & 0x02) ? PS_UNUSED20 : 0);
}


inline uint16_t addw(uint16_t opd1, uint16_t opd2, byte c)
{
  uint16_t res;

  uint32_t lw = (uint32_t) opd1 + (uint32_t) opd2 + (uint32_t) c;
  uint32_t b  = (opd1 ^ opd2 ^ lw) >> 8;

  res = lw;
  regS = ((lw>>8) & PS_UNUSED) | (b & PS_HALFCARRY);
  if( res & 0x8000 ) regS |= PS_SIGN;
  if( res==0 ) regS |= PS_ZERO;
  if( lw&0x10000 )  regS |= PS_CARRY;
  if( (((b >> 1) ^ b) & 0x80) ) regS |= PS_OVERFLOW;

  return res;
}


inline uint16_t subw(uint16_t opd1, uint16_t opd2, byte c)
{
  uint16_t res;

  uint32_t lw = (uint32_t) opd1 - (uint32_t) opd2 - (uint32_t) c;
  uint32_t b  = (opd1 ^ opd2 ^ lw) >> 8;

  res  = lw;
  regS = ((lw>>8) & PS_UNUSED) | (b & PS_HALFCARRY) | PS_ADDSUB;
  if( res & 0x8000 ) regS |= PS_SIGN;
  if( res==0 ) regS |= PS_ZERO;
  if( lw&0x10000 )  regS |= PS_CARRY;
  if( (((b >> 1) ^ b) & 0x80) ) regS |= PS_OVERFLOW;

  return res;
}


inline uint16_t addw2(uint16_t opd1, uint16_t opd2)
{
  // similar to addw() but only affects CARRY, HALFCARRY and ADDSUB
  uint16_t res;

  uint32_t lw = (uint32_t) opd1 + (uint32_t) opd2;
  uint32_t b  = (opd1 ^ opd2 ^ lw) >> 8;

  res  = lw;
  regS = (regS & ~(PS_CARRY|PS_HALFCARRY|PS_ADDSUB|PS_UNUSED)) | ((lw>>8) & PS_UNUSED) | (b & PS_HALFCARRY);
  if( lw&0x10000 ) regS |= PS_CARRY;

  return res;
}


// --------------------------------------------  load/exchange  --------------------------------------------------


static void cpu_lda() /* ld a, (NNNN) */
{
  uint16_t addr = MEM_READ_WORD(regPC);
  regA = MEM_READ(addr);
  regPC += 2;
  TIMER_ADD_CYCLES(13);
}

static void cpu_sta() /* ld (NNNN), a */
{
  uint16_t addr = MEM_READ_WORD(regPC);
  MEM_WRITE(addr, regA);
  regPC += 2;
  TIMER_ADD_CYCLES(13);
}

#define CPU_LDX(REG) /* ld a, <(bc),(de)> */  \
  static void cpu_ldx ## REG()                \
  {                                           \
    regA = MEM_READ(reg##REG.REG);            \
    TIMER_ADD_CYCLES(7);                      \
  }

#define CPU_STX(REG) /* ld <(bc),(de)>, a */  \
  static void cpu_stx ## REG()                \
  {                                           \
    MEM_WRITE(reg##REG.REG, regA);            \
    TIMER_ADD_CYCLES(7);                      \
  }

static void cpu_lhld() /* ld hl,(NNNN) */
{
  uint16_t addr = MEM_READ_WORD(regPC);
  regL = MEM_READ(addr);
  regH = MEM_READ(addr+1);
  regPC += 2;
  TIMER_ADD_CYCLES(16);
}

static void cpu_shld() /* ld (NNNN), hl */
{
  uint16_t addr = MEM_READ_WORD(regPC);
  MEM_WRITE(addr,   regL);
  MEM_WRITE(addr+1u, regH);
  regPC += 2;
  TIMER_ADD_CYCLES(16);
}

static void cpu_lxiSP() /* ld sp, NNNN */
{
  regSP = MEM_READ_WORD(regPC);
  regPC += 2;
  TIMER_ADD_CYCLES(10);
}
  
#define CPU_LXI(REGH,REGL) /* ld <BC,DE,HL>, NNNN */ \
  static void cpu_lxi ## REGH ## REGL()         \
  {                                             \
    reg ## REGL = MEM_READ(regPC);              \
    reg ## REGH = MEM_READ(regPC+1);            \
    regPC += 2;                                 \
    TIMER_ADD_CYCLES(10);                       \
  }

#define CPU_LDRR(REGTO,REGFROM) /* ld <b,c,d,e,h,l,a>, <b,c,d,e,h,l,a> */ \
  static void cpu_ld ## REGTO ## REGFROM ()     \
  {                                             \
    reg ## REGTO = reg ## REGFROM;              \
    TIMER_ADD_CYCLES(4);                        \
  }

#define CPU_LDMR(REGFROM) /* ld (hl), <b,c,d,e,h,l,a> */ \
  static void cpu_ldM ## REGFROM()              \
  {                                             \
    MEM_WRITE(regHL.HL, reg ## REGFROM);        \
    TIMER_ADD_CYCLES(7);                        \
  }

#define CPU_LDRM(REGTO) /* ld <b,c,d,e,h,l,a>, (hl) */ \
  static void cpu_ld ## REGTO ## M()            \
  {                                             \
    reg ## REGTO = MEM_READ(regHL.HL);          \
    TIMER_ADD_CYCLES(7);                        \
  }

#define CPU_LDRI(REGTO) /* ld <b,c,d,e,h,l,a>, NN */ \
  static void cpu_ld ## REGTO ## I()            \
  {                                             \
    reg ## REGTO = MEM_READ(regPC);             \
    regPC++;                                    \
    TIMER_ADD_CYCLES(7);                        \
  }

static void cpu_ldMI() /* ld (hl), NN */
{
  MEM_WRITE(regHL.HL, MEM_READ(regPC));
  regPC++;
  TIMER_ADD_CYCLES(10);
}

static void cpu_ldSP() /* ld sp, hl */
{
  regSP = regHL.HL;
  TIMER_ADD_CYCLES(6);
}


static void cpu_exsp() /* ex (sp), hl */
{
  byte b;
  b = MEM_READ(regSP+1u); MEM_WRITE(regSP+1u, regH); regH = b;
  b = MEM_READ(regSP);    MEM_WRITE(regSP,    regL); regL = b;
  TIMER_ADD_CYCLES(19);
}

static void cpu_exde() /* ex de, hl */
{
  byte b;
  b = regD; regD = regH; regH = b;
  b = regE; regE = regL; regL = b;
  TIMER_ADD_CYCLES(4);
}

static void cpu_exaf() /* ex af, af' */
{
  uint16_t w;
  w = regAF.AF; regAF.AF = regAF_.AF; regAF_.AF = w;
  TIMER_ADD_CYCLES(4);
}

static void cpu_exx() /* exx */
{
  uint16_t w;
  w = regBC.BC; regBC.BC = regBC_.BC; regBC_.BC = w;
  w = regDE.DE; regDE.DE = regDE_.DE; regDE_.DE = w;
  w = regHL.HL; regHL.HL = regHL_.HL; regHL_.HL = w;
  TIMER_ADD_CYCLES(4);
}


CPU_LDX(BC);
CPU_LDX(DE);

CPU_STX(BC);
CPU_STX(DE);

CPU_LXI(B,C);
CPU_LXI(D,E);
CPU_LXI(H,L);

CPU_LDRR(B, B);
CPU_LDRR(B, C);
CPU_LDRR(B, D);
CPU_LDRR(B, E);
CPU_LDRR(B, H);
CPU_LDRR(B, L);
CPU_LDRR(B, A);
CPU_LDRM(B);
CPU_LDMR(B);
CPU_LDRI(B);

CPU_LDRR(C, B);
CPU_LDRR(C, C);
CPU_LDRR(C, D);
CPU_LDRR(C, E);
CPU_LDRR(C, H);
CPU_LDRR(C, L);
CPU_LDRR(C, A);
CPU_LDRM(C);
CPU_LDMR(C);
CPU_LDRI(C);

CPU_LDRR(D, B);
CPU_LDRR(D, C);
CPU_LDRR(D, D);
CPU_LDRR(D, E);
CPU_LDRR(D, H);
CPU_LDRR(D, L);
CPU_LDRR(D, A);
CPU_LDRM(D);
CPU_LDMR(D);
CPU_LDRI(D);

CPU_LDRR(E, B);
CPU_LDRR(E, C);
CPU_LDRR(E, D);
CPU_LDRR(E, E);
CPU_LDRR(E, H);
CPU_LDRR(E, L);
CPU_LDRR(E, A);
CPU_LDRM(E);
CPU_LDMR(E);
CPU_LDRI(E);

CPU_LDRR(H, B);
CPU_LDRR(H, C);
CPU_LDRR(H, D);
CPU_LDRR(H, E);
CPU_LDRR(H, H);
CPU_LDRR(H, L);
CPU_LDRR(H, A);
CPU_LDRM(H);
CPU_LDMR(H);
CPU_LDRI(H);

CPU_LDRR(L, B);
CPU_LDRR(L, C);
CPU_LDRR(L, D);
CPU_LDRR(L, E);
CPU_LDRR(L, H);
CPU_LDRR(L, L);
CPU_LDRR(L, A);
CPU_LDRM(L);
CPU_LDMR(L);
CPU_LDRI(L);

CPU_LDRR(A, B);
CPU_LDRR(A, C);
CPU_LDRR(A, D);
CPU_LDRR(A, E);
CPU_LDRR(A, H);
CPU_LDRR(A, L);
CPU_LDRR(A, A);
CPU_LDRM(A);
CPU_LDMR(A);
CPU_LDRI(A);


// ------------------------------------------  arithmetic/logic/rotate  ------------------------------------------------


#define CPU_ADC(REG) /* adc a,<b,c,d,e,h,l,a> */   \
  static void cpu_adc ## REG ()                    \
  {                                                \
    regA = add(regA, reg ## REG, regS & PS_CARRY); \
    TIMER_ADD_CYCLES(4);                           \
  }

#define CPU_ADD(REG) /* add a,<b,c,d,e,h,l,a> */   \
  static void cpu_add ## REG ()                    \
  {                                                \
    regA = add(regA, reg ## REG, 0);               \
    TIMER_ADD_CYCLES(4);                           \
  }

#define CPU_SBC(REG) /* sbc a,<b,c,d,e,h,l,a> */   \
  static void cpu_sbc ## REG ()                    \
  {                                                \
    regA = sub(regA, reg ## REG, regS & PS_CARRY); \
    TIMER_ADD_CYCLES(4); \
  }

#define CPU_SUB(REG) /* sub a,<b,c,d,e,h,l,a> */   \
  static void cpu_sub ## REG ()                    \
  {                                                \
    regA = sub(regA, reg ## REG, 0);               \
    TIMER_ADD_CYCLES(4);                           \
  }

#define CPU_AND(REG) /* and <b,c,d,e,h,l,a> */     \
  static void cpu_and ## REG ()                    \
  {                                                \
    regA &= reg ## REG;                            \
    setStatusBitsLogic(regA, PS_HALFCARRY);        \
    TIMER_ADD_CYCLES(4);                           \
  } 

#define CPU_XOR(REG) /* xor <b,c,d,e,h,l,a> */     \
  static void cpu_xor ## REG ()                    \
  {                                                \
    regA ^= reg ## REG;                            \
    setStatusBitsLogic(regA, 0);                   \
    TIMER_ADD_CYCLES(4);                           \
  }

#define CPU_OR(REG) /* or <b,c,d,e,h,l,a> */       \
  static void cpu_or ## REG ()                     \
  {                                                \
    regA |= reg ## REG;                            \
    setStatusBitsLogic(regA, 0);                   \
    TIMER_ADD_CYCLES(4);                           \
  }

#define CPU_CP(REG) /* cp <b,c,d,e,h,l,a> */       \
  static void cpu_cp ## REG ()                     \
  {                                                \
    cp(regA, reg ## REG);                          \
    TIMER_ADD_CYCLES(4);                           \
  }

static void cpu_adcM() /* adc a,(hl) */
{
  regA = add(regA, MEM_READ(regHL.HL), regS & PS_CARRY);
  TIMER_ADD_CYCLES(7);
}

static void cpu_addM() /* add a,(hl) */
{
  regA = add(regA, MEM_READ(regHL.HL), 0);
  TIMER_ADD_CYCLES(7);
}

static void cpu_sbcM() /* sbc a,(hl) */
{
  regA = sub(regA, MEM_READ(regHL.HL), regS & PS_CARRY);
  TIMER_ADD_CYCLES(7);
}

static void cpu_subM() /* sub a,(hl) */
{
  regA = sub(regA, MEM_READ(regHL.HL), 0);
  TIMER_ADD_CYCLES(7);
}

static void cpu_andM() /* and (hl) */
{
  regA &= MEM_READ(regHL.HL);
  setStatusBitsLogic(regA, PS_HALFCARRY);
  TIMER_ADD_CYCLES(7);
}

static void cpu_xorM() /* xor (hl) */
{
  regA ^= MEM_READ(regHL.HL);
  setStatusBitsLogic(regA, 0);
  TIMER_ADD_CYCLES(7);
}

static void cpu_orM() /* or (hl) */
{
  regA |= MEM_READ(regHL.HL);
  setStatusBitsLogic(regA, 0);
  TIMER_ADD_CYCLES(7);
}

static void cpu_cpM() /* cp (hl) */
{
  cp(regA, MEM_READ(regHL.HL));
  TIMER_ADD_CYCLES(7);
}

static void cpu_add() /* add a,NN */
{
  regA = add(regA, MEM_READ(regPC), 0);
  regPC++;
  TIMER_ADD_CYCLES(7);
}

static void cpu_adc() /* adc a,NN */
{
  regA = add(regA, MEM_READ(regPC), regS & PS_CARRY);
  regPC++;
  TIMER_ADD_CYCLES(7);
}

static void cpu_sub() /* sub a,NN */
{
  regA = sub(regA, MEM_READ(regPC), 0);
  regPC++;
  TIMER_ADD_CYCLES(7);
}

static void cpu_sbc() /* sbc a,NN */
{
  regA = sub(regA, MEM_READ(regPC), regS & PS_CARRY);
  regPC++;
  TIMER_ADD_CYCLES(7);
}

static void cpu_and() /* and NN */
{
  regA &= MEM_READ(regPC);
  setStatusBitsLogic(regA, PS_HALFCARRY);
  regPC++;
  TIMER_ADD_CYCLES(7);
}

static void cpu_xor() /* xor NN */
{
  regA ^= MEM_READ(regPC);
  setStatusBitsLogic(regA, 0);
  regPC++;
  TIMER_ADD_CYCLES(7);
}

static void cpu_or() /* or NN */
{
  regA |= MEM_READ(regPC);
  setStatusBitsLogic(regA, 0);
  regPC++;
  TIMER_ADD_CYCLES(7);
}

static void cpu_cpi() /* cp NN */
{
  cp(regA, MEM_READ(regPC));
  regPC++;
  TIMER_ADD_CYCLES(7);
}


static void cpu_rlca() /* rlca */
{
  byte b = regA & 128;
  regA   = (regA * 2) | (b ? 1 : 0) ;
  regS   = (regS & ~(PS_HALFCARRY | PS_ADDSUB | PS_CARRY | PS_UNUSED)) | (regA & PS_UNUSED);
  if( b ) regS |= PS_CARRY;
  TIMER_ADD_CYCLES(4);
}

static void cpu_rrca() /* rrca */
{
  byte b = regA & 1;
  regA   = (regA / 2) | (b ? 128 : 0) ;
  regS   = (regS & ~(PS_HALFCARRY | PS_ADDSUB | PS_CARRY | PS_UNUSED)) | (regA & PS_UNUSED);
  if( b ) regS |= PS_CARRY;
  TIMER_ADD_CYCLES(4);
}

static void cpu_rla() /* rla */
{
  byte b = regA & 128;
  regA   = (regA * 2) | ((regS & PS_CARRY) ? 1 : 0) ;
  regS   = (regS & ~(PS_HALFCARRY | PS_ADDSUB | PS_CARRY | PS_UNUSED)) | (regA & PS_UNUSED);
  if( b ) regS |= PS_CARRY;
  TIMER_ADD_CYCLES(4);
}

static void cpu_rra() /* rra */
{
  byte b = regA & 1;
  regA   = (regA / 2) | ((regS & PS_CARRY) ? 128 : 0) ;
  regS   = (regS & ~(PS_HALFCARRY | PS_ADDSUB | PS_CARRY | PS_UNUSED)) | (regA & PS_UNUSED);
  if( b ) regS |= PS_CARRY;
  TIMER_ADD_CYCLES(4);
}


#define CPU_DAD(REG) /* add hl, <bc,de,hl> */   \
  static void cpu_dad ## REG()                  \
  {                                             \
    regHL.HL = addw2(regHL.HL, reg ## REG.REG); \
    TIMER_ADD_CYCLES(11);                       \
  }


static void cpu_dadSP() /* add hl, sp */
{
  regHL.HL = addw2(regHL.HL, regSP);
  TIMER_ADD_CYCLES(11);    
}


CPU_ADC(B);
CPU_ADC(C);
CPU_ADC(D);
CPU_ADC(E);
CPU_ADC(H);
CPU_ADC(L);
CPU_ADC(A);


CPU_ADD(B);
CPU_ADD(C);
CPU_ADD(D);
CPU_ADD(E);
CPU_ADD(H);
CPU_ADD(L);
CPU_ADD(A);

CPU_SBC(B);
CPU_SBC(C);
CPU_SBC(D);
CPU_SBC(E);
CPU_SBC(H);
CPU_SBC(L);
CPU_SBC(A);

CPU_SUB(B);
CPU_SUB(C);
CPU_SUB(D);
CPU_SUB(E);
CPU_SUB(H);
CPU_SUB(L);
CPU_SUB(A);

CPU_AND(B);
CPU_AND(C);
CPU_AND(D);
CPU_AND(E);
CPU_AND(H);
CPU_AND(L);
CPU_AND(A);

CPU_XOR(B);
CPU_XOR(C);
CPU_XOR(D);
CPU_XOR(E);
CPU_XOR(H);
CPU_XOR(L);
CPU_XOR(A);

CPU_OR(B);
CPU_OR(C);
CPU_OR(D);
CPU_OR(E);
CPU_OR(H);
CPU_OR(L);
CPU_OR(A);

CPU_CP(B);
CPU_CP(C);
CPU_CP(D);
CPU_CP(E);
CPU_CP(H);
CPU_CP(L);
CPU_CP(A);

CPU_DAD(BC);
CPU_DAD(DE);
CPU_DAD(HL);



// --------------------------------------------  increment/decrement  ---------------------------------------------------


#define CPU_DEC(REG) /* dec <b,c,d,e,h,l,a> */  \
static void cpu_dec ## REG ()                   \
  {                                             \
    reg ## REG = dec(reg ## REG);               \
    TIMER_ADD_CYCLES(4);                        \
  }

#define CPU_INC(REG)  /* inc <b,c,d,e,h,l,a> */ \
  static void cpu_inc ## REG()                  \
  {                                             \
    reg ## REG = inc(reg ## REG);               \
    TIMER_ADD_CYCLES(4);                        \
  }

static void cpu_decM() /* dec (hl) */ 
{
  byte res = dec(MEM_READ(regHL.HL));
  MEM_WRITE(regHL.HL, res);
  TIMER_ADD_CYCLES(11);
}

static void cpu_incM() /* inc (hl) */ 
{
  byte res = inc(MEM_READ(regHL.HL));
  MEM_WRITE(regHL.HL, res);
  TIMER_ADD_CYCLES(11);
}

#define CPU_DCX(REG) /* dec <bc,de,hl> */       \
  static void cpu_dcx ## REG ()                 \
  {                                             \
    reg##REG.REG--;                             \
    TIMER_ADD_CYCLES(6);                        \
  }

static void cpu_dcxSP() /* dec sp */
{
  regSP--;
  TIMER_ADD_CYCLES(6);
}

#define CPU_INX(REG) /* inc <bc,de,hl> */       \
  static void cpu_inx ## REG ()                 \
  {                                             \
    reg##REG.REG++;                             \
    TIMER_ADD_CYCLES(6);                        \
  }

static void cpu_inxSP() /* inc sp */
{
  regSP++;
  TIMER_ADD_CYCLES(6);
}


CPU_DEC(B);
CPU_DEC(C);
CPU_DEC(D);
CPU_DEC(E);
CPU_DEC(H);
CPU_DEC(L);
CPU_DEC(A);

CPU_INC(B);
CPU_INC(C);
CPU_INC(D);
CPU_INC(E);
CPU_INC(H);
CPU_INC(L);
CPU_INC(A);

CPU_DCX(BC);
CPU_DCX(DE);
CPU_DCX(HL);

CPU_INX(BC);
CPU_INX(DE);
CPU_INX(HL);


// --------------------------------------------  jump/call/return  ---------------------------------------------------


static void cpu_jp() /* jp NNNN */
{
  regPC = MEM_READ_WORD(regPC);
  TIMER_ADD_CYCLES(10);
}

static void cpu_jpnz() /* jp nz, NNNN */
{
  uint16_t addr = MEM_READ_WORD(regPC);
  if( !(regS & PS_ZERO) ) regPC = addr; else regPC += 2;
  TIMER_ADD_CYCLES(10);
}

static void cpu_jpz() /* jp z, NNNN */
{
  uint16_t addr = MEM_READ_WORD(regPC);
  if( (regS & PS_ZERO) ) regPC = addr; else regPC += 2;
  TIMER_ADD_CYCLES(10);
}

static void cpu_jpnc() /* jp nc, NNNN */
{
  uint16_t addr = MEM_READ_WORD(regPC);
  if( !(regS & PS_CARRY) ) regPC = addr; else regPC += 2;
  TIMER_ADD_CYCLES(10);
}

static void cpu_jpc() /* jp c, NNNN */
{
  uint16_t addr = MEM_READ_WORD(regPC);
  if( (regS & PS_CARRY) ) regPC = addr; else regPC += 2;
  TIMER_ADD_CYCLES(10);
}

static void cpu_jppo() /* jp po, NNNN */
{
  uint16_t addr = MEM_READ_WORD(regPC);
  if( !(regS & PS_PARITY) ) regPC = addr; else regPC += 2;
  TIMER_ADD_CYCLES(10);
}

static void cpu_jppe() /* jp pe, NNNN */
{
  uint16_t addr = MEM_READ_WORD(regPC);
  if( (regS & PS_PARITY) ) regPC = addr; else regPC += 2;
  TIMER_ADD_CYCLES(10);
}

static void cpu_jpp() /* jp p, NNNN */
{
  uint16_t addr = MEM_READ_WORD(regPC);
  if( !(regS & PS_SIGN) ) regPC = addr; else regPC += 2;
  TIMER_ADD_CYCLES(10);
}

static void cpu_jpm() /* jp m, NNNN */
{
  uint16_t addr = MEM_READ_WORD(regPC);
  if( (regS & PS_SIGN) ) regPC = addr; else regPC += 2;
  TIMER_ADD_CYCLES(10);
}

static void cpu_jpHL() /* jp (hl) */
{
  regPC = regHL.HL;
  TIMER_ADD_CYCLES(5);
}


static void cpu_jr() /* jr NN */
{
  int8_t offset = MEM_READ(regPC);
  regPC += offset+1;
  TIMER_ADD_CYCLES(12);
}

static void cpu_jrz() /* jr z, NN */
{
  int8_t offset = MEM_READ(regPC);
  if( (regS & PS_ZERO) ) 
    { regPC += offset+1; TIMER_ADD_CYCLES(12); }
  else 
    { regPC += 1; TIMER_ADD_CYCLES(7); }
}

static void cpu_jrnz() /* jr nz, NN */
{
  int8_t offset = MEM_READ(regPC);
  if( !(regS & PS_ZERO) ) 
    { regPC += offset+1; TIMER_ADD_CYCLES(12); }
  else 
    { regPC += 1; TIMER_ADD_CYCLES(7); }
}

static void cpu_jrc() /* jr c, NN */
{
  int8_t offset = MEM_READ(regPC);
  if( (regS & PS_CARRY) ) 
    { regPC += offset+1; TIMER_ADD_CYCLES(12); }
  else 
    { regPC += 1; TIMER_ADD_CYCLES(7); }
}

static void cpu_jrnc()  /* jr nc, NN */
{
  int8_t offset = MEM_READ(regPC);
  if( !(regS & PS_CARRY) ) 
    { regPC += offset+1; TIMER_ADD_CYCLES(12); }
  else 
    { regPC += 1; TIMER_ADD_CYCLES(7); }
}

static void cpu_djnz() /* djnz NN */
{
  int8_t offset = MEM_READ(regPC);
  if( --regB != 0 )
    { regPC += offset+1; TIMER_ADD_CYCLES(13); }
  else 
    { regPC += 1; TIMER_ADD_CYCLES(8); }
}


static void cpu_call() // call NNNN
{
  regPC += 2;
  pushPC();
  regPC = MEM_READ_WORD(regPC-2);
  TIMER_ADD_CYCLES(17);
}

static void cpu_cnz() /* call nz, NNNN */
{
  uint16_t addr = MEM_READ_WORD(regPC);
  regPC+=2; 
  if( !(regS & PS_ZERO) ) 
    { pushPC(); regPC = addr; TIMER_ADD_CYCLES(17); }
  else
    { TIMER_ADD_CYCLES(10); }
}

static void cpu_cz() /* call z, NNNN */
{
  uint16_t addr = MEM_READ_WORD(regPC);
  regPC+=2; 
  if( (regS & PS_ZERO) ) 
    { pushPC(); regPC = addr; TIMER_ADD_CYCLES(17); }
  else
    { TIMER_ADD_CYCLES(10); }
}

static void cpu_cnc() /* call nc, NNNN */
{
  uint16_t addr = MEM_READ_WORD(regPC);
  regPC+=2; 
  if( !(regS & PS_CARRY) ) 
    { pushPC(); regPC = addr; TIMER_ADD_CYCLES(17); }
  else
    { TIMER_ADD_CYCLES(10); }
}

static void cpu_cc() /* call c, NNNN */
{
  uint16_t addr = MEM_READ_WORD(regPC);
  regPC+=2; 
  if( (regS & PS_CARRY) ) 
    { pushPC(); regPC = addr; TIMER_ADD_CYCLES(17); }
  else
    { TIMER_ADD_CYCLES(10); }
}

static void cpu_cpo() /* call po, NNNN */
{
  uint16_t addr = MEM_READ_WORD(regPC);
  regPC+=2; 
  if( !(regS & PS_PARITY) ) 
    { pushPC(); regPC = addr; TIMER_ADD_CYCLES(17); }
  else
    { TIMER_ADD_CYCLES(10); }
}

static void cpu_cpe() /* call pe, NNNN */
{
  uint16_t addr = MEM_READ_WORD(regPC);
  regPC+=2; 
  if( (regS & PS_PARITY) ) 
    { pushPC(); regPC = addr; TIMER_ADD_CYCLES(17); }
  else
    { TIMER_ADD_CYCLES(10); }
}

static void cpu_cp() /* call p, NNNN */
{
  uint16_t addr = MEM_READ_WORD(regPC);
  regPC+=2; 
  if( !(regS & PS_SIGN) ) 
    { pushPC(); regPC = addr; TIMER_ADD_CYCLES(17); }
  else
    { TIMER_ADD_CYCLES(10); }
}

static void cpu_cm() /* call m, NNNN */
{
  uint16_t addr = MEM_READ_WORD(regPC);
  regPC+=2; 
  if( (regS & PS_SIGN) ) 
    { pushPC(); regPC = addr; TIMER_ADD_CYCLES(17); }
  else
    { TIMER_ADD_CYCLES(10); }
}


static void cpu_ret() /* ret */
{
  popPC();
  TIMER_ADD_CYCLES(10);
}

static void cpu_rnz() /* ret nz */
{
  if( !(regS & PS_ZERO) ) 
    { popPC(); TIMER_ADD_CYCLES(11); }
  else
    TIMER_ADD_CYCLES(5);
}

static void cpu_rz() /* ret z */
{
  if( (regS & PS_ZERO) ) 
    { popPC(); TIMER_ADD_CYCLES(11); }
  else
    TIMER_ADD_CYCLES(5);
}

static void cpu_rnc() /* ret nc */
{
  if( !(regS & PS_CARRY) ) 
    { popPC(); TIMER_ADD_CYCLES(11); }
  else
    TIMER_ADD_CYCLES(5);
}

static void cpu_rc() /* ret c */
{
  if( (regS & PS_CARRY) ) 
    { popPC(); TIMER_ADD_CYCLES(11); }
  else
    TIMER_ADD_CYCLES(5);
}

static void cpu_rpo() /* ret po */
{
  if( !(regS & PS_PARITY) ) 
    { popPC(); TIMER_ADD_CYCLES(11); }
  else
    TIMER_ADD_CYCLES(5);
}

static void cpu_rpe() /* ret pe */
{
  if( (regS & PS_PARITY) ) 
    { popPC(); TIMER_ADD_CYCLES(11); }
  else
    TIMER_ADD_CYCLES(5);
}

static void cpu_rp() /* ret p */
{
  if( !(regS & PS_SIGN) ) 
    { popPC(); TIMER_ADD_CYCLES(11); }
  else
    TIMER_ADD_CYCLES(5);
}

static void cpu_rm() /* ret m */
{
  if( (regS & PS_SIGN) ) 
    { popPC(); TIMER_ADD_CYCLES(11); }
  else
    TIMER_ADD_CYCLES(5);
}


#define CPU_RST(N) /* rst <00,08,10,18,20,28,30,38> */  \
  static void cpu_rst ## N()  \
  {                           \
    pushPC();                 \
    regPC = 0x00 ## N;        \
    TIMER_ADD_CYCLES(11);     \
  }

CPU_RST(00);
CPU_RST(08);
CPU_RST(10);
CPU_RST(18);
CPU_RST(20);
CPU_RST(28);
CPU_RST(30);
CPU_RST(38);


// --------------------------------------------  other  ---------------------------------------------------


static void cpu_cpl() /* cpl */
{
  regA = ~regA;
  regS = (regS & ~PS_UNUSED) | (regA & PS_UNUSED) | PS_HALFCARRY | PS_ADDSUB;
  TIMER_ADD_CYCLES(4);
}

static void cpu_ccf() /* ccf */
{
  regS &= ~(PS_HALFCARRY|PS_ADDSUB|PS_UNUSED);
  regS |= regA & PS_UNUSED;
  if( regS & PS_CARRY ) regS |= PS_HALFCARRY;
  regS ^= PS_CARRY;
  TIMER_ADD_CYCLES(4);
}

static void cpu_scf() /* scf */
{
  regS &= ~(PS_HALFCARRY | PS_ADDSUB | PS_UNUSED);
  regS |= PS_CARRY | (regA & PS_UNUSED);
  TIMER_ADD_CYCLES(4);
}

static void cpu_daa() /* daa */
{
  byte ldigit;
  uint16_t w;

  w = regA;
  ldigit = w & 0x0F;

  if( regS & PS_ADDSUB ) 
    {	
      // last operation was a subtract
      int hd = (regS & PS_CARRY) || w > 0x99;

      if( (regS & PS_HALFCARRY) || (ldigit > 9) ) 
        { 
          // adjust low digit
          if (ldigit > 5) regS &= ~PS_HALFCARRY;
          w -= 6;
          w &= 0xff;
        }

      if( hd ) w -= 0x160; // adjust high digit
    }
  else 
    {
      // last operation was an add
      if( (regS & PS_HALFCARRY) || (ldigit > 9) ) 
        { 
          // adjust low digit
          if( ldigit>9 ) regS |= PS_HALFCARRY; else regS &= ~PS_HALFCARRY;
          w += 6;
        }
      
      // adjust high digit
      if( (regS & PS_CARRY) || ((w & 0x1f0) > 0x90) ) w += 0x60;
    }

  regA = (byte) w;
  regS = regS & (PS_HALFCARRY|PS_CARRY|PS_ADDSUB) | (regA & (PS_SIGN|PS_UNUSED));
  if( regA==0 ) regS |= PS_ZERO;
  if( w & 0x100 ) regS |= PS_CARRY;
  if( parity_table[regA] ) regS |= PS_PARITY;

  TIMER_ADD_CYCLES(4);
}

static void cpu_di() /* di */
{
  altair_interrupt_disable();
  TIMER_ADD_CYCLES(4);
}

static void cpu_ei() /* ei */
{
  altair_interrupt_enable();
  TIMER_ADD_CYCLES(4);
}

static void cpu_hlt() /* hlt */
{
  altair_hlt();
  TIMER_ADD_CYCLES(4);
}

static void cpu_nop() /* nop */
{
  TIMER_ADD_CYCLES(4);
}

#define CPU_POP(REGH, REGL) /* pop <bc,de,hl,as> */ \
  static void cpu_pop ## REGH ## REGL()             \
  {                                                 \
    popStack(reg ## REGH, reg ## REGL);             \
    TIMER_ADD_CYCLES(10);                           \
  }

#define CPU_PSH(REGH, REGL) /* push <bc,de,hl,as>*/ \
  static void cpu_psh ## REGH ## REGL()             \
  {                                                 \
    pushStack(reg ## REGH, reg ## REGL);            \
    TIMER_ADD_CYCLES(10);                           \
  }

static void cpu_out() /* out NN */
{
  altair_out(MEM_READ(regPC), regA);
  TIMER_ADD_CYCLES(11);
  regPC++;
}

static void cpu_in() /* in NN */
{
  regA = altair_in(MEM_READ(regPC));
  TIMER_ADD_CYCLES(11);
  regPC++;
}


CPU_POP(B, C);
CPU_POP(D, E);
CPU_POP(H, L);
CPU_POP(A, S);

CPU_PSH(B, C);
CPU_PSH(D, E);
CPU_PSH(H, L);
CPU_PSH(A, S);


// --------------------------------------------  prefixes (0xCB, 0xDD, 0xED, 0xFD)  ---------------------------------------------------


static byte cpu_bitop(byte opcode, byte *v, byte mode) // Z80 bit operations
{
  byte b;

  switch( opcode & 0xC0 )
    {
    case 0x00:
      {
        byte carry = 0;

        // rotate operations
        switch( opcode & 0x38 )
          {
          case 0x00: // rlc
            b = *v & 0x80;
            *v = (*v * 2) | (b ? 0x01 : 0);
            if( b ) carry = PS_CARRY; 
            break;

          case 0x08: // rrc
            b = *v & 0x01;
            *v = (*v / 2) | (b ? 0x80 : 0);
            if( b ) carry = PS_CARRY; 
            break;

          case 0x10: // rl
            b = *v & 0x80;
            *v = (*v * 2) | ((regS & PS_CARRY) ? 0x01 : 0);
            if( b ) carry = PS_CARRY; 
            break;

          case 0x18: // rr
            b = *v & 0x01;
            *v = (*v / 2) | ((regS & PS_CARRY) ? 0x80 : 0);
            if( b ) carry = PS_CARRY; 
            break;

          case 0x20: // sla
            b = *v & 0x80;
            *v = (*v * 2);
            if( b ) carry = PS_CARRY; 
            break;

          case 0x28: // sra
            b = *v;
            *v = (*v / 2) | (b & 0x80);
            if( b & 0x01 ) carry = PS_CARRY; 
            break;

          case 0x30: // sll
            b = *v & 0x80;
            *v = (*v * 2) | 0x01;
            if( b ) carry = PS_CARRY; 
            break;

          case 0x38: // srl
            b = *v & 0x01;
            *v = (*v / 2);
            if( b ) carry = PS_CARRY; 
            break;
          }

        setStatusBitsLogic(*v, carry);
        break;
      }

    case 0x40:
      {
        // bit test operations
        byte n = (opcode & 0x38)/8, mask = 1 << n;
        if( *v & mask )
          regS = (regS & PS_CARRY) | PS_HALFCARRY | ((opcode & 0x38) == 0x38 ? PS_SIGN : 0);
        else
          regS = (regS & PS_CARRY) | PS_HALFCARRY | PS_ZERO | PS_PARITY;

        if( (opcode&7) != 6 ) regS |= *v & PS_UNUSED;
        break;
      }

    case 0x80:
      {
        // bit reset operations
        *v &= ~(1 << ((opcode & 0x38)/8));
        break;
      }

    case 0xC0:
      {
        // bit set operations
        *v |= 1 << ((opcode & 0x38)/8);
        break;
      }
    }

  return 8;
}


static void cpu_bit() // 0xCB prefix
{
  // Z80 BIT operations
  byte *reg, opcode = MEM_READ(regPC);
  byte cycles = 0;

  reg = registers[opcode & 0x07];
  if( reg==NULL ) 
    { 
      // read HL memory location
      byte m = MEM_READ(regHL.HL);
      TIMER_ADD_CYCLES(4);

      // perform operation
      cycles += cpu_bitop(opcode, &m, 1);

      // write back to memory (if required)
      if( opcode < 0x40 || opcode > 0x7F ) { MEM_WRITE(regHL.HL, m); cycles += 3; }
    }
  else
    cycles += cpu_bitop(opcode, reg, 0);
  
  regPC++;
  TIMER_ADD_CYCLES(cycles); 
}


static void cpu_ixiybit(union unionIXY *regIXY) // 0xDD/0xFD 0xCB prefix
{
  // IX/IY register bit instructions
  uint16_t addr;
  byte m, opcode, cycles = 0, *reg;

  // construct indexed address
  addr = regIXY->HL + ((int8_t) MEM_READ(regPC));
  regPC++;
  
  // read opcode
  opcode = MEM_READ(regPC);
  regPC++;

  // read value
  m = MEM_READ(addr);
  cycles += 12;

  // perform operation
  cycles += cpu_bitop(opcode, &m, 2);

  // write back to memory (if required)
  if( opcode < 0x40 || opcode > 0x7F ) { MEM_WRITE(addr, m); cycles += 3; }

  // copy to register (if required)
  reg = registers[opcode & 0x07];
  if( reg!=NULL ) *reg = m;

  TIMER_ADD_CYCLES(cycles);
}



static void cpu_ixiy(union unionIXY *regIXY) // 0xDD/0xFD prefix
{
  // Z80 IX/IY register instructions
  uint16_t addr, w;
  int8_t c;
  byte b, opcode = MEM_READ(regPC);
  regPC++;
  switch(opcode)
    {
    case 0x09: // add ix, bc
      regIXY->HL = addw2(regIXY->HL, regBC.BC);
      TIMER_ADD_CYCLES(15);
      break;

    case 0x19: // add ix, de
      regIXY->HL = addw2(regIXY->HL, regDE.DE);
      TIMER_ADD_CYCLES(15);
      break;

    case 0x21: // ld  ix, **
      regIXY->HL = MEM_READ_WORD(regPC);
      regPC += 2;
      TIMER_ADD_CYCLES(14);
      break;

    case 0x22: // ld (**),ix
      addr = MEM_READ_WORD(regPC);
      MEM_WRITE_WORD(addr, regIXY->HL);
      regPC += 2;
      TIMER_ADD_CYCLES(20);
      break;

    case 0x23: // inc ix
      regIXY->HL++;
      TIMER_ADD_CYCLES(10);
      break;

    case 0x24: // inc ixh
      regIXY->H = inc(regIXY->H);
      TIMER_ADD_CYCLES(8);
      break;

    case 0x25: // dec ixh
      regIXY->H = dec(regIXY->H);
      TIMER_ADD_CYCLES(8);
      break;
      
    case 0x26: // ld ixh,*
      regIXY->H = MEM_READ(regPC);
      regPC += 1;
      TIMER_ADD_CYCLES(11);
      break;

    case 0x29: // add ix, ix
      regIXY->HL = addw2(regIXY->HL, regIXY->HL);
      TIMER_ADD_CYCLES(15);
      break;

    case 0x2A: // ld ix, (**)
      addr = MEM_READ_WORD(regPC);
      regIXY->HL = MEM_READ_WORD(addr);
      regPC += 2;
      TIMER_ADD_CYCLES(20);
      break;

    case 0x2B: // dec ix
      regIXY->HL--;
      TIMER_ADD_CYCLES(10);
      break;

    case 0x2C: // inc ixl
      regIXY->L = inc(regIXY->L);
      TIMER_ADD_CYCLES(8);
      break;

    case 0x2D: // dec ixl
      regIXY->L = dec(regIXY->L);
      TIMER_ADD_CYCLES(8);
      break;
      
    case 0x2E: // ld ixl,*
      regIXY->L = MEM_READ(regPC);
      regPC += 1;
      TIMER_ADD_CYCLES(11);
      break;

    case 0x34: // inc (ix+*)
      c = MEM_READ(regPC);
      regPC += 1;
      addr = regIXY->HL + c;
      b = inc(MEM_READ(addr));
      MEM_WRITE(addr, b);
      TIMER_ADD_CYCLES(23);
      break;

    case 0x35: // dec (ix+*)
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      b = dec(MEM_READ(addr));
      MEM_WRITE(addr, b);
      TIMER_ADD_CYCLES(23);
      break;

    case 0x36: // ld (ix+*),*
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      b = MEM_READ(regPC);
      regPC++;
      MEM_WRITE(addr, b);
      TIMER_ADD_CYCLES(19);
      break;

    case 0x39: // add ix, sp
      regIXY->HL = addw2(regIXY->HL, regSP);
      TIMER_ADD_CYCLES(15);
      break;

    case 0x44: // ld b, ixh
      regB = regIXY->H;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x45: // ld b, ixl
      regB = regIXY->L;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x46: // ld b, (ix+*)
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      regB = MEM_READ(addr);
      TIMER_ADD_CYCLES(19);
      break;

    case 0x4C: // ld c, ixh
      regC = regIXY->H;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x4D: // ld c, ixl
      regC = regIXY->L;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x4E: // ld c, (ix+*)
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      regC = MEM_READ(addr);
      TIMER_ADD_CYCLES(19);
      break;

    case 0x54: // ld d, ixh
      regD = regIXY->H;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x55: // ld d, ixl
      regD = regIXY->L;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x56: // ld d, (ix+*)
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      regD = MEM_READ(addr);
      TIMER_ADD_CYCLES(19);
      break;

    case 0x5C: // ld e, ixh
      regE = regIXY->H;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x5D: // ld e, ixl
      regE = regIXY->L;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x5E: // ld e, (ix+*)
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      regE = MEM_READ(addr);
      TIMER_ADD_CYCLES(19);
      break;

    case 0x60: // ld ixh, b
      regIXY->H = regB;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x61: // ld ixh, c
      regIXY->H = regC;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x62: // ld ixh, d
      regIXY->H = regD;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x63: // ld ixh, e
      regIXY->H = regE;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x64: // ld ixh, ixh
      regIXY->H = regIXY->H;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x65: // ld ixh, ixl
      regIXY->H = regIXY->L;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x66: // ld h, (ix+*)
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      regH = MEM_READ(addr);
      TIMER_ADD_CYCLES(19);
      break;

    case 0x67: // ld ixh, a
      regIXY->H = regA;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x68: // ld ixl, b
      regIXY->L = regB;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x69: // ld ixl, c
      regIXY->L = regC;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x6A: // ld ixl, d
      regIXY->L = regD;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x6B: // ld ixl, e
      regIXY->L = regE;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x6C: // ld ixl, ixh
      regIXY->L = regIXY->H;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x6D: // ld ixl, ixl
      regIXY->L = regIXY->L;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x6E: // ld l, (ix+*)
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      regL = MEM_READ(addr);
      TIMER_ADD_CYCLES(19);
      break;

    case 0x6F: // ld ixl, a
      regIXY->L = regA;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x70: // ld (ix+*),b
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      MEM_WRITE(addr, regB);
      TIMER_ADD_CYCLES(19);
      break;

    case 0x71: // ld (ix+*),c
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      MEM_WRITE(addr, regC);
      TIMER_ADD_CYCLES(19);
      break;

    case 0x72: // ld (ix+*),d
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      MEM_WRITE(addr, regD);
      TIMER_ADD_CYCLES(19);
      break;

    case 0x73: // ld (ix+*),e
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      MEM_WRITE(addr, regE);
      TIMER_ADD_CYCLES(19);
      break;

    case 0x74: // ld (ix+*),h
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      MEM_WRITE(addr, regH);
      TIMER_ADD_CYCLES(19);
      break;

    case 0x75: // ld (ix+*),l
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      MEM_WRITE(addr, regL);
      TIMER_ADD_CYCLES(19);
      break;

    case 0x77: // ld (ix+*),a
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      MEM_WRITE(addr, regA);
      TIMER_ADD_CYCLES(19);
      break;

    case 0x7C: // ld a, ixh
      regA = regIXY->H;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x7D: // ld a, ixl
      regA = regIXY->L;
      TIMER_ADD_CYCLES(8);
      break;

    case 0x7E: // ld a,(ix+*)
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      regA = MEM_READ(addr);
      TIMER_ADD_CYCLES(19);
      break;

    case 0x84: // add a, ixh
      regA = add(regA, regIXY->H, 0);
      TIMER_ADD_CYCLES(8);
      break;

    case 0x85: // add a, ixl
      regA = add(regA, regIXY->L, 0);
      TIMER_ADD_CYCLES(8);
      break;

    case 0x86: // add a, (ix+*)
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      regA = add(regA, MEM_READ(addr), 0);
      TIMER_ADD_CYCLES(19);
      break;

    case 0x8C: // adc a, ixh
      regA = add(regA, regIXY->H, regS & PS_CARRY);
      TIMER_ADD_CYCLES(8);
      break;

    case 0x8D: // adc a, ixl
      regA = add(regA, regIXY->L, regS & PS_CARRY);
      TIMER_ADD_CYCLES(8);
      break;

    case 0x8E: // adc a, (ix+*)
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      regA = add(regA, MEM_READ(addr), regS & PS_CARRY);
      TIMER_ADD_CYCLES(19);
      break;

    case 0x94: // sub a, ixh
      regA = sub(regA, regIXY->H, 0);
      TIMER_ADD_CYCLES(8);
      break;

    case 0x95: // sub a, ixl
      regA = sub(regA, regIXY->L, 0);
      TIMER_ADD_CYCLES(8);
      break;

    case 0x96: // sub a, (ix+*)
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      regA = sub(regA, MEM_READ(addr), 0);
      TIMER_ADD_CYCLES(19);
      break;

    case 0x9C: // sbc a, ixh
      regA = sub(regA, regIXY->H, regS & PS_CARRY);
      TIMER_ADD_CYCLES(8);
      break;

    case 0x9D: // sbc a, ixl
      regA = sub(regA, regIXY->L, regS & PS_CARRY);
      TIMER_ADD_CYCLES(8);
      break;

    case 0x9E: // sbc a, (ix+*)
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      regA = sub(regA, MEM_READ(addr), regS & PS_CARRY);
      TIMER_ADD_CYCLES(19);
      break;

    case 0xA4: // and ixh
      regA &= regIXY->H;
      setStatusBitsLogic(regA, PS_HALFCARRY);
      TIMER_ADD_CYCLES(8);
      break;

    case 0xA5: // and ixl
      regA &= regIXY->L;
      setStatusBitsLogic(regA, PS_HALFCARRY);
      TIMER_ADD_CYCLES(8);
      break;

    case 0xA6: // and (ix+*)
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      regA &= MEM_READ(addr);
      setStatusBitsLogic(regA, PS_HALFCARRY);
      TIMER_ADD_CYCLES(19);
      break;

    case 0xAC: // xor ixh
      regA ^= regIXY->H;
      setStatusBitsLogic(regA, 0);
      TIMER_ADD_CYCLES(8);
      break;

    case 0xAD: // xor ixl
      regA ^= regIXY->L;
      setStatusBitsLogic(regA, 0);
      TIMER_ADD_CYCLES(8);
      break;

    case 0xAE: // xor (ix+*)
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      regA ^= MEM_READ(addr);
      setStatusBitsLogic(regA, 0);
      TIMER_ADD_CYCLES(19);
      break;

    case 0xB4: // or ixh
      regA |= regIXY->H;
      setStatusBitsLogic(regA, 0);
      TIMER_ADD_CYCLES(8);
      break;

    case 0xB5: // or ixl
      regA |= regIXY->L;
      setStatusBitsLogic(regA, 0);
      TIMER_ADD_CYCLES(8);
      break;

    case 0xB6: // or (ix+*)
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      regA |= MEM_READ(addr);
      setStatusBitsLogic(regA, 0);
      TIMER_ADD_CYCLES(19);
      break;

    case 0xBC: // cp a, ixh
      cp(regA, regIXY->H);
      TIMER_ADD_CYCLES(8);
      break;

    case 0xBD: // cp a, ixl
      cp(regA, regIXY->L);
      TIMER_ADD_CYCLES(8);
      break;

    case 0xBE: // cp a, (ix+*)
      c = MEM_READ(regPC);
      regPC++;
      addr = regIXY->HL + c;
      cp(regA, MEM_READ(addr));
      TIMER_ADD_CYCLES(19);
      break;

    case 0xCB: // bit operations
      cpu_ixiybit(regIXY);
      break;

    case 0xE1: // pop ix
      popStack(regIXY->H, regIXY->L);
      TIMER_ADD_CYCLES(14);
      break;

    case 0xE3: // ex (sp), ix
      w = MEM_READ_WORD(regSP);
      MEM_WRITE_WORD(regSP, regIXY->HL);
      regIXY->HL = w;
      TIMER_ADD_CYCLES(23);
      break;

    case 0xE5: // push ix
      pushStack(regIXY->H, regIXY->L);
      TIMER_ADD_CYCLES(15);
      break;

    case 0xE9: // jp (ix)
      regPC = regIXY->HL;
      TIMER_ADD_CYCLES(8);
      break;

    case 0xF9: // ld sp,ix
      regSP = regIXY->HL;
      TIMER_ADD_CYCLES(8);
      break;

    default:
      // ignore 0xDD/0xFD prefix
      CPU_EXEC(opcode);
      break;
    }
}


static void cpu_ix() // 0xDD prefix
{
  // Z80 IX register instructions
  cpu_ixiy(&regIX);
}


static void cpu_iy() // 0xFD prefix
{
  // Z80 IY register instructions
  cpu_ixiy(&regIY);
}


static void cpu_ext() // 0xED prefix
{
  // Z80 extended instructions
  uint16_t addr, w;
  byte b, *reg, cycles = 0, opcode = MEM_READ(regPC);
  regPC++;

  switch( opcode )
    {
    case 0x40:
    case 0x48:
    case 0x50:
    case 0x58:
    case 0x60:
    case 0x68:
    case 0x78: // in (b/c/d/e/h/l/a) (c)
      b = altair_in(regC);
      setStatusBitsLogic(b, regS & PS_CARRY);
      reg = registers[(opcode&0x38)/8];
      if( reg!=NULL ) *reg = b;
      TIMER_ADD_CYCLES(12);
      break;

    case 0x41:
    case 0x49:
    case 0x51:
    case 0x59:
    case 0x61:
    case 0x69:
    case 0x79: // out (c), (b/c/d/e/h/l/a)
      reg = registers[(opcode&0x38)/8];
      altair_out(regC, reg==NULL ? 0 : *reg);
      TIMER_ADD_CYCLES(12);
      break;

    case 0x42:
    case 0x52:
    case 0x62:
    case 0x72: // sbc (hl), (bc,de,hl,sp)
      regHL.HL = subw(regHL.HL, *registers_wide[(opcode&0x30)/16], regS & PS_CARRY);
      TIMER_ADD_CYCLES(15);
      break;

    case 0x4A:
    case 0x5A:
    case 0x6A:
    case 0x7A: // adc (hl), (bc,de,hl,sp)
      regHL.HL = addw(regHL.HL, *registers_wide[(opcode&0x30)/16], regS & PS_CARRY);
      TIMER_ADD_CYCLES(15);
      break;

    case 0x43:
    case 0x53:
    case 0x63:
    case 0x73: // ld (**), (bc,de,hl,sp)
      addr = MEM_READ_WORD(regPC);
      regPC += 2;
      w = *registers_wide[(opcode&0x30)/16];
      MEM_WRITE_WORD(addr, w);
      TIMER_ADD_CYCLES(20);
      break;

    case 0x4B:
    case 0x5B:
    case 0x6B:
    case 0x7B: // ld (bc,de,hl,sp), (**)
      addr = MEM_READ_WORD(regPC);
      regPC += 2;
      *registers_wide[(opcode&0x30)/16] = MEM_READ_WORD(addr);
      TIMER_ADD_CYCLES(20);
      break;
     
    case 0x44:
    case 0x4C:
    case 0x54:
    case 0x5C:
    case 0x64:
    case 0x6C:
    case 0x74:
    case 0x7C: // neg
      regA = sub(0, regA, 0);
      TIMER_ADD_CYCLES(8);
      break;
 
    case 0x45:
    case 0x4D: // reti
    case 0x55:
    case 0x5D:
    case 0x65:
    case 0x6D:
    case 0x75:
    case 0x7D: // retn
      popPC();
      TIMER_ADD_CYCLES(14);
      break;
 
    case 0x46:
    case 0x4E:
    case 0x56:
    case 0x5E:
    case 0x66:
    case 0x6E:
    case 0x76:
    case 0x7E: // im *
      TIMER_ADD_CYCLES(8);
      break;

    case 0x47: // ld i, a
      regI = regA;
      TIMER_ADD_CYCLES(9);
      break;

    case 0x57: // ld a, i
      regA = regI;
      regS = (regS & (PS_CARRY|PS_UNUSED)) | (regA & PS_SIGN);
      if( regA==0 ) regS |= PS_ZERO;
      if( altair_interrupt_enabled() ) regS |= PS_PARITY;
      TIMER_ADD_CYCLES(9);
      break;

    case 0x4F: // ld r, a
      regRH = regA;
      TIMER_ADD_CYCLES(9);
      break;

    case 0x5F: // ld a, r
      // we merge the instruction counter and the user-settable
      // bit 7 here instead of having to deal with a potential
      // overflow at each instruction fetch
      regA = (regRL & 0x7f) | (regRH & 0x80);
      regS = (regS & (PS_CARRY|PS_UNUSED)) | (regA & PS_SIGN);
      if( regA==0 ) regS |= PS_ZERO;
      if( altair_interrupt_enabled() ) regS |= PS_PARITY;
      TIMER_ADD_CYCLES(9);
      break;

    case 0x67: // rrd
      w = MEM_READ(regHL.HL) | (regA << 8);
      regA = (regA & 0xF0) | (w & 0x0F);
      w = (w >> 4) & 0xFF;
      MEM_WRITE(regHL.HL, (byte) w);
      setStatusBitsLogic(regA, regS & PS_CARRY);
      TIMER_ADD_CYCLES(18);
      break;

    case 0x6F: // rld
      w = MEM_READ(regHL.HL);
      w = (w << 4) | (regA & 0x0F);
      regA = regA & 0xF0 | (w >> 8);
      w &= 0xFF;
      MEM_WRITE(regHL.HL, (byte) w);
      setStatusBitsLogic(regA, regS & PS_CARRY);
      TIMER_ADD_CYCLES(18);
      break;

    case 0xA0: // ldi
    case 0xB0: // ldir
      b = MEM_READ(regHL.HL);
      MEM_WRITE(regDE.DE, b);
      regHL.HL++;
      regDE.DE++;
      regBC.BC--;
      regS &= ~(PS_HALFCARRY | PS_PARITY | PS_ADDSUB | PS_UNUSED);
      b += regA; regS |= (b & PS_UNUSED08) | ((b & 0x02) ? PS_UNUSED20 : 0);
      if( regBC.BC!=0 ) regS |= PS_PARITY;
      if( opcode==0xB0 && regBC.BC!=0 ) 
        { regPC -= 2; TIMER_ADD_CYCLES(21); }
      else
        TIMER_ADD_CYCLES(16);
      break;

    case 0xA8: // ldd
    case 0xB8: // lddr
      b = MEM_READ(regHL.HL);
      MEM_WRITE(regDE.DE, b);
      regHL.HL--;
      regDE.DE--;
      regBC.BC--;
      regS &= ~(PS_HALFCARRY | PS_PARITY | PS_ADDSUB | PS_UNUSED);
      b += regA; regS |= (b & PS_UNUSED08) | ((b & 0x02) ? PS_UNUSED20 : 0);
      if( regBC.BC!=0 ) regS |= PS_PARITY;
      if( opcode==0xB8 && regBC.BC!=0 ) 
        { regPC -=2; TIMER_ADD_CYCLES(21); }
      else
        TIMER_ADD_CYCLES(16);
      break;

    case 0xA1: // cpi
    case 0xB1: // cpir
      cpnc(regA, MEM_READ(regHL.HL));
      regHL.HL++;
      regBC.BC--;
      if( regBC.BC!=0 ) regS |= PS_PARITY;
      if( opcode==0xB1 && regBC.BC!=0 && (regS&PS_ZERO)==0 )
        { regPC -= 2; TIMER_ADD_CYCLES(21); }
      else
        TIMER_ADD_CYCLES(16);
      break;

    case 0xA9: // cpd
    case 0xB9: // cpdr
      cpnc(regA, MEM_READ(regHL.HL));
      regHL.HL--;
      regBC.BC--;
      if( regBC.BC!=0 ) regS |= PS_PARITY;
      if( opcode==0xB9 && regBC.BC!=0 && (regS&PS_ZERO)==0 ) 
        { regPC -= 2; TIMER_ADD_CYCLES(21); }
      else
        TIMER_ADD_CYCLES(16);
      break;

    case 0xA2: // ini
    case 0xB2: // inir
      b = altair_in(regC);
      MEM_WRITE(regHL.HL, b);
      regHL.HL++;
      regB--;
      regS = (regS & ~PS_ZERO) | PS_ADDSUB;
      if( regB==0 ) regS |= PS_ZERO;
      if( opcode==0xB2 && regB!=0 )
        { regPC -= 2; TIMER_ADD_CYCLES(21); }
      else
        TIMER_ADD_CYCLES(16);
      break;

    case 0xAA: // ind
    case 0xBA: // indr
      b = altair_in(regC);
      MEM_WRITE(regHL.HL, b);
      regHL.HL--;
      regB--;
      regS = (regS & ~PS_ZERO) | PS_ADDSUB;
      if( regB==0 ) regS |= PS_ZERO;
      if( opcode==0xBA && regB!=0 )
        { regPC -= 2; TIMER_ADD_CYCLES(21); }
      else
        TIMER_ADD_CYCLES(16);
      break;

    case 0xA3: // outi
    case 0xB3: // otir
      altair_out(regC, MEM_READ(regHL.HL));
      regHL.HL++;
      regB--;
      regS = (regS & ~PS_ZERO) | PS_ADDSUB;
      if( regB==0 ) regS |= PS_ZERO;
      if( opcode==0xB3 && regB!=0 )
        { regPC -= 2; TIMER_ADD_CYCLES(21); }
      else
        TIMER_ADD_CYCLES(16);
      break;

    case 0xAB: // outd
    case 0xBB: // otdr
      altair_out(regC, MEM_READ(regHL.HL));
      regHL.HL--;
      regB--;
      regS = (regS & ~PS_ZERO) | PS_ADDSUB;
      if( regB==0 ) regS |= PS_ZERO;
      if( opcode==0xBB && regB!=0 )
        { regPC -= 2; TIMER_ADD_CYCLES(21); }
      else
        TIMER_ADD_CYCLES(16);
      break;

    default:
      // ignore 0xED prefix
      CPU_EXEC(opcode);
      break;
    }
}


CPUFUN cpucore_z80_opcodes[256] = {
  cpu_nop,   cpu_lxiBC, cpu_stxBC, cpu_inxBC, cpu_incB,  cpu_decB,  cpu_ldBI,  cpu_rlca,	// 000-007 (0x00-0x07)
  cpu_exaf,  cpu_dadBC, cpu_ldxBC, cpu_dcxBC, cpu_incC,  cpu_decC,  cpu_ldCI,  cpu_rrca,	// 010-017 (0x08-0x0F)
  cpu_djnz,  cpu_lxiDE, cpu_stxDE, cpu_inxDE, cpu_incD,  cpu_decD,  cpu_ldDI,  cpu_rla,		// 020-027 (0x10-0x17)
  cpu_jr,    cpu_dadDE, cpu_ldxDE, cpu_dcxDE, cpu_incE,  cpu_decE,  cpu_ldEI,  cpu_rra,		// 030-037 (0x18-0x1F)
  cpu_jrnz,  cpu_lxiHL, cpu_shld,  cpu_inxHL, cpu_incH,  cpu_decH,  cpu_ldHI,  cpu_daa,		// 040-047 (0x20-0x27)
  cpu_jrz,   cpu_dadHL, cpu_lhld,  cpu_dcxHL, cpu_incL,  cpu_decL,  cpu_ldLI,  cpu_cpl,		// 050-057 (0x28-0x2F)
  cpu_jrnc,  cpu_lxiSP, cpu_sta,   cpu_inxSP, cpu_incM,  cpu_decM,  cpu_ldMI,  cpu_scf,		// 060-067 (0x30-0x37)
  cpu_jrc,   cpu_dadSP, cpu_lda,   cpu_dcxSP, cpu_incA,  cpu_decA,  cpu_ldAI,  cpu_ccf,		// 070-077 (0x38-0x3F)
  
  cpu_ldBB,  cpu_ldBC,  cpu_ldBD,  cpu_ldBE,  cpu_ldBH,  cpu_ldBL,  cpu_ldBM,  cpu_ldBA,	// 100-107 (0x40-0x47)
  cpu_ldCB,  cpu_ldCC,  cpu_ldCD,  cpu_ldCE,  cpu_ldCH,  cpu_ldCL,  cpu_ldCM,  cpu_ldCA,       	// 110-117 (0x48-0x4F)
  cpu_ldDB,  cpu_ldDC,  cpu_ldDD,  cpu_ldDE,  cpu_ldDH,  cpu_ldDL,  cpu_ldDM,  cpu_ldDA,	// 120-127 (0x50-0x57)
  cpu_ldEB,  cpu_ldEC,  cpu_ldED,  cpu_ldEE,  cpu_ldEH,  cpu_ldEL,  cpu_ldEM,  cpu_ldEA,	// 130-137 (0x58-0x5F)
  cpu_ldHB,  cpu_ldHC,  cpu_ldHD,  cpu_ldHE,  cpu_ldHH,  cpu_ldHL,  cpu_ldHM,  cpu_ldHA,	// 140-147 (0x60-0x67)
  cpu_ldLB,  cpu_ldLC,  cpu_ldLD,  cpu_ldLE,  cpu_ldLH,  cpu_ldLL,  cpu_ldLM,  cpu_ldLA,	// 150-157 (0x68-0x6F)
  cpu_ldMB,  cpu_ldMC,  cpu_ldMD,  cpu_ldME,  cpu_ldMH,  cpu_ldML,  cpu_hlt,   cpu_ldMA,	// 160-167 (0x70-0x77)
  cpu_ldAB,  cpu_ldAC,  cpu_ldAD,  cpu_ldAE,  cpu_ldAH,  cpu_ldAL,  cpu_ldAM,  cpu_ldAA,	// 170-177 (0x78-0x7F)
  
  cpu_addB,  cpu_addC,  cpu_addD,  cpu_addE,  cpu_addH,  cpu_addL,  cpu_addM,  cpu_addA,	// 200-207 (0x80-0x87)
  cpu_adcB,  cpu_adcC,  cpu_adcD,  cpu_adcE,  cpu_adcH,  cpu_adcL,  cpu_adcM,  cpu_adcA,	// 210-217 (0x88-0x8F)
  cpu_subB,  cpu_subC,  cpu_subD,  cpu_subE,  cpu_subH,  cpu_subL,  cpu_subM,  cpu_subA,	// 220-227 (0x90-0x97)
  cpu_sbcB,  cpu_sbcC,  cpu_sbcD,  cpu_sbcE,  cpu_sbcH,  cpu_sbcL,  cpu_sbcM,  cpu_sbcA,	// 230-237 (0x98-0x9F)
  cpu_andB,  cpu_andC,  cpu_andD,  cpu_andE,  cpu_andH,  cpu_andL,  cpu_andM,  cpu_andA,	// 240-247 (0xA0-0xA7)
  cpu_xorB,  cpu_xorC,  cpu_xorD,  cpu_xorE,  cpu_xorH,  cpu_xorL,  cpu_xorM,  cpu_xorA,	// 250-257 (0xA8-0xAF)
  cpu_orB,   cpu_orC,   cpu_orD,   cpu_orE,   cpu_orH,   cpu_orL,   cpu_orM,   cpu_orA,         // 260-267 (0xB0-0xB7)
  cpu_cpB,   cpu_cpC,   cpu_cpD,   cpu_cpE,   cpu_cpH,   cpu_cpL,   cpu_cpM,   cpu_cpA, 	// 270-277 (0xB8-0xBF)
  
  cpu_rnz,   cpu_popBC, cpu_jpnz,  cpu_jp,    cpu_cnz,   cpu_pshBC, cpu_add,   cpu_rst00,	// 300-307 (0xC0-0xC7)
  cpu_rz,    cpu_ret,   cpu_jpz,   cpu_bit,   cpu_cz,    cpu_call,  cpu_adc,   cpu_rst08,	// 310-317 (0xC8-0xCF)
  cpu_rnc,   cpu_popDE, cpu_jpnc,  cpu_out,   cpu_cnc,   cpu_pshDE, cpu_sub,   cpu_rst10,	// 320-327 (0xD0-0xD7)
  cpu_rc,    cpu_exx,   cpu_jpc,   cpu_in,    cpu_cc,    cpu_ix,    cpu_sbc,   cpu_rst18,	// 330-337 (0xD8-0xDF)
  cpu_rpo,   cpu_popHL, cpu_jppo,  cpu_exsp,  cpu_cpo,   cpu_pshHL, cpu_and,   cpu_rst20,	// 340-347 (0xE0-0xE7)
  cpu_rpe,   cpu_jpHL,  cpu_jppe,  cpu_exde,  cpu_cpe,   cpu_ext,   cpu_xor,   cpu_rst28,	// 350-357 (0xE8-0xEF)
  cpu_rp,    cpu_popAS, cpu_jpp,   cpu_di,    cpu_cp,    cpu_pshAS, cpu_or,    cpu_rst30,	// 360-367 (0xF0-0xF7)
  cpu_rm,    cpu_ldSP,  cpu_jpm,   cpu_ei,    cpu_cm,    cpu_iy,    cpu_cpi,   cpu_rst38	// 370-377 (0xF8-0xFF)
};


static void cpu_print_status_register(byte s)
{
  if( s & PS_SIGN )     Serial.print('S'); else Serial.print('.');
  if( s & PS_ZERO )     Serial.print('Z'); else Serial.print('.');
  Serial.print('.');
  if( s & PS_HALFCARRY ) Serial.print('A'); else Serial.print('.');
  Serial.print('.');
  if( s & PS_PARITY )   Serial.print('P'); else Serial.print('.');
  Serial.print('.');
  if( s & PS_ADDSUB )   Serial.print('N'); else Serial.print('.');
  if( s & PS_CARRY )    Serial.print('C'); else Serial.print('.');
}


void cpucore_z80_print_registers()
{
  Serial.print(F("\r\n PC   = "));   numsys_print_word(regPC);
  Serial.print(F(" = ")); numsys_print_mem(regPC, 4, true); 
  Serial.print(F(" = ")); disassemble(Mem, regPC, false);
  Serial.print(F("\r\n SP   = ")); numsys_print_word(regSP);
  Serial.print(F(" = ")); numsys_print_mem(regSP, 8, true); 
  Serial.print(F("\r\n regA = ")); numsys_print_byte(regA);
  Serial.print(F(" regS = "));   numsys_print_byte(regS);
  Serial.print(F(" = "));  cpu_print_status_register(regS);
  Serial.print(F("\r\n regB = ")); numsys_print_byte(regB);
  Serial.print(F(" regC = "));   numsys_print_byte(regC);
  Serial.print(F(" regD = "));   numsys_print_byte(regD);
  Serial.print(F(" regE = "));   numsys_print_byte(regE);
  Serial.print(F(" regH = "));   numsys_print_byte(regH);
  Serial.print(F(" regL = "));   numsys_print_byte(regL);
  Serial.print(F("\r\n regA'= ")); numsys_print_byte(regAF_.A);
  Serial.print(F(" regS'= "));   numsys_print_byte(regAF_.F);
  Serial.print(F(" = "));  cpu_print_status_register(regAF_.F);
  Serial.print(F("\r\n regB'= ")); numsys_print_byte(regBC_.B);
  Serial.print(F(" regC'= "));   numsys_print_byte(regBC_.C);
  Serial.print(F(" regD'= "));   numsys_print_byte(regDE_.D);
  Serial.print(F(" regE'= "));   numsys_print_byte(regDE_.E);
  Serial.print(F(" regH'= "));   numsys_print_byte(regHL_.H);
  Serial.print(F(" regL'= "));   numsys_print_byte(regHL_.L);
  Serial.print(F("\r\n regXH= ")); numsys_print_byte(regIX.H);
  Serial.print(F(" regXL= ")); numsys_print_byte(regIX.L);
  Serial.print(F(" regYH= ")); numsys_print_byte(regIY.H);
  Serial.print(F(" regYL= ")); numsys_print_byte(regIY.L);
  Serial.print(F(" regR = ")); numsys_print_byte((regRL & 0x7f) | (regRH & 0x80));
  Serial.print(F(" regI = ")); numsys_print_byte(regI);
  Serial.println();
}

#endif
