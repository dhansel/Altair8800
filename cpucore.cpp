// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
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
#include "Altair8800.h"

union unionBC regBC;
union unionDE regDE;
union unionHL regHL;
byte regS, regA;
uint16_t regSP;
union unionPC regPCU;

#define setCarryBit(v)     if(v) regS |= PS_CARRY;     else regS &= ~PS_CARRY
#define setHalfCarryBit(v) if(v) regS |= PS_HALFCARRY; else regS &= ~PS_HALFCARRY

static const byte halfCarryTableAdd[] = { 0, 0, 1, 0, 1, 0, 1, 1 };
static const byte halfCarryTableSub[] = { 1, 0, 0, 0, 1, 1, 1, 0 };
#define setHalfCarryBitAdd(opd1, opd2, res) setHalfCarryBit(halfCarryTableAdd[((((opd1) & 0x08) / 2) | (((opd2) & 0x08) / 4) | (((res) & 0x08) / 8)) & 0x07])
#define setHalfCarryBitSub(opd1, opd2, res) setHalfCarryBit(halfCarryTableSub[((((opd1) & 0x08) / 2) | (((opd2) & 0x08) / 4) | (((res) & 0x08) / 8)) & 0x07])

static const byte parity_table[256] = 
  {1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
   0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
   0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
   1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
   0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 
   1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
   1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 
   0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1};


inline void setStatusBits(byte value)
{
  byte b;

  b = regS & ~(PS_ZERO|PS_SIGN|PS_PARITY);
  if( parity_table[value] ) b |= PS_PARITY;
  if( value==0 ) b |= PS_ZERO;
  b |= (value & PS_SIGN);

  regS = b;
}


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
      host_set_status_leds_READMEM();
      host_set_addr_leds(addr);
      l = MREAD(addr);
      addr++;
      host_set_addr_leds(addr);
      h = MREAD(addr);
      host_set_data_leds(h);
      return l | (h * 256);
    }
}


inline uint16_t MEM_WRITE_WORD(uint16_t addr, uint16_t v)
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
      host_set_status_leds_WRITEMEM();
      host_set_data_leds(0xff);
      host_set_addr_leds(addr);
      b = v & 255;
      MWRITE(addr, b);
      addr++;
      host_set_addr_leds(addr);
      b = v / 256;
      MWRITE(addr, b);
    }
}


void pushStackSlow(byte valueH, byte valueL)
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

void popStackSlow(byte *valueH, byte *valueL)
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


#define pushPC() pushStack(regPCU.H, regPCU.L) //pushStackWord(regPC)
#define popPC()  regPC = popStackWord()



#define CPU_ADC(REG) \
  void cpu_ADC ## REG () \
  { \
    uint16_t w = regA + reg ## REG; \
    if(regS & PS_CARRY) w++; \
    setHalfCarryBitAdd(regA, reg ## REG, w); \
    setCarryBit(w&0x100); \
    setStatusBits((byte) w); \
    regA = (byte) w; \
    TIMER_ADD_CYCLES(4); \
  }

CPU_ADC(B);
CPU_ADC(C);
CPU_ADC(D);
CPU_ADC(E);
CPU_ADC(H);
CPU_ADC(L);
CPU_ADC(A);


#define CPU_ADD(REG) \
  void cpu_ADD ## REG () \
  { \
    uint16_t w = regA + reg ## REG; \
    setCarryBit(w&0x100); \
    setHalfCarryBitAdd(regA, reg ## REG, w); \
    setStatusBits((byte) w); \
    regA = (byte) w; \
    TIMER_ADD_CYCLES(4); \
  }

CPU_ADD(B);
CPU_ADD(C);
CPU_ADD(D);
CPU_ADD(E);
CPU_ADD(H);
CPU_ADD(L);
CPU_ADD(A);


#define CPU_SBB(REG) \
  void cpu_SBB ## REG () \
  { \
    uint16_t w = regA - reg ## REG; \
    if(regS & PS_CARRY) w--; \
    setHalfCarryBitSub(regA, reg ## REG, w); \
    setCarryBit(w&0x100); \
    setStatusBits((byte) w); \
    regA = (byte) w; \
    TIMER_ADD_CYCLES(4); \
  }

CPU_SBB(B);
CPU_SBB(C);
CPU_SBB(D);
CPU_SBB(E);
CPU_SBB(H);
CPU_SBB(L);
CPU_SBB(A);


#define CPU_SUB(REG) \
  void cpu_SUB ## REG () \
  { \
    uint16_t w = regA - reg ## REG; \
    setCarryBit(w&0x100); \
    setHalfCarryBitSub(regA, reg ## REG, w); \
    setStatusBits((byte) w); \
    regA = (byte) w; \
    TIMER_ADD_CYCLES(4); \
  }

CPU_SUB(B);
CPU_SUB(C);
CPU_SUB(D);
CPU_SUB(E);
CPU_SUB(H);
CPU_SUB(L);
CPU_SUB(A);


#define CPU_ANA(REG) \
  void cpu_ANA ## REG () \
  { \
    setHalfCarryBit(regA&0x08 | reg ## REG & 0x08); \
    regA &= reg ## REG; \
    setCarryBit(0); \
    setStatusBits(regA); \
    TIMER_ADD_CYCLES(4); \
  } 

CPU_ANA(B);
CPU_ANA(C);
CPU_ANA(D);
CPU_ANA(E);
CPU_ANA(H);
CPU_ANA(L);
CPU_ANA(A);


#define CPU_XRA(REG) \
  void cpu_XRA ## REG () \
  { \
    regA ^= reg ## REG; \
    setCarryBit(0); \
    setHalfCarryBit(0); \
    setStatusBits(regA); \
    TIMER_ADD_CYCLES(4); \
  }

CPU_XRA(B);
CPU_XRA(C);
CPU_XRA(D);
CPU_XRA(E);
CPU_XRA(H);
CPU_XRA(L);
CPU_XRA(A);


#define CPU_ORA(REG) \
  void cpu_ORA ## REG () \
  { \
    regA |= reg ## REG; \
    setCarryBit(0); \
    setHalfCarryBit(0); \
    setStatusBits(regA); \
    TIMER_ADD_CYCLES(4); \
  }

CPU_ORA(B);
CPU_ORA(C);
CPU_ORA(D);
CPU_ORA(E);
CPU_ORA(H);
CPU_ORA(L);
CPU_ORA(A);


void cpu_ADCM()
{
  byte opd2  = MEM_READ(regHL.HL);
  uint16_t w = regA + opd2;
  if(regS & PS_CARRY) w++; 
  setHalfCarryBitAdd(regA, opd2, w);
  setCarryBit(w&0x100);
  setStatusBits((byte) w);
  regA = (byte) w;
  TIMER_ADD_CYCLES(7);
}

void cpu_ADDM()
{
  byte opd2 = MEM_READ(regHL.HL);
  uint16_t w    = regA + opd2;
  setCarryBit(w&0x100);
  setHalfCarryBitAdd(regA, opd2, w);
  setStatusBits((byte) w);
  regA = (byte) w;
  TIMER_ADD_CYCLES(7);
}

void cpu_SBBM()
{
  byte opd2 = MEM_READ(regHL.HL);
  uint16_t w    = regA - opd2;
  if(regS & PS_CARRY) w--; 
  setHalfCarryBitSub(regA, opd2, w);
  setCarryBit(w&0x100);
  setStatusBits((byte) w);
  regA = (byte) w;
  TIMER_ADD_CYCLES(7);
}

void cpu_SUBM()
{
  byte opd2 = MEM_READ(regHL.HL);
  uint16_t w    = regA - opd2;
  setCarryBit(w&0x100);
  setHalfCarryBitSub(regA, opd2, w);
  setStatusBits((byte) w);
  regA = (byte) w;
  TIMER_ADD_CYCLES(7);
}

void cpu_ANAM()
{
  byte opd2 = MEM_READ(regHL.HL);
  setHalfCarryBit(regA&0x08 | opd2&0x08);
  regA &= opd2;
  setCarryBit(0);
  setStatusBits(regA);
  TIMER_ADD_CYCLES(7);
}

void cpu_XRAM()
{
  regA ^= MEM_READ(regHL.HL);
  setCarryBit(0);
  setHalfCarryBit(0);
  setStatusBits(regA);
  TIMER_ADD_CYCLES(7);
}

void cpu_ORAM()
{
  regA |= MEM_READ(regHL.HL);
  setCarryBit(0);
  setHalfCarryBit(0);
  setStatusBits(regA);
  TIMER_ADD_CYCLES(7);
}

void cpu_CALL()
{
  regPC += 2;
  pushPC();
  regPC = MEM_READ_WORD(regPC-2);
  TIMER_ADD_CYCLES(17);
}

#define CPU_CMP(REG) \
  void cpu_CMP ## REG () \
  { \
    uint16_t w = regA - reg ## REG; \
    setCarryBit(w&0x100); \
    setHalfCarryBitSub(regA, reg ## REG, w); \
    setStatusBits((byte) w); \
    TIMER_ADD_CYCLES(4); \
  }

CPU_CMP(B);
CPU_CMP(C);
CPU_CMP(D);
CPU_CMP(E);
CPU_CMP(H);
CPU_CMP(L);
CPU_CMP(A);


void cpu_CMPM()
{
  byte opd2 = MEM_READ(regHL.HL);
  uint16_t w    = regA - opd2;
  setCarryBit(w&0x100);
  setHalfCarryBitSub(regA, opd2, w);
  setStatusBits((byte) w);
  TIMER_ADD_CYCLES(7);
}

#define CPU_DCR(REG) \
void cpu_DCR ## REG () \
  { \
    byte res = reg ## REG - 1; \
    setHalfCarryBit((res & 0x0f)!=0x0f); \
    setStatusBits(res); \
    reg ## REG = res; \
    TIMER_ADD_CYCLES(4); \
  }

CPU_DCR(B);
CPU_DCR(C);
CPU_DCR(D);
CPU_DCR(E);
CPU_DCR(H);
CPU_DCR(L);
CPU_DCR(A);

void cpu_DCRM()
{
  byte res  = MEM_READ(regHL.HL) - 1;
  setHalfCarryBit((res & 0x0f)!=0x0f);
  setStatusBits(res);
  MEM_WRITE(regHL.HL, res);
  TIMER_ADD_CYCLES(7);
}

void cpu_ADI()
{
  byte opd2 = MEM_READ(regPC);
  uint16_t w    = regA + opd2;
  setCarryBit(w & 0x100);
  setHalfCarryBitAdd(regA, opd2, w);
  setStatusBits((byte) w);
  regA = (byte) w;
  regPC++;
  TIMER_ADD_CYCLES(7);
}

void cpu_ACI()
{
  byte opd2 = MEM_READ(regPC);
  uint16_t w    = regA + opd2;
  if(regS & PS_CARRY) w++;
  setHalfCarryBitAdd(regA, opd2, w);
  setCarryBit(w&0x100);
  setStatusBits((byte) w);
  regA = (byte) w;
  regPC++;
  TIMER_ADD_CYCLES(7);
}

void cpu_SUI()
{
  byte opd2 = MEM_READ(regPC);
  uint16_t w    = regA - opd2;
  setCarryBit(w&0x100);
  setHalfCarryBitSub(regA, opd2, w);
  setStatusBits((byte) w);
  regA = (byte) w;
  regPC++;
  TIMER_ADD_CYCLES(7);
}

void cpu_SBI()
{
  byte opd2 = MEM_READ(regPC);
  uint16_t w    = regA - opd2;
  if(regS & PS_CARRY) w--;
  setHalfCarryBitSub(regA, opd2, w);
  setCarryBit(w&0x100);
  setStatusBits((byte) w);
  regA = (byte) w;
  regPC++;
  TIMER_ADD_CYCLES(7);
}

void cpu_ANI()
{
  byte opd2 = MEM_READ(regPC);
  setHalfCarryBit(regA&0x08 | opd2&0x08);
  regA &= opd2;
  setCarryBit(0);
  setStatusBits(regA);
  regPC++;
  TIMER_ADD_CYCLES(7);
}

void cpu_XRI()
{
  regA ^= MEM_READ(regPC);
  setCarryBit(0);
  setHalfCarryBit(0);
  setStatusBits(regA);
  regPC++;
  TIMER_ADD_CYCLES(7);
}

void cpu_ORI()
{
  regA |= MEM_READ(regPC);
  setCarryBit(0);
  setHalfCarryBit(0);
  setStatusBits(regA);
  regPC++;
  TIMER_ADD_CYCLES(7);
}

void cpu_CPI()
{
  byte opd2  = MEM_READ(regPC);
  uint16_t w = regA - opd2;
  setCarryBit(w&0x100);
  setHalfCarryBitSub(regA, opd2, w);
  setStatusBits((byte) w);
  regPC++;
  TIMER_ADD_CYCLES(7);
}

void cpu_CMA()
{
  regA = ~regA;
  TIMER_ADD_CYCLES(4);
}

void cpu_CMC()
{
  regS ^= PS_CARRY;
  TIMER_ADD_CYCLES(4);
}

void cpu_DAA()
{
  byte b   = regA;
  byte adj = 0;
  if( (regS & PS_HALFCARRY) || (b & 0x0f) > 0x09 ) adj = 0x06;
  if( (regS & PS_CARRY)     || (b & 0xf0) > 0x90 || ((b & 0xf0) == 0x90 && (b & 0x0f) > 9) )
    { adj  |= 0x60; regS |= PS_CARRY; }

  regA = b + adj;
  setHalfCarryBitAdd(b, adj, regA);
  setStatusBits(regA);
  TIMER_ADD_CYCLES(4);
}



#define CPU_DAD(REG)            \
  void cpu_DAD ## REG()         \
  {                             \
    uint16_t w = reg##REG.REG;  \
    regHL.HL += w;              \
    setCarryBit(regHL.HL < w);  \
    TIMER_ADD_CYCLES(10);        \
  }

CPU_DAD(BC);
CPU_DAD(DE);
CPU_DAD(HL);


void cpu_DADS()
{
  regHL.HL += regSP;
  setCarryBit(regHL.HL < regSP);
  TIMER_ADD_CYCLES(10);
}


#define CPU_DCX(REG) \
  void cpu_DCX ## REG () \
  {                      \
    reg##REG.REG--;      \
    TIMER_ADD_CYCLES(5);  \
  }

CPU_DCX(BC);
CPU_DCX(DE);
CPU_DCX(HL);

void cpu_DCXSP()
{
  regSP--;
  TIMER_ADD_CYCLES(5);
}

void cpu_DI()
{
  altair_interrupt_disable();
  TIMER_ADD_CYCLES(4);
}

void cpu_EI()
{
  altair_interrupt_enable();
  TIMER_ADD_CYCLES(4);
}

void cpu_HLT()
{
  altair_hlt();
  TIMER_ADD_CYCLES(7);
}

#define CPU_INR(REG) \
  void cpu_INR ## REG() \
  { \
    byte res = reg ## REG + 1; \
    setHalfCarryBit((res&0x0f)==0); \
    setStatusBits(res); \
    reg ## REG = res; \
    TIMER_ADD_CYCLES(5); \
  }

CPU_INR(B);
CPU_INR(C);
CPU_INR(D);
CPU_INR(E);
CPU_INR(H);
CPU_INR(L);
CPU_INR(A);


void cpu_INRM()
{
  byte res = MEM_READ(regHL.HL) + 1;
  setHalfCarryBit((res&0x0f)==0);
  setStatusBits(res);
  MEM_WRITE(regHL.HL, res);
  TIMER_ADD_CYCLES(10);
}

#define CPU_INX(REG) \
  void cpu_INX ## REG () \
  { \
    reg##REG.REG++; \
    TIMER_ADD_CYCLES(5); \
  }

CPU_INX(BC);
CPU_INX(DE);
CPU_INX(HL);


void cpu_INXSP()
{
  regSP++;
  TIMER_ADD_CYCLES(5);
}

void cpu_LDA()
{
  uint16_t addr = MEM_READ_WORD(regPC);
  regA = MEM_READ(addr);
  regPC += 2;
  TIMER_ADD_CYCLES(13);
}

#define CPU_LDX(REG) \
  void cpu_LDX ## REG() \
  { \
    regA = MEM_READ(reg##REG.REG); \
    TIMER_ADD_CYCLES(7); \
  }

CPU_LDX(BC);
CPU_LDX(DE);

void cpu_LHLD()
{
  uint16_t addr = MEM_READ_WORD(regPC);
  regL = MEM_READ(addr);
  regH = MEM_READ(addr+1);
  regPC += 2;
  TIMER_ADD_CYCLES(16);
}

void cpu_LXIS()
{
  regSP = MEM_READ_WORD(regPC);
  regPC += 2;
  TIMER_ADD_CYCLES(10);
}
  
#define CPU_LXI(REGH,REGL) \
  void cpu_LXI ## REGH ## REGL() \
  { \
    reg ## REGL = MEM_READ(regPC); \
    reg ## REGH = MEM_READ(regPC+1); \
    regPC += 2; \
    TIMER_ADD_CYCLES(10); \
  }

CPU_LXI(B,C);
CPU_LXI(D,E);
CPU_LXI(H,L);


#define CPU_MVRR(REGTO,REGFROM)                 \
  void cpu_MV ## REGTO ## REGFROM ()            \
  {                                             \
    reg ## REGTO = reg ## REGFROM;              \
    TIMER_ADD_CYCLES(5);                         \
  }

#define CPU_MVMR(REGFROM)                       \
  void cpu_MVM ## REGFROM()                     \
  {                                             \
    MEM_WRITE(regHL.HL, reg ## REGFROM);        \
    TIMER_ADD_CYCLES(7);                         \
  }

#define CPU_MVRM(REGTO)                         \
  void cpu_MV ## REGTO ## M()                   \
  {                                             \
    reg ## REGTO = MEM_READ(regHL.HL);          \
    TIMER_ADD_CYCLES(7);                         \
  }

#define CPU_MVRI(REGTO)                         \
  void cpu_MV ## REGTO ## I()                   \
  {                                             \
    reg ## REGTO = MEM_READ(regPC);             \
    regPC++;                                    \
    TIMER_ADD_CYCLES(7);                         \
  }

CPU_MVRR(B, B);
CPU_MVRR(B, C);
CPU_MVRR(B, D);
CPU_MVRR(B, E);
CPU_MVRR(B, H);
CPU_MVRR(B, L);
CPU_MVRR(B, A);
CPU_MVRM(B);
CPU_MVMR(B);
CPU_MVRI(B);

CPU_MVRR(C, B);
CPU_MVRR(C, C);
CPU_MVRR(C, D);
CPU_MVRR(C, E);
CPU_MVRR(C, H);
CPU_MVRR(C, L);
CPU_MVRR(C, A);
CPU_MVRM(C);
CPU_MVMR(C);
CPU_MVRI(C);

CPU_MVRR(D, B);
CPU_MVRR(D, C);
CPU_MVRR(D, D);
CPU_MVRR(D, E);
CPU_MVRR(D, H);
CPU_MVRR(D, L);
CPU_MVRR(D, A);
CPU_MVRM(D);
CPU_MVMR(D);
CPU_MVRI(D);

CPU_MVRR(E, B);
CPU_MVRR(E, C);
CPU_MVRR(E, D);
CPU_MVRR(E, E);
CPU_MVRR(E, H);
CPU_MVRR(E, L);
CPU_MVRR(E, A);
CPU_MVRM(E);
CPU_MVMR(E);
CPU_MVRI(E);

CPU_MVRR(H, B);
CPU_MVRR(H, C);
CPU_MVRR(H, D);
CPU_MVRR(H, E);
CPU_MVRR(H, H);
CPU_MVRR(H, L);
CPU_MVRR(H, A);
CPU_MVRM(H);
CPU_MVMR(H);
CPU_MVRI(H);

CPU_MVRR(L, B);
CPU_MVRR(L, C);
CPU_MVRR(L, D);
CPU_MVRR(L, E);
CPU_MVRR(L, H);
CPU_MVRR(L, L);
CPU_MVRR(L, A);
CPU_MVRM(L);
CPU_MVMR(L);
CPU_MVRI(L);

CPU_MVRR(A, B);
CPU_MVRR(A, C);
CPU_MVRR(A, D);
CPU_MVRR(A, E);
CPU_MVRR(A, H);
CPU_MVRR(A, L);
CPU_MVRR(A, A);
CPU_MVRM(A);
CPU_MVMR(A);
CPU_MVRI(A);


void cpu_MVMI()
{
  // MVI dst, M 
  MEM_WRITE(regHL.HL, MEM_READ(regPC));
  regPC++;
  TIMER_ADD_CYCLES(10);
}


void cpu_NOP()
{
  TIMER_ADD_CYCLES(4);
}

void cpu_PCHL()
{
  regPC = regHL.HL;
  TIMER_ADD_CYCLES(5);
}


#define CPU_POP(REGH, REGL)  \
  void cpu_POP ## REGH ## REGL() \
  { \
    popStack(reg ## REGH, reg ## REGL); \
    TIMER_ADD_CYCLES(10); \
  }

CPU_POP(B, C);
CPU_POP(D, E);
CPU_POP(H, L);
CPU_POP(A, S);


#define CPU_PSH(REGH, REGL) \
  void cpu_PSH ## REGH ## REGL() \
  { \
    pushStack(reg ## REGH, reg ## REGL); \
    TIMER_ADD_CYCLES(11); \
  }

CPU_PSH(B, C);
CPU_PSH(D, E);
CPU_PSH(H, L);


void cpu_PSHAS()
{
  pushStack(regA, (regS & 0xD5) | 0x02); 
  TIMER_ADD_CYCLES(11);
}

void cpu_RLC()
{
  byte b = regA & 128;
  regA   = (regA * 2) | (b ? 1 : 0) ;
  setCarryBit(b);
  TIMER_ADD_CYCLES(4);
}

void cpu_RRC()
{
  byte b = regA & 1;
  regA   = (regA / 2) | (b ? 128 : 0) ;
  setCarryBit(b);
  TIMER_ADD_CYCLES(4);
}

void cpu_RAL()
{
  byte b = regA & 128;
  regA   = (regA * 2) | ((regS & PS_CARRY) ? 1 : 0) ;
  setCarryBit(b);
  TIMER_ADD_CYCLES(4);
}

void cpu_RAR()
{
  byte b = regA & 1;
  regA   = (regA / 2) | ((regS & PS_CARRY) ? 128 : 0) ;
  setCarryBit(b);
  TIMER_ADD_CYCLES(4);
}

void cpu_RET()
{
  popPC();
  TIMER_ADD_CYCLES(10);
}

#define CPU_RST(N) \
  void cpu_RST ## N() \
  { \
    pushPC(); \
    regPC = 0x00 ## N; \
    TIMER_ADD_CYCLES(11); \
  }

CPU_RST(00);
CPU_RST(08);
CPU_RST(10);
CPU_RST(18);
CPU_RST(20);
CPU_RST(28);
CPU_RST(30);
CPU_RST(38);

void cpu_RNZ()
{
  if( !(regS & PS_ZERO) ) 
    { popPC(); TIMER_ADD_CYCLES(11); }
  else
    TIMER_ADD_CYCLES(5);
}

void cpu_RZ()
{
  if( (regS & PS_ZERO) ) 
    { popPC(); TIMER_ADD_CYCLES(11); }
  else
    TIMER_ADD_CYCLES(5);
}

void cpu_RNC()
{
  if( !(regS & PS_CARRY) ) 
    { popPC(); TIMER_ADD_CYCLES(11); }
  else
    TIMER_ADD_CYCLES(5);
}

void cpu_RC()
{
  if( (regS & PS_CARRY) ) 
    { popPC(); TIMER_ADD_CYCLES(11); }
  else
    TIMER_ADD_CYCLES(5);
}

void cpu_RPO()
{
  if( !(regS & PS_PARITY) ) 
    { popPC(); TIMER_ADD_CYCLES(11); }
  else
    TIMER_ADD_CYCLES(5);
}

void cpu_RPE()
{
  if( (regS & PS_PARITY) ) 
    { popPC(); TIMER_ADD_CYCLES(11); }
  else
    TIMER_ADD_CYCLES(5);
}

void cpu_RP()
{
  if( !(regS & PS_SIGN) ) 
    { popPC(); TIMER_ADD_CYCLES(11); }
  else
    TIMER_ADD_CYCLES(5);
}

void cpu_RM()
{
  if( (regS & PS_SIGN) ) 
    { popPC(); TIMER_ADD_CYCLES(11); }
  else
    TIMER_ADD_CYCLES(5);
}

void cpu_JMP()
{
  regPC = MEM_READ_WORD(regPC);
  TIMER_ADD_CYCLES(10);
}

void cpu_JNZ()
{
  uint16_t addr = MEM_READ_WORD(regPC);
  if( !(regS & PS_ZERO) ) regPC = addr; else regPC += 2;
  TIMER_ADD_CYCLES(10);
}

void cpu_JZ()
{
  uint16_t addr = MEM_READ_WORD(regPC);
  if( (regS & PS_ZERO) ) regPC = addr; else regPC += 2;
  TIMER_ADD_CYCLES(10);
}

void cpu_JNC()
{
  uint16_t addr = MEM_READ_WORD(regPC);
  if( !(regS & PS_CARRY) ) regPC = addr; else regPC += 2;
  TIMER_ADD_CYCLES(10);
}

void cpu_JC()
{
  uint16_t addr = MEM_READ_WORD(regPC);
  if( (regS & PS_CARRY) ) regPC = addr; else regPC += 2;
  TIMER_ADD_CYCLES(10);
}

void cpu_JPO()
{
  uint16_t addr = MEM_READ_WORD(regPC);
  if( !(regS & PS_PARITY) ) regPC = addr; else regPC += 2;
  TIMER_ADD_CYCLES(10);
}

void cpu_JPE()
{
  uint16_t addr = MEM_READ_WORD(regPC);
  if( (regS & PS_PARITY) ) regPC = addr; else regPC += 2;
  TIMER_ADD_CYCLES(10);
}

void cpu_JP()
{
  uint16_t addr = MEM_READ_WORD(regPC);
  if( !(regS & PS_SIGN) ) regPC = addr; else regPC += 2;
  TIMER_ADD_CYCLES(10);
}

void cpu_JM()
{
  uint16_t addr = MEM_READ_WORD(regPC);
  if( (regS & PS_SIGN) ) regPC = addr; else regPC += 2;
  TIMER_ADD_CYCLES(10);
}

void cpu_CNZ()
{
  uint16_t addr = MEM_READ_WORD(regPC);
  regPC+=2; 
  if( !(regS & PS_ZERO) ) 
    { pushPC(); regPC = addr; TIMER_ADD_CYCLES(17); }
  else
    { TIMER_ADD_CYCLES(11); }
}

void cpu_CZ()
{
  uint16_t addr = MEM_READ_WORD(regPC);
  regPC+=2; 
  if( (regS & PS_ZERO) ) 
    { pushPC(); regPC = addr; TIMER_ADD_CYCLES(17); }
  else
    { TIMER_ADD_CYCLES(11); }
}

void cpu_CNC()
{
  uint16_t addr = MEM_READ_WORD(regPC);
  regPC+=2; 
  if( !(regS & PS_CARRY) ) 
    { pushPC(); regPC = addr; TIMER_ADD_CYCLES(17); }
  else
    { TIMER_ADD_CYCLES(11); }
}

void cpu_CC()
{
  uint16_t addr = MEM_READ_WORD(regPC);
  regPC+=2; 
  if( (regS & PS_CARRY) ) 
    { pushPC(); regPC = addr; TIMER_ADD_CYCLES(17); }
  else
    { TIMER_ADD_CYCLES(11); }
}

void cpu_CPO()
{
  uint16_t addr = MEM_READ_WORD(regPC);
  regPC+=2; 
  if( !(regS & PS_PARITY) ) 
    { pushPC(); regPC = addr; TIMER_ADD_CYCLES(17); }
  else
    { TIMER_ADD_CYCLES(11); }
}

void cpu_CPE()
{
  uint16_t addr = MEM_READ_WORD(regPC);
  regPC+=2; 
  if( (regS & PS_PARITY) ) 
    { pushPC(); regPC = addr; TIMER_ADD_CYCLES(17); }
  else
    { TIMER_ADD_CYCLES(11); }
}

void cpu_CP()
{
  uint16_t addr = MEM_READ_WORD(regPC);
  regPC+=2; 
  if( !(regS & PS_SIGN) ) 
    { pushPC(); regPC = addr; TIMER_ADD_CYCLES(17); }
  else
    { TIMER_ADD_CYCLES(11); }
}

void cpu_CM()
{
  uint16_t addr = MEM_READ_WORD(regPC);
  regPC+=2; 
  if( (regS & PS_SIGN) ) 
    { pushPC(); regPC = addr; TIMER_ADD_CYCLES(17); }
  else
    { TIMER_ADD_CYCLES(11); }
}

void cpu_SHLD()
{
  uint16_t addr = MEM_READ_WORD(regPC);
  MEM_WRITE(addr,   regL);
  MEM_WRITE(addr+1u, regH);
  regPC += 2;
  TIMER_ADD_CYCLES(16);
}

void cpu_SPHL()
{
  regSP = regHL.HL;
  TIMER_ADD_CYCLES(5);
}

void cpu_STA()
{
  uint16_t addr = MEM_READ_WORD(regPC);
  MEM_WRITE(addr, regA);
  regPC += 2;
  TIMER_ADD_CYCLES(13);
}

#define CPU_STX(REG) \
  void cpu_STX ## REG() \
  { \
    MEM_WRITE(reg##REG.REG, regA); \
    TIMER_ADD_CYCLES(7); \
  }

CPU_STX(BC);
CPU_STX(DE);


void cpu_STC()
{
  regS |= PS_CARRY;
  TIMER_ADD_CYCLES(4);
}

void cpu_XTHL()
{
  byte b;
  b = MEM_READ(regSP+1u); MEM_WRITE(regSP+1u, regH); regH = b;
  b = MEM_READ(regSP);    MEM_WRITE(regSP,    regL); regL = b;
  TIMER_ADD_CYCLES(18);
}

void cpu_XCHG()
{
  byte b;
  b = regD; regD = regH; regH = b;
  b = regE; regE = regL; regL = b;
  TIMER_ADD_CYCLES(5);
}

void cpu_OUT()
{
  altair_out(MEM_READ(regPC), regA);
  TIMER_ADD_CYCLES(10);
  regPC++;
}

void cpu_IN()
{
  regA = altair_in(MEM_READ(regPC));
  TIMER_ADD_CYCLES(10);
  regPC++;
}


CPUFUN cpu_opcodes[256] = {
  cpu_NOP,   cpu_LXIBC, cpu_STXBC, cpu_INXBC, cpu_INRB,  cpu_DCRB,  cpu_MVBI,  cpu_RLC,		// 000-007 (0x00-0x07)
  cpu_NOP,   cpu_DADBC, cpu_LDXBC, cpu_DCXBC, cpu_INRC,  cpu_DCRC,  cpu_MVCI,  cpu_RRC,		// 010-017 (0x08-0x0F)
  cpu_NOP,   cpu_LXIDE, cpu_STXDE, cpu_INXDE, cpu_INRD,  cpu_DCRD,  cpu_MVDI,  cpu_RAL,		// 020-027 (0x10-0x17)
  cpu_NOP,   cpu_DADDE, cpu_LDXDE, cpu_DCXDE, cpu_INRE,  cpu_DCRE,  cpu_MVEI,  cpu_RAR,		// 030-037 (0x18-0x1F)
  cpu_NOP,   cpu_LXIHL, cpu_SHLD,  cpu_INXHL, cpu_INRH,  cpu_DCRH,  cpu_MVHI,  cpu_DAA,		// 040-047 (0x20-0x27)
  cpu_NOP,   cpu_DADHL, cpu_LHLD,  cpu_DCXHL, cpu_INRL,  cpu_DCRL,  cpu_MVLI,  cpu_CMA,		// 050-057 (0x28-0x2F)
  cpu_NOP,   cpu_LXIS,  cpu_STA,   cpu_INXSP, cpu_INRM,  cpu_DCRM,  cpu_MVMI,  cpu_STC,		// 060-067 (0x30-0x37)
  cpu_NOP,   cpu_DADS,  cpu_LDA,   cpu_DCXSP, cpu_INRA,  cpu_DCRA,  cpu_MVAI,  cpu_CMC,		// 070-077 (0x38-0x3F)
  
  cpu_MVBB,  cpu_MVBC,  cpu_MVBD,  cpu_MVBE,  cpu_MVBH,  cpu_MVBL,  cpu_MVBM,  cpu_MVBA,	// 100-107 (0x40-0x47)
  cpu_MVCB,  cpu_MVCC,  cpu_MVCD,  cpu_MVCE,  cpu_MVCH,  cpu_MVCL,  cpu_MVCM,  cpu_MVCA,       	// 110-117 (0x48-0x4F)
  cpu_MVDB,  cpu_MVDC,  cpu_MVDD,  cpu_MVDE,  cpu_MVDH,  cpu_MVDL,  cpu_MVDM,  cpu_MVDA,	// 120-127 (0x50-0x57)
  cpu_MVEB,  cpu_MVEC,  cpu_MVED,  cpu_MVEE,  cpu_MVEH,  cpu_MVEL,  cpu_MVEM,  cpu_MVEA,	// 130-137 (0x58-0x5F)
  cpu_MVHB,  cpu_MVHC,  cpu_MVHD,  cpu_MVHE,  cpu_MVHH,  cpu_MVHL,  cpu_MVHM,  cpu_MVHA,	// 140-147 (0x60-0x67)
  cpu_MVLB,  cpu_MVLC,  cpu_MVLD,  cpu_MVLE,  cpu_MVLH,  cpu_MVLL,  cpu_MVLM,  cpu_MVLA,	// 150-157 (0x68-0x6F)
  cpu_MVMB,  cpu_MVMC,  cpu_MVMD,  cpu_MVME,  cpu_MVMH,  cpu_MVML,  cpu_HLT,   cpu_MVMA,	// 160-167 (0x70-0x77)
  cpu_MVAB,  cpu_MVAC,  cpu_MVAD,  cpu_MVAE,  cpu_MVAH,  cpu_MVAL,  cpu_MVAM,  cpu_MVAA,	// 170-177 (0x78-0x7F)
  
  cpu_ADDB,  cpu_ADDC,  cpu_ADDD,  cpu_ADDE,  cpu_ADDH,  cpu_ADDL,  cpu_ADDM,  cpu_ADDA,	// 200-207 (0x80-0x87)
  cpu_ADCB,  cpu_ADCC,  cpu_ADCD,  cpu_ADCE,  cpu_ADCH,  cpu_ADCL,  cpu_ADCM,  cpu_ADCA,	// 210-217 (0x88-0x8F)
  cpu_SUBB,  cpu_SUBC,  cpu_SUBD,  cpu_SUBE,  cpu_SUBH,  cpu_SUBL,  cpu_SUBM,  cpu_SUBA,	// 220-227 (0x90-0x97)
  cpu_SBBB,  cpu_SBBC,  cpu_SBBD,  cpu_SBBE,  cpu_SBBH,  cpu_SBBL,  cpu_SBBM,  cpu_SBBA,	// 230-237 (0x98-0x9F)
  cpu_ANAB,  cpu_ANAC,  cpu_ANAD,  cpu_ANAE,  cpu_ANAH,  cpu_ANAL,  cpu_ANAM,  cpu_ANAA,	// 240-247 (0xA0-0xA7)
  cpu_XRAB,  cpu_XRAC,  cpu_XRAD,  cpu_XRAE,  cpu_XRAH,  cpu_XRAL,  cpu_XRAM,  cpu_XRAA,	// 250-257 (0xA8-0xAF)
  cpu_ORAB,  cpu_ORAC,  cpu_ORAD,  cpu_ORAE,  cpu_ORAH,  cpu_ORAL,  cpu_ORAM,  cpu_ORAA,        // 260-267 (0xB0-0xB7)
  cpu_CMPB,  cpu_CMPC,  cpu_CMPD,  cpu_CMPE,  cpu_CMPH,  cpu_CMPL,  cpu_CMPM,  cpu_CMPA,	// 270-277 (0xB8-0xBF)
  
  cpu_RNZ,   cpu_POPBC, cpu_JNZ,   cpu_JMP,   cpu_CNZ,   cpu_PSHBC, cpu_ADI,   cpu_RST00,	// 300-307 (0xC0-0xC7)
  cpu_RZ,    cpu_RET,   cpu_JZ,    cpu_JMP,   cpu_CZ,    cpu_CALL,  cpu_ACI,   cpu_RST08,	// 310-317 (0xC8-0xCF)
  cpu_RNC,   cpu_POPDE, cpu_JNC,   cpu_OUT,   cpu_CNC,   cpu_PSHDE, cpu_SUI,   cpu_RST10,	// 320-327 (0xD0-0xD7)
  cpu_RC,    cpu_RET,   cpu_JC,    cpu_IN,    cpu_CC,    cpu_CALL,  cpu_SBI,   cpu_RST18,	// 330-337 (0xD8-0xDF)
  cpu_RPO,   cpu_POPHL, cpu_JPO,   cpu_XTHL,  cpu_CPO,   cpu_PSHHL, cpu_ANI,   cpu_RST20,	// 340-347 (0xE0-0xE7)
  cpu_RPE,   cpu_PCHL,  cpu_JPE,   cpu_XCHG,  cpu_CPE,   cpu_CALL,  cpu_XRI,   cpu_RST28,	// 350-357 (0xE8-0xEF)
  cpu_RP,    cpu_POPAS, cpu_JP,    cpu_DI,    cpu_CP,    cpu_PSHAS, cpu_ORI,   cpu_RST30,	// 360-367 (0xF0-0xF7)
  cpu_RM,    cpu_SPHL,  cpu_JM,    cpu_EI,    cpu_CM,    cpu_CALL,  cpu_CPI,   cpu_RST38	// 370-377 (0xF8-0xFF)
};
