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

#include <Arduino.h>
#include "disassembler.h"
#include "numsys.h"
#include "mem.h"
#include "host.h"
#include "config.h"

#if USE_Z80 != 0

typedef byte (*DAFUN)(byte, byte *, uint16_t);

static bool print_bytes = true;


static void pRN(byte regname)
{
  switch(regname)
    {
    case 0: Serial.print('b'); break; 
    case 1: Serial.print('c'); break; 
    case 2: Serial.print('d'); break; 
    case 3: Serial.print('e'); break; 
    case 4: Serial.print('h'); break; 
    case 5: Serial.print('l'); break; 
    case 6: Serial.print(F("(hl)")); break; 
    case 7: Serial.print('a'); break; 
    default: Serial.print('?'); break; 
    }
}

static void pR2N(byte regname)
{
  switch(regname)
    {
    case 0: { Serial.print('b'); Serial.print('c'); break; }
    case 1: { Serial.print('d'); Serial.print('e'); break; }
    case 2: { Serial.print('h'); Serial.print('l'); break; }
    case 3: { Serial.print('s'); Serial.print('p'); break; }
    default: Serial.print('?'); break; 
    }
}

static void pC(byte cond)
{
  switch( cond )
    {
    case 0: { Serial.print('n'); Serial.print('z'); break; }
    case 1: { Serial.print('z'); break; }
    case 2: { Serial.print('n'); Serial.print('c'); break; }
    case 3: { Serial.print('c'); break; }
    case 4: { Serial.print('p'); Serial.print('o'); break; }
    case 5: { Serial.print('p'); Serial.print('e'); break; }
    case 6: { Serial.print('p'); break; }
    case 7: { Serial.print('m'); break; }
    }
}

static void pB(byte *Mem, uint16_t addr)
{
  numsys_print_byte(MREAD(addr));
}

static void pW(byte *Mem, uint16_t addr)
{
  numsys_print_word(MREAD(addr) | (MREAD(addr+1) << 8));
}

static void pBN(byte *Mem, uint16_t addr, byte n)
{
  if( !print_bytes ) return;
  byte bl = numsys_get_byte_length()+1;
  for(int i=0; i<n; i++) { Serial.print(' '); numsys_print_byte(MREAD(addr+i)); }
  for(int i=0; i<bl*(4-n); i++) Serial.print(' ');
  Serial.print(F(": "));
}


// --------------------------------------------  load/exchange  --------------------------------------------------


static byte da_LDA(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 3); Serial.print(F("ld   a,(")); pW(Mem, PC+1); Serial.print(')');
  return 3;
}

static byte da_ldx(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("ld   a,(")); pR2N((opcode & 0060)>>4); Serial.print(')');
  return 1;
}

static byte da_lhld(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 3); Serial.print(F("ld   hl,(")); pW(Mem, PC+1); Serial.print(')');
  return 3;
}

static byte da_lxi(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 3); Serial.print(F("ld   ")); pR2N((opcode & 0060)>>4); Serial.print(','); pW(Mem, PC+1);
  return 3;
}
  
static byte da_ldRR(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("ld   ")); pRN((opcode&0070)>>3); Serial.print(','); pRN(opcode&0007);
  return 1;
}

static byte da_ldRI(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 2); Serial.print(F("ld   ")); pRN((opcode&0070)>>3); Serial.print(','); pB(Mem, PC+1);
  return 2;
}

static byte da_ldRM(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("ld   ")); pRN((opcode&0070)>>3); Serial.print(F(",(hl)"));
  return 1;
}

static byte da_shld(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 3); Serial.print(F("ld   (")); pW(Mem, PC+1); Serial.print(F("),hl"));
  return 3;
}

static byte da_ldSP(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("ld   sp,hl"));
  return 1;
}

static byte da_STA(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 3); Serial.print(F("ld   (")); pW(Mem, PC+1); Serial.print(F("),a"));
  return 3;
}

static byte da_stx(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("ld   (")); pR2N((opcode&0060)>>4); Serial.print(F("),a"));
  return 1;
}

static byte da_exsp(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("ex   (sp),hl"));
  return 1;
}

static byte da_exde(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("ex   de,hl"));
  return 1;
}

static byte da_exx(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("exx"));
  return 1;
}

static byte da_exaf(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("ex   af,af'"));
  return 1;
}


// ------------------------------------------  arithmetic/logic/rotate  ------------------------------------------------


static byte da_adc(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("adc  ")); pRN(opcode&0007);
  return 1;
}

static byte da_add(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("add  ")); pRN(opcode&0007);
  return 1;
}

static byte da_sbc(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("sbc  ")); pRN(opcode&0007);
  return 1;
}

static byte da_sub(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("sub  ")); pRN(opcode&0007);
  return 1;
}

static byte da_and(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("and  ")); pRN(opcode&0007);
  return 1;
}

static byte da_xor(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("xor  ")); pRN(opcode&0007);
  return 1;
}

static byte da_or(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("or   ")); pRN(opcode&0007);
  return 1;
}

static byte da_cp(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("cp   ")); pRN(opcode&0007);
  return 1;
}

static byte da_addI(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 2); Serial.print(F("add  ")); pB(Mem, PC+1);
  return 2;
}

static byte da_adcI(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 2); Serial.print(F("adc  ")); pB(Mem, PC+1);
  return 2;
}

static byte da_subI(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 2); Serial.print(F("sub  ")); pB(Mem, PC+1);
  return 2;
}

static byte da_sbcI(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 2); Serial.print(F("sbc  ")); pB(Mem, PC+1);
  return 2;
}

static byte da_andI(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 2); Serial.print(F("and  ")); pB(Mem, PC+1);
  return 2;
}

static byte da_xorI(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 2); Serial.print(F("xor  ")); pB(Mem, PC+1);
  return 2;
}

static byte da_orI(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 2); Serial.print(F("or   ")); pB(Mem, PC+1);
  return 2;
}

static byte da_cpI(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 2); Serial.print(F("cp   ")); pB(Mem, PC+1);
  return 2;
}

static byte da_dad(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("add  hl,")); pR2N((opcode & 0060)>>4);
  return 1;
}

static byte da_rlca(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("rlca"));
  return 1;
}

static byte da_rrca(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("rrca"));
  return 1;
}

static byte da_rla(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("rla"));
  return 1;
}

static byte da_rra(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("rra"));
  return 1;
}


// --------------------------------------------  increment/decrement  ---------------------------------------------------


static byte da_dec(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("dec  ")); pRN((opcode&0070)>>3);
  return 1;
}

static byte da_dcx(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("dec  ")); pR2N((opcode & 0060)>>4);
  return 1;
}

static byte da_di(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("di"));
  return 1;
}

static byte da_ei(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("ei"));
  return 1;
}

static byte da_hlt(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("hlt"));
  return 1;
}

static byte da_inc(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("inc  ")); pRN((opcode&0070)>>3);
  return 1;
}

static byte da_inx(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("inc  ")); pR2N((opcode & 0060)>>4);
  return 1;
}


// --------------------------------------------  jump/call/return  ---------------------------------------------------


static byte da_jmp(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 3); Serial.print(F("jp   ")); pW(Mem, PC+1);
  return 3;
}

static byte da_jpC(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 3); Serial.print(F("jp   ")); pC((opcode&0x38)/8); Serial.print(','); pW(Mem, PC+1);
  return 3;
}

static byte da_jpHL(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("jp   (hl)"));
  return 1;
}

static byte da_jr(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 2); Serial.print(F("jr   ")); numsys_print_word(PC + 2 + (int8_t) (MREAD(PC+1)));
  return 2;
}

static byte da_jrC(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 2); Serial.print(F("jr   ")); pC((opcode&0x18)/8); Serial.print(','); numsys_print_word(PC + 2 + (int8_t) (MREAD(PC+1)));
  return 2;
}

static byte da_djnz(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 2); Serial.print(F("djnz ")); numsys_print_word(PC + 2 + (int8_t) (MREAD(PC+1)));
  return 2;
}

static byte da_call(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 3); Serial.print(F("call ")); pW(Mem, PC+1);
  return 3;
}

static byte da_callC(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 3); Serial.print(F("call ")); pC((opcode&0x38)/8); Serial.print(','); pW(Mem, PC+1);
  return 3;
}

static byte da_ret(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("ret"));
  return 1;
}

static byte da_retC(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("ret  ")); pC((opcode&0x38)/8);
  return 1;
}

static byte da_rst(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("rst ")); numsys_print_word(opcode & 0070);
  return 1;
}


// --------------------------------------------  other  ---------------------------------------------------


static byte da_nop(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("nop"));
  return 1;
}

static byte da_out(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 2); Serial.print(F("out  ")); pB(Mem, PC+1);
  return 2;
}

static byte da_in(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 2); Serial.print(F("in   ")); pB(Mem, PC+1);
  return 2;
}

static byte da_cpl(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("cpl"));
  return 1;
}

static byte da_scf(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("scf"));
  return 1;
}

static byte da_ccf(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("ccf"));
  return 1;
}

static byte da_daa(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("daa"));
  return 1;
}

static byte da_pop(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("pop  ")); pR2N((opcode&0060)>>4);
  return 1;
}

static byte da_popA(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("pop  af"));
  return 1;
}

static byte da_push(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("push ")); pR2N((opcode&0060)>>4);
  return 1;
}

static byte da_pusa(byte opcode, byte *Mem, uint16_t PC)
{
  pBN(Mem, PC, 1); Serial.print(F("push af"));
  return 1;
}


// --------------------------------------------  prefixes (0xCB, 0xDD, 0xED, 0xFD)  ---------------------------------------------------


static void printIXYOffset(char xy, char offset)
{
  Serial.print('(');
  Serial.print('i'); 
  Serial.print(xy);
  if( offset<0 ) 
    { Serial.print('-'); numsys_print_byte(-offset); }
  else
    { Serial.print('+'); numsys_print_byte(offset); }
  Serial.print(')');
}


static void da_bitop(byte opcode, char xy, char offset)
{
  if( (opcode & 0xC0)==0 )
    {
      byte carry = 0;
      
      // rotate operations
      switch( opcode & 0x38 )
        {
        case 0x00: Serial.print(F("rlc  ")); break;
        case 0x08: Serial.print(F("rrc  ")); break;
        case 0x10: Serial.print(F("rl   ")); break;
        case 0x18: Serial.print(F("rr   ")); break;
        case 0x20: Serial.print(F("sla  ")); break;
        case 0x28: Serial.print(F("sra  ")); break;
        case 0x30: Serial.print(F("sll  ")); break;
        case 0x38: Serial.print(F("srl  ")); break;
        }
      
      pRN(opcode & 0x07);
    }
  else
    {
      switch(opcode & 0xC0)
        {
        case 0x40: Serial.print(F("bit  ")); break;
        case 0x80: Serial.print(F("res  ")); break;
        case 0xC0: Serial.print(F("set  ")); break;
        }
    }

  Serial.print((opcode & 0x38)/8);
  if( xy!=0 ) { Serial.print(','); printIXYOffset(xy, offset); }
  if( xy==0 || ((opcode&0xC0)!=0x40 && (opcode&0x07)!=0x06) ) { Serial.print(','); pRN(opcode & 0x07); }
}


static byte da_ixiy(byte prefix, byte *Mem, uint16_t PC) // 0xDD/0xFD prefix
{
  // IX/IY register instructions (0xDD/0xFD prefix)
#define pr(s1, s2) {Serial.print(F(s1)); Serial.print('i'); Serial.print(xy); Serial.print(F(s2)); }
  byte opcode = MREAD(PC+1);
  char xy = prefix==0xDD ? 'x' : 'y';

  switch( opcode )
    {
    case 0x09: pBN(Mem, PC, 2); pr("add  ", ",bc"); return 2;
    case 0x19: pBN(Mem, PC, 2); pr("add  ", ",de"); return 2;
    case 0x21: pBN(Mem, PC, 4); pr("ld   ", ","); pW(Mem, PC+2); return 4;
    case 0x22: pBN(Mem, PC, 4); Serial.print(F("ld   (")); pW(Mem, PC+2); pr("),",""); return 4;
    case 0x23: pBN(Mem, PC, 2); pr("inc  ", ""); return 2; 
    case 0x24: pBN(Mem, PC, 2); pr("inc  ", "h"); return 2; 
    case 0x25: pBN(Mem, PC, 2); pr("dec  ", "h"); return 2; 
    case 0x26: pBN(Mem, PC, 3); pr("ld   ", "h,"); pB(Mem, PC+2); return 3;
    case 0x29: pBN(Mem, PC, 2); pr("add  ", ",i"); Serial.print(xy); return 2;
    case 0x2A: pBN(Mem, PC, 4); pr("ld   ", ",("); pW(Mem, PC+2); Serial.print(')'); return 4;
    case 0x2B: pBN(Mem, PC, 2); pr("dec  ", ""); return 2; 
    case 0x2C: pBN(Mem, PC, 2); pr("inc  ", "l"); return 2; 
    case 0x2D: pBN(Mem, PC, 2); pr("dec  ", "l"); return 2; 
    case 0x2E: pBN(Mem, PC, 3); pr("ld   ", "l,"); pB(Mem, PC+2); return 3;
    case 0x34: pBN(Mem, PC, 3); Serial.print(F("inc  ")); printIXYOffset(xy, MREAD(PC+2)); return 3;
    case 0x35: pBN(Mem, PC, 3); Serial.print(F("dec  ")); printIXYOffset(xy, MREAD(PC+2)); return 3;
    case 0x36: pBN(Mem, PC, 4); Serial.print(F("ld   ")); printIXYOffset(xy, MREAD(PC+2)); Serial.print(','); pB(Mem, PC+3); return 4;
    case 0x39: pBN(Mem, PC, 2); pr("add  ", ",sp"); return 2;
    case 0x44: pBN(Mem, PC, 2); pr("ld   b,", "h"); return 2;
    case 0x45: pBN(Mem, PC, 2); pr("ld   b,", "l"); return 2;
    case 0x46: pBN(Mem, PC, 3); Serial.print(F("ld   b,")); printIXYOffset(xy, MREAD(PC+2)); return 3;
    case 0x4C: pBN(Mem, PC, 2); pr("ld   c,", "h"); return 2;
    case 0x4D: pBN(Mem, PC, 2); pr("ld   c,", "l"); return 2;
    case 0x4E: pBN(Mem, PC, 3); Serial.print(F("ld   c,")); printIXYOffset(xy, MREAD(PC+2)); return 3;
    case 0x54: pBN(Mem, PC, 2); pr("ld   d,", "h"); return 2;
    case 0x55: pBN(Mem, PC, 2); pr("ld   d,", "l"); return 2;
    case 0x56: pBN(Mem, PC, 3); Serial.print(F("ld   d,")); printIXYOffset(xy, MREAD(PC+2)); return 3;
    case 0x5C: pBN(Mem, PC, 2); pr("ld   e,", "h"); return 2;
    case 0x5D: pBN(Mem, PC, 2); pr("ld   e,", "l"); return 2;
    case 0x5E: pBN(Mem, PC, 3); Serial.print(F("ld   e,")); printIXYOffset(xy, MREAD(PC+2)); return 3;
    case 0x60: pBN(Mem, PC, 2); pr("ld   ","h,b"); return 2;
    case 0x61: pBN(Mem, PC, 2); pr("ld   ","h,c"); return 2;
    case 0x62: pBN(Mem, PC, 2); pr("ld   ","h,d"); return 2;
    case 0x63: pBN(Mem, PC, 2); pr("ld   ","h,e"); return 2;
    case 0x64: pBN(Mem, PC, 2); pr("ld   ","h,i"); Serial.print(xy); Serial.print('h'); return 2;
    case 0x65: pBN(Mem, PC, 2); pr("ld   ","h,i"); Serial.print(xy); Serial.print('l'); return 2;
    case 0x66: pBN(Mem, PC, 3); Serial.print(F("ld   h,")); printIXYOffset(xy, MREAD(PC+2)); return 3;
    case 0x67: pBN(Mem, PC, 2); pr("ld   ","l,a"); return 2;
    case 0x68: pBN(Mem, PC, 2); pr("ld   ","l,b"); return 2;
    case 0x69: pBN(Mem, PC, 2); pr("ld   ","l,c"); return 2;
    case 0x6A: pBN(Mem, PC, 2); pr("ld   ","l,d"); return 2;
    case 0x6B: pBN(Mem, PC, 2); pr("ld   ","l,e"); return 2;
    case 0x6C: pBN(Mem, PC, 2); pr("ld   ","l,i"); Serial.print(xy); Serial.print('h'); return 2;
    case 0x6D: pBN(Mem, PC, 2); pr("ld   ","l,i"); Serial.print(xy); Serial.print('l'); return 2;
    case 0x6E: pBN(Mem, PC, 3); Serial.print(F("ld   l,")); printIXYOffset(xy, MREAD(PC+2)); return 3;
    case 0x6F: pBN(Mem, PC, 2); pr("ld   ","l,a"); return 2;
    case 0x70: pBN(Mem, PC, 3); Serial.print(F("ld   ")); printIXYOffset(xy, MREAD(PC+2)); Serial.print(F(",b")); return 3;
    case 0x71: pBN(Mem, PC, 3); Serial.print(F("ld   ")); printIXYOffset(xy, MREAD(PC+2)); Serial.print(F(",c")); return 3;
    case 0x72: pBN(Mem, PC, 3); Serial.print(F("ld   ")); printIXYOffset(xy, MREAD(PC+2)); Serial.print(F(",d")); return 3;
    case 0x73: pBN(Mem, PC, 3); Serial.print(F("ld   ")); printIXYOffset(xy, MREAD(PC+2)); Serial.print(F(",e")); return 3;
    case 0x74: pBN(Mem, PC, 3); Serial.print(F("ld   ")); printIXYOffset(xy, MREAD(PC+2)); Serial.print(F(",h")); return 3;
    case 0x75: pBN(Mem, PC, 3); Serial.print(F("ld   ")); printIXYOffset(xy, MREAD(PC+2)); Serial.print(F(",l")); return 3;
    case 0x77: pBN(Mem, PC, 3); Serial.print(F("ld   ")); printIXYOffset(xy, MREAD(PC+2)); Serial.print(F(",a")); return 3;
    case 0x7C: pBN(Mem, PC, 2); pr("ld   a,", "h"); return 2;
    case 0x7D: pBN(Mem, PC, 2); pr("ld   a,", "l"); return 2;
    case 0x7E: pBN(Mem, PC, 3); Serial.print(F("ld   a,")); printIXYOffset(xy, MREAD(PC+2)); return 3;
    case 0x84: pBN(Mem, PC, 2); pr("add  a,", "h"); return 2;
    case 0x85: pBN(Mem, PC, 2); pr("add  a,", "l"); return 2;
    case 0x86: pBN(Mem, PC, 3); Serial.print(F("add  a,")); printIXYOffset(xy, MREAD(PC+2)); return 3;
    case 0x8C: pBN(Mem, PC, 2); pr("adc  a,", "h"); return 2;
    case 0x8D: pBN(Mem, PC, 2); pr("adc  a,", "l"); return 2;
    case 0x8E: pBN(Mem, PC, 3); Serial.print(F("adc  a,")); printIXYOffset(xy, MREAD(PC+2)); return 3;
    case 0x94: pBN(Mem, PC, 2); pr("sub  a,", "h"); return 2;
    case 0x95: pBN(Mem, PC, 2); pr("sub  a,", "l"); return 2;
    case 0x96: pBN(Mem, PC, 3); Serial.print(F("sub  a,")); printIXYOffset(xy, MREAD(PC+2)); return 3;
    case 0x9C: pBN(Mem, PC, 2); pr("sbc  a,", "h"); return 2;
    case 0x9D: pBN(Mem, PC, 2); pr("sbc  a,", "l"); return 2;
    case 0x9E: pBN(Mem, PC, 3); Serial.print(F("sbc  a,")); printIXYOffset(xy, MREAD(PC+2)); return 3;
    case 0xA4: pBN(Mem, PC, 2); pr("and  ", "h"); return 2;
    case 0xA5: pBN(Mem, PC, 2); pr("and  ", "l"); return 2;
    case 0xA6: pBN(Mem, PC, 3); Serial.print(F("and  ")); printIXYOffset(xy, MREAD(PC+2)); return 3;
    case 0xAC: pBN(Mem, PC, 2); pr("xor  ", "h"); return 2;
    case 0xAD: pBN(Mem, PC, 2); pr("xor  ", "l"); return 2;
    case 0xAE: pBN(Mem, PC, 3); Serial.print(F("xor  ")); printIXYOffset(xy, MREAD(PC+2)); return 3;
    case 0xB4: pBN(Mem, PC, 2); pr("or   ", "h"); return 2;
    case 0xB5: pBN(Mem, PC, 2); pr("or   ", "l"); return 2;
    case 0xB6: pBN(Mem, PC, 3); Serial.print(F("or   ")); printIXYOffset(xy, MREAD(PC+2)); return 3;
    case 0xBC: pBN(Mem, PC, 2); pr("cp   ", "h"); return 2;
    case 0xBD: pBN(Mem, PC, 2); pr("cp   ", "l"); return 2;
    case 0xBE: pBN(Mem, PC, 3); Serial.print(F("cp   ")); printIXYOffset(xy, MREAD(PC+2)); return 3;
    case 0xCB: pBN(Mem, PC, 4); da_bitop(MREAD(PC+3), xy, MREAD(PC+2)); return 4;
    case 0xE1: pBN(Mem, PC, 2); pr("pop  ", ""); return 2;
    case 0xE3: pBN(Mem, PC, 2); pr("ex   (sp),", ""); return 2;
    case 0xE5: pBN(Mem, PC, 2); pr("push ", ""); return 2;
    case 0xE9: pBN(Mem, PC, 2); pr("jp   (", ")"); return 2;
    case 0xF9: pBN(Mem, PC, 2); pr("ld   sp,", ""); return 2;

    default:
      pBN(Mem, PC, 1); 
      Serial.print('[');
      numsys_print_byte(prefix);
      Serial.print(']');
      return 1;
    }
}


static byte da_bit(byte opcode, byte *Mem, uint16_t PC) // 0xCB prefix
{
  pBN(Mem, PC, 2); 
  da_bitop(MREAD(PC+1), 0, 0);
  return 2;
}


static byte da_ext(byte prefix, byte *Mem, uint16_t PC) // 0xED prefix
{
  // extended instructions
  byte opcode = MREAD(PC+1);

  switch( opcode & 0xCF )
    {
    case 0x40: 
    case 0x48: pBN(Mem, PC, 2); Serial.print(F("in   ")); pRN((opcode & 0x38)/8); Serial.print(F(",(c)")); return 2;
    case 0x41:
    case 0x49: pBN(Mem, PC, 2); Serial.print(F("out  ")); pRN((opcode & 0x38)/8); Serial.print(F(",(c)")); return 2;
    case 0x42: pBN(Mem, PC, 2); Serial.print(F("sbc  hl,")); pR2N((opcode & 0x30)/16); return 2;
    case 0x4A: pBN(Mem, PC, 2); Serial.print(F("adc  hl,")); pR2N((opcode & 0x30)/16); return 2;
    case 0x43: pBN(Mem, PC, 4); Serial.print(F("ld   (")); pW(Mem, PC+2); Serial.print(F("),")); pR2N((opcode & 0x30)/16); return 4;
    case 0x4B: pBN(Mem, PC, 4); Serial.print(F("ld   ")); pR2N((opcode & 0x30)/16); Serial.print(F(",(")); pW(Mem, PC+2); Serial.print(')'); return 4;
    case 0x44:
    case 0x4C: pBN(Mem, PC, 2); Serial.print(F("neg")); return 2;
    case 0x45:
    case 0x4D: pBN(Mem, PC, 2); Serial.print(F("retn")); return 2;
    case 0x46: pBN(Mem, PC, 2); Serial.print(F("im   ")); Serial.print((opcode & 0x10) ? F("0")   : F("1")); return 2;
    case 0x4E: pBN(Mem, PC, 2); Serial.print(F("im   ")); Serial.print((opcode & 0x10) ? F("0/1") : F("2")); return 2;
      
    default:
      {
        switch( opcode )
          {
          case 0x47: pBN(Mem, PC, 2); Serial.print(F("ld   i,a")); return 2;
          case 0x4F: pBN(Mem, PC, 2); Serial.print(F("ld   r,a")); return 2;
          case 0x57: pBN(Mem, PC, 2); Serial.print(F("ld   a,i")); return 2;
          case 0x5F: pBN(Mem, PC, 2); Serial.print(F("ld   a,r")); return 2;
          case 0x67: pBN(Mem, PC, 2); Serial.print(F("rrd")); return 2;
          case 0x6F: pBN(Mem, PC, 2); Serial.print(F("rld")); return 2;
          case 0xA0: pBN(Mem, PC, 2); Serial.print(F("ldi")); return 2;
          case 0xA1: pBN(Mem, PC, 2); Serial.print(F("cpi")); return 2;
          case 0xA2: pBN(Mem, PC, 2); Serial.print(F("ini")); return 2;
          case 0xA3: pBN(Mem, PC, 2); Serial.print(F("outi")); return 2;
          case 0xA8: pBN(Mem, PC, 2); Serial.print(F("ldd")); return 2;
          case 0xA9: pBN(Mem, PC, 2); Serial.print(F("cpd")); return 2;
          case 0xAA: pBN(Mem, PC, 2); Serial.print(F("ind")); return 2;
          case 0xAB: pBN(Mem, PC, 2); Serial.print(F("outd")); return 2;
          case 0xB0: pBN(Mem, PC, 2); Serial.print(F("ldir")); return 2;
          case 0xB1: pBN(Mem, PC, 2); Serial.print(F("cpir")); return 2;
          case 0xB2: pBN(Mem, PC, 2); Serial.print(F("inir")); return 2;
          case 0xB3: pBN(Mem, PC, 2); Serial.print(F("otir")); return 2;
          case 0xB8: pBN(Mem, PC, 2); Serial.print(F("lddr")); return 2;
          case 0xB9: pBN(Mem, PC, 2); Serial.print(F("cpdr")); return 2;
          case 0xBA: pBN(Mem, PC, 2); Serial.print(F("indr")); return 2;
          case 0xBB: pBN(Mem, PC, 2); Serial.print(F("otdr")); return 2;
      
          default:
            pBN(Mem, PC, 1); 
            Serial.print('[');
            numsys_print_byte(prefix);
            Serial.print(']');
            return 1;
          }
      }
    }
}


const DAFUN da_opcodes[] PROGMEM = {
  da_nop,   da_lxi,   da_stx,   da_inx,   da_inc,   da_dec,   da_ldRI,  da_rlca,	// 000-007 (0X00-0X07)
  da_exaf,  da_dad,   da_ldx,   da_dcx,   da_inc,   da_dec,   da_ldRI,  da_rrca,	// 010-017 (0X08-0X0F)
  da_djnz,  da_lxi,   da_stx,   da_inx,   da_inc,   da_dec,   da_ldRI,  da_rla,		// 020-027 (0X10-0X17)
  da_jr,    da_dad,   da_ldx,   da_dcx,   da_inc,   da_dec,   da_ldRI,  da_rra,		// 030-037 (0X18-0X1F)
  da_jrC,   da_lxi,   da_shld,  da_inx,   da_inc,   da_dec,   da_ldRI,  da_daa,		// 040-047 (0X20-0X27)
  da_jrC,   da_dad,   da_lhld,  da_dcx,   da_inc,   da_dec,   da_ldRI,  da_cpl,		// 050-057 (0X28-0X2F)
  da_jrC,   da_lxi,   da_STA,   da_inx,   da_inc,   da_dec,   da_ldRI,  da_scf,		// 060-067 (0X30-0X37)
  da_jrC,   da_dad,   da_LDA,   da_dcx,   da_inc,   da_dec,   da_ldRI,  da_ccf,		// 070-077 (0X38-0X3F)

  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRM,  da_ldRR,	// 100-107 (0X40-0X47)
  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRM,  da_ldRR,	// 110-117 (0X48-0X4F)
  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRM,  da_ldRR,	// 120-127 (0X50-0X57)
  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRM,  da_ldRR,	// 130-137 (0X58-0X5F)
  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRM,  da_ldRR,	// 140-147 (0X60-0X67)
  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRM,  da_ldRR,	// 150-157 (0X68-0X6F)
  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_hlt,   da_ldRR,	// 160-167 (0X70-0X77)
  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRR,  da_ldRM,  da_ldRR,	// 170-177 (0X78-0X7F)

  da_add,   da_add,   da_add,   da_add,   da_add,   da_add,   da_add,   da_add,		// 200-207 (0X80-0X87)
  da_adc,   da_adc,   da_adc,   da_adc,   da_adc,   da_adc,   da_adc,   da_adc,		// 210-217 (0X88-0X8F)
  da_sub,   da_sub,   da_sub,   da_sub,   da_sub,   da_sub,   da_sub,   da_sub,		// 220-227 (0X90-0X97)
  da_sbc,   da_sbc,   da_sbc,   da_sbc,   da_sbc,   da_sbc,   da_sbc,   da_sbc,		// 230-237 (0X98-0X9F)
  da_and,   da_and,   da_and,   da_and,   da_and,   da_and,   da_and,   da_and,		// 240-247 (0XA0-0XA7)
  da_xor,   da_xor,   da_xor,   da_xor,   da_xor,   da_xor,   da_xor,   da_xor,		// 250-257 (0XA8-0XAF)
  da_or,    da_or,    da_or,    da_or,    da_or,    da_or,    da_or,    da_or,	        // 260-267 (0XB0-0XB7)
  da_cp,    da_cp,    da_cp,    da_cp,    da_cp,    da_cp,    da_cp,    da_cp,		// 270-277 (0XB8-0XBF)

  da_retC,  da_pop,   da_jpC ,  da_jmp,   da_callC, da_push,  da_addI,  da_rst,		// 300-307 (0XC0-0XC7)
  da_retC,  da_ret,   da_jpC ,  da_bit,   da_callC, da_call,  da_adcI,  da_rst,		// 310-317 (0XC8-0XCF)
  da_retC,  da_pop,   da_jpC,   da_out,   da_callC, da_push,  da_subI,  da_rst,		// 320-327 (0XD0-0XD7)
  da_retC,  da_exx,   da_jpC,   da_in,    da_callC, da_ixiy,  da_sbcI,  da_rst,		// 330-337 (0XD8-0XDF)
  da_retC,  da_pop,   da_jpC,   da_exsp,  da_callC, da_push,  da_andI,  da_rst,		// 340-347 (0XE0-0XE7)
  da_retC,  da_jpHL,  da_jpC,   da_exde,  da_callC, da_ext,   da_xorI,  da_rst,		// 350-357 (0XE8-0XEF)
  da_retC,  da_popA,  da_jpC,   da_di,    da_callC, da_pusa,  da_orI,   da_rst,		// 360-367 (0XF0-0XF7)
  da_retC,  da_ldSP,  da_jpC,   da_ei,    da_callC, da_ixiy,  da_cpI,   da_rst		// 370-377 (0XF8-0XFF)
};


byte disassemble_z80(byte *Mem, uint16_t PC, bool pb)
{
  print_bytes = pb;
  byte opcode = MREAD(PC);
#if defined(__AVR_ATmega2560__)
  return ((DAFUN)pgm_read_word(&da_opcodes[opcode]))(opcode, Mem, PC);
#else
  return (da_opcodes[opcode])(opcode, Mem, PC);
#endif
}

#endif
