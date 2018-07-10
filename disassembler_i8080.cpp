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

#if USE_Z80 != 1


typedef byte (*DAFUN)(byte, byte *, uint16_t);


static void printRegName(byte regname)
{
  switch(regname)
    {
    case 0: Serial.print('B'); break; 
    case 1: Serial.print('C'); break; 
    case 2: Serial.print('D'); break; 
    case 3: Serial.print('E'); break; 
    case 4: Serial.print('H'); break; 
    case 5: Serial.print('L'); break; 
    case 6: Serial.print('M'); break; 
    case 7: Serial.print('A'); break; 
    default: Serial.print('?'); break; 
    }
}

static void printDblRegName(byte regname)
{
  switch(regname)
    {
    case 0: { Serial.print('B'); Serial.print('C'); break; }
    case 1: { Serial.print('D'); Serial.print('E'); break; }
    case 2: { Serial.print('H'); Serial.print('L'); break; }
    case 3: { Serial.print('S'); Serial.print('P'); break; }
    default: Serial.print('?'); break; 
    }
}

static byte da_ADC(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("ADC "));
  printRegName(opcode&0007);
  return 1;
}

static byte da_ADD(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("ADD "));
  printRegName(opcode&0007);
  return 1;
}

static byte da_SBB(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("SBB "));
  printRegName(opcode&0007);
  return 1;
}

static byte da_SUB(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("SUB "));
  printRegName(opcode&0007);
  return 1;
}

static byte da_ANA(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("ANA "));
  printRegName(opcode&0007);
  return 1;
}

static byte da_XRA(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("XRA "));
  printRegName(opcode&0007);
  return 1;
}

static byte da_ORA(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("ORA "));
  printRegName(opcode&0007);
  return 1;
}

static byte da_CMP(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("CMP "));
  printRegName(opcode&0007);
  return 1;
}

static byte da_CALL(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("CALL "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_DCR(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("DCR "));
  printRegName((opcode&0070)>>3);
  return 1;
}

static byte da_ADI(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("ADI "));
  numsys_print_byte(MREAD(PC+1));
  return 2;
}

static byte da_ACI(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("ACI "));
  numsys_print_byte(MREAD(PC+1));
  return 2;
}

static byte da_SUI(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("SUI "));
  numsys_print_byte(MREAD(PC+1));
  return 2;
}

static byte da_SBI(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("SBI "));
  numsys_print_byte(MREAD(PC+1));
  return 2;
}

static byte da_ANI(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("ANI "));
  numsys_print_byte(MREAD(PC+1));
  return 2;
}

static byte da_XRI(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("XRI "));
  numsys_print_byte(MREAD(PC+1));
  return 2;
}

static byte da_ORI(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("ORI "));
  numsys_print_byte(MREAD(PC+1));
  return 2;
}

static byte da_CPI(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("CPI "));
  numsys_print_byte(MREAD(PC+1));
  return 2;
}

static byte da_CMA(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("CMA"));
  return 1;
}

static byte da_CMC(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("CMC"));
  return 1;
}

static byte da_DAA(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("DAA"));
  return 1;
}

static byte da_DAD(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("DAD "));
  printDblRegName((opcode & 0060)>>4);
  return 1;
}

static byte da_DCX(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("DCX "));
  printDblRegName((opcode & 0060)>>4);
  return 1;
}

static byte da_DI(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("DI"));
  return 1;
}

static byte da_EI(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("EI"));
  return 1;
}

static byte da_HLT(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("HLT"));
  return 1;
}

static byte da_INR(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("INR "));
  printRegName((opcode&0070)>>3);
  return 1;
}

static byte da_INX(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("INX "));
  printDblRegName((opcode & 0060)>>4);
  return 1;
}

static byte da_LDA(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("LDA "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_LDAX(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("LDAX "));
  printDblRegName((opcode & 0060)>>4);
  return 1;
}

static byte da_LHLD(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("LHLD "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_LXI(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("LXI "));
  printDblRegName((opcode & 0060)>>4);
  Serial.print(',');
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}
  
static byte da_MOV(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("MOV "));
  printRegName((opcode&0070)>>3);
  Serial.print(',');
  printRegName(opcode&0007);
  return 1;
}

static byte da_MVI(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("MVI "));
  printRegName((opcode&0070)>>3);
  Serial.print(',');
  numsys_print_byte(MREAD(PC+1));
  return 2;
}

static byte da_MVR(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("MVR "));
  printRegName((opcode&0070)>>3);
  Serial.print(',');
  printDblRegName(2);
  return 1;
}

static byte da_NOP(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("NOP"));
  return 1;
}

static byte da_PCHL(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("PCHL"));
  return 1;
}

static byte da_POP(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("POP "));
  printDblRegName((opcode&0060)>>4);
  return 1;
}

static byte da_POPA(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("POP AS"));
  return 1;
}

static byte da_PUSH(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("PUSH "));
  printDblRegName((opcode&0060)>>4);
  return 1;
}

static byte da_PUSA(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("PUSH AS"));
  return 1;
}

static byte da_RLC(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("RLC"));
  return 1;
}

static byte da_RRC(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("RRC"));
  return 1;
}

static byte da_RAL(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("RAL"));
  return 1;
}

static byte da_RAR(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("RAR"));
  return 1;
}

static byte da_RET(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("RET"));
  return 1;
}

static byte da_RST(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("RST "));
  numsys_print_word(opcode & 0070);
  return 1;
}

static byte da_RNZ(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("RNZ"));
  return 1;
}

static byte da_RZ(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("RZ"));
  return 1;
}

static byte da_RNC(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("RNC"));
  return 1;
}

static byte da_RC(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("RC"));
  return 1;
}

static byte da_RPO(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("RPO"));
  return 1;
}

static byte da_RPE(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("RPE"));
  return 1;
}

static byte da_RP(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("RP"));
  return 1;
}

static byte da_RM(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("RM"));
  return 1;
}

static byte da_JMP(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("JMP "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_JNZ(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("JNZ "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_JZ(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("JZ "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_JNC(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("JNC "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_JC(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("JC "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_JPO(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("JPO "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_JPE(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("JPE "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_JP(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("JP "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_JM(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("JM "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_CNZ(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("CNZ "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_CZ(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("CZ "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_CNC(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("CNC "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_CC(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("CC "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_CPO(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("CPO "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_CPE(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("CPE "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_CP(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("CP "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_CM(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("CM "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_SHLD(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("SHLD "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_SPHL(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("SPHL"));
  return 1;
}

static byte da_STA(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("STA "));
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

static byte da_STAX(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("STAX "));
  printDblRegName((opcode&0060)>>4);
  return 1;
}

static byte da_STC(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("STC"));
  return 1;
}

static byte da_XTHL(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("XTHL"));
  return 1;
}

static byte da_XCHG(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("XCHG"));
  return 1;
}

static byte da_OUT(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("OUT "));
  numsys_print_byte(MREAD(PC+1));  
  return 2;
}


static byte da_IN(byte opcode, byte *Mem, uint16_t PC)
{
  Serial.print(F("IN "));
  numsys_print_byte(MREAD(PC+1));
  return 2;
}


static byte da_NUL(byte opcode, byte *Mem, uint16_t PC)
{
  byte res = 1;

  Serial.print('<');
  numsys_print_byte(opcode);
  Serial.print(F("> ["));

  // the following opcodes are not official but (at least on 
  // the Intel 8080) behave like other opcodes. 
  // => show the actual behavior when disassembling
  switch( opcode )
    {
    case 0010:
    case 0020:
    case 0030:
    case 0040:
    case 0050:
    case 0060:
    case 0070: res = da_NOP(opcode, Mem, PC); break;
    case 0313: res = da_JMP(opcode, Mem, PC); break;
    case 0331: res = da_RET(opcode, Mem, PC); break;
    case 0335:
    case 0355:
    case 0375: res = da_CALL(opcode, Mem, PC); break;
    default:   res = 1; Serial.print(F("???")); break;
    }

  Serial.print(']');
  return res;
}

const DAFUN da_opcodes[] PROGMEM = {
  da_NOP, da_LXI, da_STAX,da_INX, da_INR, da_DCR, da_MVI, da_RLC,		// 000-007 (0X00-0X07)
  da_NUL, da_DAD, da_LDAX,da_DCX, da_INR, da_DCR, da_MVI, da_RRC,		// 010-017 (0X08-0X0F)
  da_NUL, da_LXI, da_STAX,da_INX, da_INR, da_DCR, da_MVI, da_RAL,		// 020-027 (0X10-0X17)
  da_NUL, da_DAD, da_LDAX,da_DCX, da_INR, da_DCR, da_MVI, da_RAR,		// 030-037 (0X18-0X1F)
  da_NUL, da_LXI, da_SHLD,da_INX, da_INR, da_DCR, da_MVI, da_DAA,		// 040-047 (0X20-0X27)
  da_NUL, da_DAD, da_LHLD,da_DCX, da_INR, da_DCR, da_MVI, da_CMA,		// 050-057 (0X28-0X2F)
  da_NUL, da_LXI, da_STA, da_INX, da_INR, da_DCR, da_MVI, da_STC,		// 060-067 (0X30-0X37)
  da_NUL, da_DAD, da_LDA, da_DCX, da_INR, da_DCR, da_MVI, da_CMC,		// 070-077 (0X38-0X3F)
  
  da_MOV, da_MOV, da_MOV, da_MOV, da_MOV, da_MOV, da_MVR, da_MOV,		// 100-107 (0X40-0X47)
  da_MOV, da_MOV, da_MOV, da_MOV, da_MOV, da_MOV, da_MVR, da_MOV,		// 110-117 (0X48-0X4F)
  da_MOV, da_MOV, da_MOV, da_MOV, da_MOV, da_MOV, da_MVR, da_MOV,		// 120-127 (0X50-0X57)
  da_MOV, da_MOV, da_MOV, da_MOV, da_MOV, da_MOV, da_MVR, da_MOV,		// 130-137 (0X58-0X5F)
  da_MOV, da_MOV, da_MOV, da_MOV, da_MOV, da_MOV, da_MVR, da_MOV,		// 140-147 (0X60-0X67)
  da_MOV, da_MOV, da_MOV, da_MOV, da_MOV, da_MOV, da_MVR, da_MOV,		// 150-157 (0X68-0X6F)
  da_MOV, da_MOV, da_MOV, da_MOV, da_MOV, da_MOV, da_HLT, da_MOV,		// 160-167 (0X70-0X77)
  da_MOV, da_MOV, da_MOV, da_MOV, da_MOV, da_MOV, da_MVR, da_MOV,		// 170-177 (0X78-0X7F)
  
  da_ADD, da_ADD, da_ADD, da_ADD, da_ADD, da_ADD, da_ADD, da_ADD,		// 200-207 (0X80-0X87)
  da_ADC, da_ADC, da_ADC, da_ADC, da_ADC, da_ADC, da_ADC, da_ADC,		// 210-217 (0X88-0X8F)
  da_SUB, da_SUB, da_SUB, da_SUB, da_SUB, da_SUB, da_SUB, da_SUB,		// 220-227 (0X90-0X97)
  da_SBB, da_SBB, da_SBB, da_SBB, da_SBB, da_SBB, da_SBB, da_SBB,		// 230-237 (0X98-0X9F)
  da_ANA, da_ANA, da_ANA, da_ANA, da_ANA, da_ANA, da_ANA, da_ANA,		// 240-247 (0XA0-0XA7)
  da_XRA, da_XRA, da_XRA, da_XRA, da_XRA, da_XRA, da_XRA, da_XRA,		// 250-257 (0XA8-0XAF)
  da_ORA, da_ORA, da_ORA, da_ORA, da_ORA, da_ORA, da_ORA, da_ORA, 	        // 260-267 (0XB0-0XB7)
  da_CMP, da_CMP, da_CMP, da_CMP, da_CMP, da_CMP, da_CMP, da_CMP,		// 270-277 (0XB8-0XBF)
  
  da_RNZ, da_POP, da_JNZ, da_JMP, da_CNZ, da_PUSH,da_ADI, da_RST,		// 300-307 (0XC0-0XC7)
  da_RZ,  da_RET, da_JZ,  da_NUL, da_CZ,  da_CALL,da_ACI, da_RST,		// 310-317 (0XC8-0XCF)
  da_RNC, da_POP, da_JNC, da_OUT, da_CNC, da_PUSH,da_SUI, da_RST,		// 320-327 (0XD0-0XD7)
  da_RC,  da_NUL, da_JC,  da_IN,  da_CC,  da_NUL, da_SBI, da_RST,		// 330-337 (0XD8-0XDF)
  da_RPO, da_POP, da_JPO, da_XTHL,da_CPO, da_PUSH,da_ANI, da_RST,		// 340-347 (0XE0-0XE7)
  da_RPE, da_PCHL,da_JPE, da_XCHG,da_CPE, da_NUL, da_XRI, da_RST,		// 350-357 (0XE8-0XEF)
  da_RP,  da_POPA,da_JP,  da_DI,  da_CP,  da_PUSA,da_ORI, da_RST,		// 360-367 (0XF0-0XF7)
  da_RM,  da_SPHL,da_JM,  da_EI,  da_CM,  da_NUL, da_CPI, da_RST		// 370-377 (0XF8-0XFF)
  };


byte disassemble_i8080(byte *Mem, uint16_t PC, bool pb)
{
  byte opcode = MREAD(PC);
#if defined(__AVR_ATmega2560__)
  return ((DAFUN)pgm_read_word(&da_opcodes[opcode]))(opcode, Mem, PC);
#else
  return (da_opcodes[opcode])(opcode, Mem, PC);
#endif
}

#endif
