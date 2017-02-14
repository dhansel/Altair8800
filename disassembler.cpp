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

typedef byte (*DAFUN)(byte, byte *, uint16_t);

#define PRINT(x) Serial.print(x)

void PRINTPS(const char PROGMEM *s)
{
  byte b;
  while( (b=pgm_read_byte(s++))!=0 ) Serial.write(b);
}

void printRegName(byte regname)
{
  switch(regname)
    {
    case 0: PRINT('B'); break; 
    case 1: PRINT('C'); break; 
    case 2: PRINT('D'); break; 
    case 3: PRINT('E'); break; 
    case 4: PRINT('H'); break; 
    case 5: PRINT('L'); break; 
    case 6: PRINT('M'); break; 
    case 7: PRINT('A'); break; 
    default: PRINT('?'); break; 
    }
}

void printDblRegName(byte regname)
{
  switch(regname)
    {
    case 0: { PRINT('B'); PRINT('C'); break; }
    case 1: { PRINT('D'); PRINT('E'); break; }
    case 2: { PRINT('H'); PRINT('L'); break; }
    case 3: { PRINT('S'); PRINT('P'); break; }
    default: PRINT('?'); break; 
    }
}

byte da_ADC(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "ADC ";
  PRINTPS(nm);
  printRegName(opcode&0007);
  return 1;
}

byte da_ADD(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "ADD ";
  PRINTPS(nm);
  printRegName(opcode&0007);
  return 1;
}

byte da_SBB(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "SBB ";
  PRINTPS(nm);
  printRegName(opcode&0007);
  return 1;
}

byte da_SUB(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "SUB ";
  PRINTPS(nm);
  printRegName(opcode&0007);
  return 1;
}

byte da_ANA(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "ANA ";
  PRINTPS(nm);
  printRegName(opcode&0007);
  return 1;
}

byte da_XRA(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "XRA ";
  PRINTPS(nm);
  printRegName(opcode&0007);
  return 1;
}

byte da_ORA(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "ORA ";
  PRINTPS(nm);
  printRegName(opcode&0007);
  return 1;
}

byte da_CMP(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "CMP ";
  PRINTPS(nm);
  printRegName(opcode&0007);
  return 1;
}

byte da_CALL(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "CALL ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_DCR(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "DCR ";
  PRINTPS(nm);
  printRegName((opcode&0070)>>3);
  return 1;
}

byte da_ADI(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "ADI ";
  PRINTPS(nm);
  numsys_print_byte(MREAD(PC+1));
  return 2;
}

byte da_ACI(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "ACI ";
  PRINTPS(nm);
  numsys_print_byte(MREAD(PC+1));
  return 2;
}

byte da_SUI(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "SUI ";
  PRINTPS(nm);
  numsys_print_byte(MREAD(PC+1));
  return 2;
}

byte da_SBI(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "SBI ";
  PRINTPS(nm);
  numsys_print_byte(MREAD(PC+1));
  return 2;
}

byte da_ANI(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "ANI ";
  PRINTPS(nm);
  numsys_print_byte(MREAD(PC+1));
  return 2;
}

byte da_XRI(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "XRI ";
  PRINTPS(nm);
  numsys_print_byte(MREAD(PC+1));
  return 2;
}

byte da_ORI(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "ORI ";
  PRINTPS(nm);
  numsys_print_byte(MREAD(PC+1));
  return 2;
}

byte da_CPI(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "CPI ";
  PRINTPS(nm);
  numsys_print_byte(MREAD(PC+1));
  return 2;
}

byte da_CMA(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "CMA";
  PRINTPS(nm);
  return 1;
}

byte da_CMC(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "CMC";
  PRINTPS(nm);
  return 1;
}

byte da_DAA(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "DAA";
  PRINTPS(nm);
  return 1;
}

byte da_DAD(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "DAD ";
  PRINTPS(nm);
  printDblRegName((opcode & 0060)>>4);
  return 1;
}

byte da_DCX(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "DCX ";
  PRINTPS(nm);
  printDblRegName((opcode & 0060)>>4);
  return 1;
}

byte da_DI(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "DI";
  PRINTPS(nm);
  return 1;
}

byte da_EI(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "EI";
  PRINTPS(nm);
  return 1;
}

byte da_HLT(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "HLT";
  PRINTPS(nm);
  return 1;
}

byte da_INR(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "INR ";
  PRINTPS(nm);
  printRegName((opcode&0070)>>3);
  return 1;
}

byte da_INX(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "INX ";
  PRINTPS(nm);
  printDblRegName((opcode & 0060)>>4);
  return 1;
}

byte da_LDA(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "LDA ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_LDAX(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "LDAX ";
  PRINTPS(nm);
  printDblRegName((opcode & 0060)>>4);
  return 1;
}

byte da_LHLD(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "LHLD ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_LXI(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "LXI ";
  PRINTPS(nm);
  printDblRegName((opcode & 0060)>>4);
  PRINT(',');
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}
  
byte da_MOV(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "MOV ";
  PRINTPS(nm);
  printRegName((opcode&0070)>>3);
  PRINT(',');
  printRegName(opcode&0007);
  return 1;
}

byte da_MVI(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "MVI ";
  PRINTPS(nm);
  printRegName((opcode&0070)>>3);
  PRINT(',');
  numsys_print_byte(MREAD(PC+1));
  return 2;
}

byte da_MVR(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "MVR ";
  PRINTPS(nm);
  printRegName((opcode&0070)>>3);
  PRINT(',');
  printDblRegName(2);
  return 1;
}

byte da_NOP(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "NOP";
  PRINTPS(nm);
  return 1;
}

byte da_NUL(byte opcode, byte *Mem, uint16_t PC)
{
  PRINT('<');
  numsys_print_byte(opcode);
  PRINT('>');
  return 1;
}

byte da_PCHL(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "PCHL";
  PRINTPS(nm);
  return 1;
}

byte da_POP(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "POP ";
  PRINTPS(nm);
  printDblRegName((opcode&0060)>>4);
  return 1;
}

byte da_POPA(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "POP AS";
  PRINTPS(nm);
  return 1;
}

byte da_PUSH(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "PUSH ";
  PRINTPS(nm);
  printDblRegName((opcode&0060)>>4);
  return 1;
}

byte da_PUSA(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "PUSH AS";
  PRINTPS(nm);
  return 1;
}

byte da_RLC(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "RLC";
  PRINTPS(nm);
  return 1;
}

byte da_RRC(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "RRC";
  PRINTPS(nm);
  return 1;
}

byte da_RAL(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "RAL";
  PRINTPS(nm);
  return 1;
}

byte da_RAR(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "RAR";
  PRINTPS(nm);
  return 1;
}

byte da_RET(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "RET";
  PRINTPS(nm);
  return 1;
}

byte da_RST(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "RST ";
  PRINTPS(nm);
  numsys_print_word(opcode & 0070);
  return 1;
}

byte da_RNZ(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "RNZ";
  PRINTPS(nm);
  return 1;
}

byte da_RZ(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "RZ";
  PRINTPS(nm);
  return 1;
}

byte da_RNC(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "RNC";
  PRINTPS(nm);
  return 1;
}

byte da_RC(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "RC";
  PRINTPS(nm);
  return 1;
}

byte da_RPO(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "RPO";
  PRINTPS(nm);
  return 1;
}

byte da_RPE(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "RPE";
  PRINTPS(nm);
  return 1;
}

byte da_RP(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "RP";
  PRINTPS(nm);
  return 1;
}

byte da_RM(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "RM";
  PRINTPS(nm);
  return 1;
}

byte da_JMP(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "JMP ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_JNZ(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "JNZ ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_JZ(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "JZ ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_JNC(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "JNC ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_JC(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "JC ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_JPO(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "JPO ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_JPE(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "JPE ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_JP(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "JP ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_JM(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "JM ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_CNZ(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "CNZ ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_CZ(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "CZ ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_CNC(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "CNC ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_CC(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "CC ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_CPO(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "CPO ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_CPE(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "CPE ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_CP(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "CP ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_CM(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "CM ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_SHLD(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "SHLD ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_SPHL(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "SPHL";
  PRINTPS(nm);
  return 1;
}

byte da_STA(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "STA ";
  PRINTPS(nm);
  numsys_print_word(MREAD(PC+1) | (MREAD(PC+2) << 8));
  return 3;
}

byte da_STAX(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "STAX ";
  PRINTPS(nm);
  printDblRegName((opcode&0060)>>4);
  return 1;
}

byte da_STC(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "STC";
  PRINTPS(nm);
  return 1;
}

byte da_XTHL(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "XTHL";
  PRINTPS(nm);
  return 1;
}

byte da_XCHG(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "XCHG";
  PRINTPS(nm);
  return 1;
}

byte da_OUT(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "OUT ";
  PRINTPS(nm);
  numsys_print_byte(MREAD(PC+1));  
  return 2;
}


byte da_IN(byte opcode, byte *Mem, uint16_t PC)
{
  static const char PROGMEM nm[] = "IN ";
  PRINTPS(nm);
  numsys_print_byte(MREAD(PC+1));
  return 2;
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


byte disassemble(byte *Mem, uint16_t PC)
{
  byte opcode = MREAD(PC);
  return ((DAFUN)pgm_read_word(&da_opcodes[opcode]))(opcode, Mem, PC);
}
