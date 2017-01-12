#include "cpucore.h"
#include "profile.h"
#include "mem.h"
#include "Altair8800.h"

#if CPUCORE_VERSION == 1

byte Regs[8];
uint16_t regSP;
uint16_t regPC;

#define setCarryBit(v)     if(v) regS |= PS_CARRY;     else regS &= ~PS_CARRY
#define setHalfCarryBit(v) if(v) regS |= PS_HALFCARRY; else regS &= ~PS_HALFCARRY

static const byte halfCarryTableAdd[] = { 0, 0, 1, 0, 1, 0, 1, 1 };
static const byte halfCarryTableSub[] = { 1, 0, 0, 0, 1, 1, 1, 0 };
#define setHalfCarryBitAdd(opd1, opd2, res) setHalfCarryBit(halfCarryTableAdd[((((opd1) & 0x08) / 2) | (((opd2) & 0x08) / 4) | (((res) & 0x08) / 8)) & 0x07])
#define setHalfCarryBitSub(opd1, opd2, res) setHalfCarryBit(halfCarryTableSub[((((opd1) & 0x08) / 2) | (((opd2) & 0x08) / 4) | (((res) & 0x08) / 8)) & 0x07])


inline void setStatusBits(byte value)
{
  byte b;
  static const byte bit_table[16] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};
  b = regS & ~(PS_ZERO|PS_SIGN|PS_PARITY);

  if( !((bit_table[value&0x0f]+bit_table[value/16]) & 1) ) b |= PS_PARITY;
  if( value==0 ) b |= PS_ZERO;
  b |= (value & PS_SIGN);

  regS = b;
}


inline void pushStack(byte value)
{
  if( !host_read_status_led_WAIT() )
    {
      regSP--;
      host_set_status_led_STACK();
      MEM_WRITE(regSP, value);
      host_clr_status_led_STACK();
    }
  else if( altair_isreset() )
    {
      host_set_status_led_STACK();
      regSP--;
      MEM_WRITE(regSP, value);
      host_clr_status_led_STACK();
    }
}

inline byte popStack()
{
  if( !host_read_status_led_WAIT() )
    {
      host_set_status_led_STACK();
      byte v = MEM_READ(regSP);
      host_clr_status_led_STACK();
      regSP++;
      return v;
    }
  else if( altair_isreset() )
    {
      byte v;
      host_set_status_led_STACK();
      v = MEM_READ(regSP);
      if( altair_isreset() ) regSP++;
      host_clr_status_led_STACK();
      return v;
    }
  else
    return 0;
}

inline void pushPC()
{
  pushStack(regPC/256);
  pushStack(regPC&0xff);
}

inline void popPC()
{
  regPC  = popStack();
  regPC |= popStack()*256;
}



void cpu_ADC(byte opcode)
{
  byte opd2 = Regs[opcode&0007];
  uint16_t w    = regA + opd2;
  if(regS & PS_CARRY) w++;
  setHalfCarryBitAdd(regA, opd2, w);
  setCarryBit(w&0x100);
  setStatusBits(w);
  regA = w;
  PROF_ADD_CYCLES(4);
}

void cpu_ADD(byte opcode)
{
  byte opd2 = Regs[opcode&0007];
  uint16_t w    = regA + opd2;
  setCarryBit(w&0x100);
  setHalfCarryBitAdd(regA, opd2, w);
  setStatusBits(w);
  regA = w;
  PROF_ADD_CYCLES(4);
}

void cpu_SBB(byte opcode)
{
  byte opd2 = Regs[opcode&0007];
  uint16_t w    = regA - opd2;
  if(regS & PS_CARRY) w--; 
  setHalfCarryBitSub(regA, opd2, w);
  setCarryBit(w&0x100);
  setStatusBits(w);
  regA = w;
  PROF_ADD_CYCLES(4);
}

void cpu_SUB(byte opcode)
{
  byte opd2 = Regs[opcode&0007];
  uint16_t w    = regA - opd2;
  setCarryBit(w&0x100);
  setHalfCarryBitSub(regA, opd2, w);
  setStatusBits(w);
  regA = w;
  PROF_ADD_CYCLES(4);
}

void cpu_ANA(byte opcode)
{
  byte opd2 = Regs[opcode&0007];
  setHalfCarryBit(regA&0x08 | opd2&0x08);
  regA &= opd2;
  setCarryBit(0);
  setStatusBits(regA);
  PROF_ADD_CYCLES(4);
}

void cpu_XRA(byte opcode)
{
  regA ^= Regs[opcode&0007];
  setCarryBit(0);
  setHalfCarryBit(0);
  setStatusBits(regA);
  PROF_ADD_CYCLES(4);
}

void cpu_ORA(byte opcode)
{
  regA |= Regs[opcode&0007];
  setCarryBit(0);
  setHalfCarryBit(0);
  setStatusBits(regA);
  PROF_ADD_CYCLES(4);
}

void cpu_ADCM(byte opcode)
{
  uint16_t addr = (regH << 8) | regL;
  byte opd2 = MEM_READ(addr);
  uint16_t w    = regA + opd2;
  if(regS & PS_CARRY) w++; 
  setHalfCarryBitAdd(regA, opd2, w);
  setCarryBit(w&0x100);
  setStatusBits(w);
  regA = w;
  PROF_ADD_CYCLES(7);
}

void cpu_ADDM(byte opcode)
{
  uint16_t addr = (regH << 8) | regL;
  byte opd2 = MEM_READ(addr);
  uint16_t w    = regA + opd2;
  setCarryBit(w&0x100);
  setHalfCarryBitAdd(regA, opd2, w);
  setStatusBits(w);
  regA = w;
  PROF_ADD_CYCLES(7);
}

void cpu_SBBM(byte opcode)
{
  uint16_t addr = (regH << 8) | regL;
  byte opd2 = MEM_READ(addr);
  uint16_t w    = regA - opd2;
  if(regS & PS_CARRY) w--; 
  setHalfCarryBitSub(regA, opd2, w);
  setCarryBit(w&0x100);
  setStatusBits(w);
  regA = w;
  PROF_ADD_CYCLES(7);
}

void cpu_SUBM(byte opcode)
{
  uint16_t addr = (regH << 8) | regL;
  byte opd2 = MEM_READ(addr);
  uint16_t w    = regA - opd2;
  setCarryBit(w&0x100);
  setHalfCarryBitSub(regA, opd2, w);
  setStatusBits(w);
  regA = w;
  PROF_ADD_CYCLES(7);
}

void cpu_ANAM(byte opcode)
{
  uint16_t addr = (regH << 8) | regL;
  byte opd2 = MEM_READ(addr);
  setHalfCarryBit(regA&0x08 | opd2&0x08);
  regA &= opd2;
  setCarryBit(0);
  setStatusBits(regA);
  PROF_ADD_CYCLES(7);
}

void cpu_XRAM(byte opcode)
{
  uint16_t addr = (regH << 8) | regL;
  regA ^= MEM_READ(addr);
  setCarryBit(0);
  setHalfCarryBit(0);
  setStatusBits(regA);
  PROF_ADD_CYCLES(7);
}

void cpu_ORAM(byte opcode)
{
  uint16_t addr = (regH << 8) | regL;
  regA |= MEM_READ(addr);
  setCarryBit(0);
  setHalfCarryBit(0);
  setStatusBits(regA);
  PROF_ADD_CYCLES(7);
}

void cpu_CALL(byte opcode)
{
  regPC += 2;
  pushPC();
  regPC = MEM_READ(regPC-2) | (MEM_READ(regPC-1) << 8);
  PROF_ADD_CYCLES(17);
}

void cpu_CMP(byte opcode)
{
  byte opd2 = Regs[opcode&0007];
  uint16_t w = regA - opd2;
  setCarryBit(w&0x100);
  setHalfCarryBitSub(regA, opd2, w);
  setStatusBits(w);
  PROF_ADD_CYCLES(4);
}

void cpu_CMPM(byte opcode)
{
  uint16_t addr = (regH << 8) | regL;
  byte opd2 = MEM_READ(addr);
  uint16_t w    = regA - opd2;
  setCarryBit(w&0x100);
  setHalfCarryBitSub(regA, opd2, w);
  setStatusBits(w);
  PROF_ADD_CYCLES(7);
}

void cpu_DCR(byte opcode)
{
  byte dst = (opcode&0070); dst /= 8;
  byte res = Regs[dst] - 1;
  setHalfCarryBit((res & 0x0f)!=0x0f);
  setStatusBits(res);
  Regs[dst] = res;
  PROF_ADD_CYCLES(4);
}

void cpu_DCRM(byte opcode)
{
  uint16_t addr = (regH << 8) | regL;
  byte res  = MEM_READ(addr) - 1;
  setHalfCarryBit((res & 0x0f)!=0x0f);
  setStatusBits(res);
  MEM_WRITE(addr, res);
  PROF_ADD_CYCLES(7);
}

void cpu_ADI(byte opcode)
{
  byte opd2 = MEM_READ(regPC);
  uint16_t w    = regA + opd2;
  setCarryBit(w & 0x100);
  setHalfCarryBitAdd(regA, opd2, w);
  setStatusBits(w);
  regA = w;
  regPC++;
  PROF_ADD_CYCLES(7);
}

void cpu_ACI(byte opcode)
{
  byte opd2 = MEM_READ(regPC);
  uint16_t w    = regA + opd2;
  if(regS & PS_CARRY) w++;
  setHalfCarryBitAdd(regA, opd2, w);
  setCarryBit(w&0x100);
  setStatusBits(w);
  regA = w;
  regPC++;
  PROF_ADD_CYCLES(7);
}

void cpu_SUI(byte opcode)
{
  byte opd2 = MEM_READ(regPC);
  uint16_t w    = regA - opd2;
  setCarryBit(w&0x100);
  setHalfCarryBitSub(regA, opd2, w);
  setStatusBits(w);
  regA = w;
  regPC++;
  PROF_ADD_CYCLES(7);
}

void cpu_SBI(byte opcode)
{
  byte opd2 = MEM_READ(regPC);
  uint16_t w    = regA - opd2;
  if(regS & PS_CARRY) w--;
  setHalfCarryBitSub(regA, opd2, w);
  setCarryBit(w&0x100);
  setStatusBits(w);
  regA = w;
  regPC++;
  PROF_ADD_CYCLES(7);
}

void cpu_ANI(byte opcode)
{
  byte opd2 = MEM_READ(regPC);
  setHalfCarryBit(regA&0x08 | opd2&0x08);
  regA &= opd2;
  setCarryBit(0);
  setStatusBits(regA);
  regPC++;
  PROF_ADD_CYCLES(7);
}

void cpu_XRI(byte opcode)
{
  regA ^= MEM_READ(regPC);
  setCarryBit(0);
  setHalfCarryBit(0);
  setStatusBits(regA);
  regPC++;
  PROF_ADD_CYCLES(7);
}

void cpu_ORI(byte opcode)
{
  regA |= MEM_READ(regPC);
  setCarryBit(0);
  setHalfCarryBit(0);
  setStatusBits(regA);
  regPC++;
  PROF_ADD_CYCLES(7);
}

void cpu_CPI(byte opcode)
{
  byte opd2 = MEM_READ(regPC);
  uint16_t w    = regA - opd2;
  setCarryBit(w&0x100);
  setHalfCarryBitSub(regA, opd2, w);
  setStatusBits(w);
  regPC++;
  PROF_ADD_CYCLES(7);
}

void cpu_CMA(byte opcode)
{
  regA = ~regA;
  PROF_ADD_CYCLES(4);
}

void cpu_CMC(byte opcode)
{
  regS ^= PS_CARRY;
  PROF_ADD_CYCLES(4);
}

void cpu_DAA(byte opcode)
{
  byte b   = regA;
  byte adj = 0;
  if( (regS & PS_HALFCARRY) || (b & 0x0f) > 0x09 ) adj = 0x06;
  if( (regS & PS_CARRY)     || (b & 0xf0) > 0x90 || ((b & 0xf0) == 0x90 && (b & 0x0f) > 9) )
    { adj  |= 0x60; regS |= PS_CARRY; }

  regA = b + adj;
  setHalfCarryBitAdd(b, adj, regA);
  setStatusBits(regA);
  PROF_ADD_CYCLES(4);
}

void cpu_DAD(byte opcode)
{
  byte src = (opcode & 0060); src /= 8;
  uint16_t w0  = Regs[src+1] | (Regs[src] << 8);
  uint16_t w   = w0 + ((regH << 8) | regL);
  regH = w / 256;
  regL = w & 0xff;
  setCarryBit(w < w0);
  PROF_ADD_CYCLES(10);
}

void cpu_DADS(byte opcode)
{
  uint16_t w = regSP + ((regH << 8) | regL);
  regH = w / 256;
  regL = w & 0xff;
  setCarryBit(w < regSP);
  PROF_ADD_CYCLES(10);
}

void cpu_DCX(byte opcode)
{
  byte dst = (opcode & 0060); dst /= 8;
  uint16_t w = Regs[dst+1] | (Regs[dst] << 8);
  w--;
  Regs[dst+1] = w & 0xff;
  Regs[dst]   = w / 256;
  PROF_ADD_CYCLES(5);
}

void cpu_DCXS(byte opcode)
{
  regSP--;
  PROF_ADD_CYCLES(5);
}

void cpu_DI(byte opcode)
{
  host_clr_status_led_INTE();
  PROF_ADD_CYCLES(4);
}

void cpu_EI(byte opcode)
{
  host_set_status_led_INTE();
  PROF_ADD_CYCLES(4);
}

void cpu_HLT(byte opcode)
{
  altair_hlt();
  PROF_ADD_CYCLES(7);
}

void cpu_INR(byte opcode)
{
  byte dst = (opcode&0070); dst /= 8;
  byte res = Regs[dst] + 1;
  setHalfCarryBit((res&0x0f)==0);
  setStatusBits(res);
  Regs[dst] = res;
  PROF_ADD_CYCLES(5);
}

void cpu_INRM(byte opcode)
{
  uint16_t addr = (regH << 8) | regL;
  byte res  = MEM_READ(addr) + 1;
  setHalfCarryBit((res&0x0f)==0);
  setStatusBits(res);
  MEM_WRITE(addr, res);
  PROF_ADD_CYCLES(10);
}

void cpu_INX(byte opcode)
{
  byte dst = (opcode & 0060); dst /= 8;
  uint16_t w   = Regs[dst+1] | (Regs[dst] << 8);
  w++;
  Regs[dst+1] = w & 0xff;
  Regs[dst]   = w / 256;
  PROF_ADD_CYCLES(5);
}

void cpu_INXS(byte opcode)
{
  regSP++;
  PROF_ADD_CYCLES(5);
}

void cpu_LDA(byte opcode)
{
  uint16_t addr = MEM_READ(regPC) | (MEM_READ(regPC+1) << 8);
  regA = MEM_READ(addr);
  regPC += 2;
  PROF_ADD_CYCLES(13);
}

void cpu_LDAX(byte opcode)
{
  byte dst = (opcode & 0060); dst /= 8;
  uint16_t addr = (Regs[dst] << 8) | Regs[dst+1];
  regA = MEM_READ(addr);
  PROF_ADD_CYCLES(7);
}

void cpu_LHLD(byte opcode)
{
  uint16_t addr = MEM_READ(regPC) | (MEM_READ(regPC+1) << 8);
  regL = MEM_READ(addr);
  regH = MEM_READ(addr+1);
  regPC += 2;
  PROF_ADD_CYCLES(16);
}

void cpu_LXIS(byte opcode)
{
  regSP = MEM_READ(regPC) | (MEM_READ(regPC+1) << 8);
  regPC += 2;
  PROF_ADD_CYCLES(10);
}
  
void cpu_LXI(byte opcode)
{
  byte dst = (opcode&0060); dst /= 8;
  Regs[dst+1] = MEM_READ(regPC);
  Regs[dst]   = MEM_READ(regPC+1);
  regPC += 2;
  PROF_ADD_CYCLES(10);
}

void cpu_MOV(byte opcode)
{
  byte dst = (opcode&0070); dst /= 8;
  Regs[dst] = Regs[opcode&0007];
  PROF_ADD_CYCLES(5);
}

void cpu_MVM(byte opcode)
{
  // MOV M,[src]
  uint16_t addr = (regH << 8) | regL;
  byte b    = Regs[opcode&0007];
  MEM_WRITE(addr, b);
  PROF_ADD_CYCLES(7);
}

void cpu_MVI(byte opcode)
{
  byte dst = (opcode&0070); dst /= 8;
  Regs[dst] = MEM_READ(regPC);
  regPC++;
  PROF_ADD_CYCLES(7);
}

void cpu_MVIM(byte opcode)
{
  // MVI dst, M 
  uint16_t addr = (regH << 8) | regL;
  uint16_t b    = MEM_READ(regPC);
  MEM_WRITE(addr, b);
  regPC++;
  PROF_ADD_CYCLES(10);
}

void cpu_MVR(byte opcode)
{
  byte dst = (opcode&0070); dst /= 8;
  uint16_t addr = (regH << 8) | regL;
  Regs[dst] = MEM_READ(addr);
  PROF_ADD_CYCLES(7);
}

void cpu_NOP(byte opcode)
{
  PROF_ADD_CYCLES(4);
}

void cpu_NUL(byte opcode)
{
  if( opcode!=0x10 || regPC!=0x4F8 )
    {
      // opcode 0x10 is used (by mistake) in ALTAIR BASIC 4k at 0x4f7
      // => do not print error message in that case
      Serial.print(F("Illegal opcode ")); 
      Serial.print(F(" => treating as NOP\n"));
    }

  PROF_ADD_CYCLES(4);
}

void cpu_PCHL(byte opcode)
{
  regPC = (regH << 8) | regL;
  PROF_ADD_CYCLES(5);
}

void cpu_POP(byte opcode)
{
  byte dst = (opcode&0060); dst /= 8;
  Regs[dst+1] = popStack();
  Regs[dst]   = popStack();
  PROF_ADD_CYCLES(10);
}       

void cpu_POPA(byte opcode)
{
  // we're popping Regs[dst] first then Regs[dst+1] in this case
  regS = popStack(); 
  regA = popStack();
  PROF_ADD_CYCLES(10);
}       

void cpu_PUSH(byte opcode)
{
  byte src = (opcode&0070); src /= 8;
  pushStack(Regs[src]);
  pushStack(Regs[src+1]);
  PROF_ADD_CYCLES(11);
}       

void cpu_PUSA(byte opcode)
{
  // we're pushing Regs[src+1] first then Regs[src] in this case
  pushStack(regA);
  pushStack((regS & 0xD5) | 0x02); 
  PROF_ADD_CYCLES(11);
}

void cpu_RLC(byte opcode)
{
  byte b = regA & 128;
  regA   = (regA * 2) | (b ? 1 : 0) ;
  setCarryBit(b);
  PROF_ADD_CYCLES(4);
}

void cpu_RRC(byte opcode)
{
  byte b = regA & 1;
  regA   = (regA / 2) | (b ? 128 : 0) ;
  setCarryBit(b);
  PROF_ADD_CYCLES(4);
}

void cpu_RAL(byte opcode)
{
  byte b = regA & 128;
  regA   = (regA * 2) | ((regS & PS_CARRY) ? 1 : 0) ;
  setCarryBit(b);
  PROF_ADD_CYCLES(4);
}

void cpu_RAR(byte opcode)
{
  byte b = regA & 1;
  regA   = (regA / 2) | ((regS & PS_CARRY) ? 128 : 0) ;
  setCarryBit(b);
  PROF_ADD_CYCLES(4);
}

void cpu_RET(byte opcode)
{
  popPC();
  PROF_ADD_CYCLES(10);
}

void cpu_RST(byte opcode)
{
  pushPC();
  regPC = (opcode & 0070);
  PROF_ADD_CYCLES(4);
}

void cpu_RNZ(byte opcode)
{
  if( !(regS & PS_ZERO) ) 
    { popPC(); PROF_ADD_CYCLES(11); }
  else
    PROF_ADD_CYCLES(5);
}

void cpu_RZ(byte opcode)
{
  if( (regS & PS_ZERO) ) 
    { popPC(); PROF_ADD_CYCLES(11); }
  else
    PROF_ADD_CYCLES(5);
}

void cpu_RNC(byte opcode)
{
  if( !(regS & PS_CARRY) ) 
    { popPC(); PROF_ADD_CYCLES(11); }
  else
    PROF_ADD_CYCLES(5);
}

void cpu_RC(byte opcode)
{
  if( (regS & PS_CARRY) ) 
    { popPC(); PROF_ADD_CYCLES(11); }
  else
    PROF_ADD_CYCLES(5);
}

void cpu_RPO(byte opcode)
{
  if( !(regS & PS_PARITY) ) 
    { popPC(); PROF_ADD_CYCLES(11); }
  else
    PROF_ADD_CYCLES(5);
}

void cpu_RPE(byte opcode)
{
  if( (regS & PS_PARITY) ) 
    { popPC(); PROF_ADD_CYCLES(11); }
  else
    PROF_ADD_CYCLES(5);
}

void cpu_RP(byte opcode)
{
  if( !(regS & PS_SIGN) ) 
    { popPC(); PROF_ADD_CYCLES(11); }
  else
    PROF_ADD_CYCLES(5);
}

void cpu_RM(byte opcode)
{
  if( (regS & PS_SIGN) ) 
    { popPC(); PROF_ADD_CYCLES(11); }
  else
    PROF_ADD_CYCLES(5);
}

void cpu_JMP(byte opcode)
{
  regPC = MEM_READ(regPC) | (MEM_READ(regPC+1) << 8);
  PROF_ADD_CYCLES(10);
}

void cpu_JNZ(byte opcode)
{
  if( !(regS & PS_ZERO) ) 
    { regPC = MEM_READ(regPC) | (MEM_READ(regPC+1) << 8);  PROF_ADD_CYCLES(10); }
  else
    { regPC += 2; PROF_ADD_CYCLES(10); }
}

void cpu_JZ(byte opcode)
{
  if( (regS & PS_ZERO) ) 
    { regPC = MEM_READ(regPC) | (MEM_READ(regPC+1) << 8);  PROF_ADD_CYCLES(10); }
  else
    { regPC += 2; PROF_ADD_CYCLES(10); }
}

void cpu_JNC(byte opcode)
{
  if( !(regS & PS_CARRY) )
    { regPC = MEM_READ(regPC) | (MEM_READ(regPC+1) << 8);  PROF_ADD_CYCLES(10); }
  else
    { regPC += 2; PROF_ADD_CYCLES(10); }
}

void cpu_JC(byte opcode)
{
  if( (regS & PS_CARRY) )
    { regPC = MEM_READ(regPC) | (MEM_READ(regPC+1) << 8);  PROF_ADD_CYCLES(10); }
  else
    { regPC += 2; PROF_ADD_CYCLES(10); }
}

void cpu_JPO(byte opcode)
{
  if( !(regS & PS_PARITY) )
    { regPC = MEM_READ(regPC) | (MEM_READ(regPC+1) << 8);  PROF_ADD_CYCLES(10); }
  else
    { regPC += 2; PROF_ADD_CYCLES(10); }
}

void cpu_JPE(byte opcode)
{
  if( (regS & PS_PARITY) ) 
    { regPC = MEM_READ(regPC) | (MEM_READ(regPC+1) << 8);  PROF_ADD_CYCLES(10); }
  else
    { regPC += 2; PROF_ADD_CYCLES(10); }
}

void cpu_JP(byte opcode)
{
  if( !(regS & PS_SIGN) ) 
    { regPC = MEM_READ(regPC) | (MEM_READ(regPC+1) << 8);  PROF_ADD_CYCLES(10); }
  else
    { regPC += 2; PROF_ADD_CYCLES(10); }
}

void cpu_JM(byte opcode)
{
  if( (regS & PS_SIGN) ) 
    { regPC = MEM_READ(regPC) | (MEM_READ(regPC+1) << 8);  PROF_ADD_CYCLES(10); }
  else
    { regPC += 2; PROF_ADD_CYCLES(10); }
}

void cpu_CNZ(byte opcode)
{
  regPC += 2;
  if( !(regS & PS_ZERO) ) 
    { pushPC(); regPC = MEM_READ(regPC-2) | (MEM_READ(regPC-1) << 8); PROF_ADD_CYCLES(17); }
  else
    PROF_ADD_CYCLES(11);
}

void cpu_CZ(byte opcode)
{
  regPC += 2;
  if( (regS & PS_ZERO) )
    { pushPC(); regPC = MEM_READ(regPC-2) | (MEM_READ(regPC-1) << 8); PROF_ADD_CYCLES(17); }
  else
    PROF_ADD_CYCLES(11);
}

void cpu_CNC(byte opcode)
{
  regPC += 2;
  if( !(regS & PS_CARRY) )
    { pushPC(); regPC = MEM_READ(regPC-2) | (MEM_READ(regPC-1) << 8); PROF_ADD_CYCLES(17); }
  else
    PROF_ADD_CYCLES(11);
}

void cpu_CC(byte opcode)
{
  regPC += 2;
  if( (regS & PS_CARRY) )
    { pushPC(); regPC = MEM_READ(regPC-2) | (MEM_READ(regPC-1) << 8); PROF_ADD_CYCLES(17); }
  else
    PROF_ADD_CYCLES(11);
}

void cpu_CPO(byte opcode)
{
  regPC += 2;
  if( !(regS & PS_PARITY) )
    { pushPC(); regPC = MEM_READ(regPC-2) | (MEM_READ(regPC-1) << 8); PROF_ADD_CYCLES(17); }
  else
    PROF_ADD_CYCLES(11);
}

void cpu_CPE(byte opcode)
{
  regPC += 2;
  if( (regS & PS_PARITY) )
    { pushPC(); regPC = MEM_READ(regPC-2) | (MEM_READ(regPC-1) << 8); PROF_ADD_CYCLES(17); }
  else
    PROF_ADD_CYCLES(11);
}

void cpu_CP(byte opcode)
{
  regPC += 2;
  if( !(regS & PS_SIGN) )
    { pushPC(); regPC = MEM_READ(regPC-2) | (MEM_READ(regPC-1) << 8); PROF_ADD_CYCLES(17); }
  else
    PROF_ADD_CYCLES(11);
}

void cpu_CM(byte opcode)
{
  regPC += 2;
  if( (regS & PS_SIGN) )
    { pushPC(); regPC = MEM_READ(regPC-2) | (MEM_READ(regPC-1) << 8); PROF_ADD_CYCLES(17); }
  else
    PROF_ADD_CYCLES(11);
}

void cpu_SHLD(byte opcode)
{
  uint16_t addr = MEM_READ(regPC) | (MEM_READ(regPC+1) << 8);
  MEM_WRITE(addr,   regL);
  MEM_WRITE(addr+1, regH);
  regPC += 2;
  PROF_ADD_CYCLES(16);
}

void cpu_SPHL(byte opcode)
{
  regSP = (regH << 8) | regL;
  PROF_ADD_CYCLES(5);
}

void cpu_STA(byte opcode)
{
  uint16_t addr = MEM_READ(regPC) | (MEM_READ(regPC+1) << 8);
  MEM_WRITE(addr, regA);
  regPC += 2;
  PROF_ADD_CYCLES(13);
}

void cpu_STAX(byte opcode)
{
  byte dst  = (opcode&0070); dst /= 8;
  uint16_t addr = (Regs[dst] << 8) | Regs[dst+1];
  MEM_WRITE(addr, regA);
  PROF_ADD_CYCLES(7);
}

void cpu_STC(byte opcode)
{
  regS |= PS_CARRY;
  PROF_ADD_CYCLES(4);
}

void cpu_XTHL(byte opcode)
{
  byte b;
  b = MEM_READ(regSP+1); MEM_WRITE(regSP+1, regH); regH = b;
  b = MEM_READ(regSP);   MEM_WRITE(regSP,   regL); regL = b;
  PROF_ADD_CYCLES(18);
}

void cpu_XCHG(byte opcode)
{
  byte b;
  b = regD; regD = regH; regH = b;
  b = regE; regE = regL; regL = b;
  PROF_ADD_CYCLES(5);
}


void cpu_OUT(byte opcode)
{
  altair_out(MEM_READ(regPC), regA);
  PROF_ADD_CYCLES(10);
  regPC++;
}


void cpu_IN(byte opcode)
{
  regA = altair_in(MEM_READ(regPC));
  PROF_ADD_CYCLES(10);
  regPC++;
}


CPUFUN cpu_opcodes[256] = {
  cpu_NOP, cpu_LXI, cpu_STAX,cpu_INX, cpu_INR, cpu_DCR, cpu_MVI, cpu_RLC,		// 000-007 (0X00-0X07)
  cpu_NUL, cpu_DAD, cpu_LDAX,cpu_DCX, cpu_INR, cpu_DCR, cpu_MVI, cpu_RRC,		// 010-017 (0X08-0X0F)
  cpu_NUL, cpu_LXI, cpu_STAX,cpu_INX, cpu_INR, cpu_DCR, cpu_MVI, cpu_RAL,		// 020-027 (0X10-0X17)
  cpu_NUL, cpu_DAD, cpu_LDAX,cpu_DCX, cpu_INR, cpu_DCR, cpu_MVI, cpu_RAR,		// 030-037 (0X18-0X1F)
  cpu_NUL, cpu_LXI, cpu_SHLD,cpu_INX, cpu_INR, cpu_DCR, cpu_MVI, cpu_DAA,		// 040-047 (0X20-0X27)
  cpu_NUL, cpu_DAD, cpu_LHLD,cpu_DCX, cpu_INR, cpu_DCR, cpu_MVI, cpu_CMA,		// 050-057 (0X28-0X2F)
  cpu_NUL, cpu_LXIS,cpu_STA, cpu_INXS,cpu_INRM,cpu_DCRM,cpu_MVIM,cpu_STC,		// 060-067 (0X30-0X37)
  cpu_NUL, cpu_DADS,cpu_LDA, cpu_DCXS,cpu_INR, cpu_DCR, cpu_MVI, cpu_CMC,		// 070-077 (0X38-0X3F)
  
  cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MVR, cpu_MOV,		// 100-107 (0X40-0X47)
  cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MVR, cpu_MOV,		// 110-117 (0X48-0X4F)
  cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MVR, cpu_MOV,		// 120-127 (0X50-0X57)
  cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MVR, cpu_MOV,		// 130-137 (0X58-0X5F)
  cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MVR, cpu_MOV,		// 140-147 (0X60-0X67)
  cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MVR, cpu_MOV,		// 150-157 (0X68-0X6F)
  cpu_MVM, cpu_MVM, cpu_MVM, cpu_MVM, cpu_MVM, cpu_MVM, cpu_HLT, cpu_MVM,		// 160-167 (0X70-0X77)
  cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MOV, cpu_MVR, cpu_MOV,		// 170-177 (0X78-0X7F)
  
  cpu_ADD, cpu_ADD, cpu_ADD, cpu_ADD, cpu_ADD, cpu_ADD, cpu_ADDM,cpu_ADD,		// 200-207 (0X80-0X87)
  cpu_ADC, cpu_ADC, cpu_ADC, cpu_ADC, cpu_ADC, cpu_ADC, cpu_ADCM,cpu_ADC,		// 210-217 (0X88-0X8F)
  cpu_SUB, cpu_SUB, cpu_SUB, cpu_SUB, cpu_SUB, cpu_SUB, cpu_SUBM,cpu_SUB,		// 220-227 (0X90-0X97)
  cpu_SBB, cpu_SBB, cpu_SBB, cpu_SBB, cpu_SBB, cpu_SBB, cpu_SBBM,cpu_SBB,		// 230-237 (0X98-0X9F)
  cpu_ANA, cpu_ANA, cpu_ANA, cpu_ANA, cpu_ANA, cpu_ANA, cpu_ANAM,cpu_ANA,		// 240-247 (0XA0-0XA7)
  cpu_XRA, cpu_XRA, cpu_XRA, cpu_XRA, cpu_XRA, cpu_XRA, cpu_XRAM,cpu_XRA,		// 250-257 (0XA8-0XAF)
  cpu_ORA, cpu_ORA, cpu_ORA, cpu_ORA, cpu_ORA, cpu_ORA, cpu_ORAM,cpu_ORA, 	        // 260-267 (0XB0-0XB7)
  cpu_CMP, cpu_CMP, cpu_CMP, cpu_CMP, cpu_CMP, cpu_CMP, cpu_CMPM,cpu_CMP,		// 270-277 (0XB8-0XBF)
  
  cpu_RNZ, cpu_POP, cpu_JNZ, cpu_JMP, cpu_CNZ, cpu_PUSH,cpu_ADI, cpu_RST,		// 300-307 (0XC0-0XC7)
  cpu_RZ,  cpu_RET, cpu_JZ,  cpu_NUL, cpu_CZ,  cpu_CALL,cpu_ACI, cpu_RST,		// 310-317 (0XC8-0XCF)
  cpu_RNC, cpu_POP, cpu_JNC, cpu_OUT, cpu_CNC, cpu_PUSH,cpu_SUI, cpu_RST,		// 320-327 (0XD0-0XD7)
  cpu_RC,  cpu_NUL, cpu_JC,  cpu_IN,  cpu_CC,  cpu_NUL, cpu_SBI, cpu_RST,		// 330-337 (0XD8-0XDF)
  cpu_RPO, cpu_POP, cpu_JPO, cpu_XTHL,cpu_CPO, cpu_PUSH,cpu_ANI, cpu_RST,		// 340-347 (0XE0-0XE7)
  cpu_RPE, cpu_PCHL,cpu_JPE, cpu_XCHG,cpu_CPE, cpu_NUL, cpu_XRI, cpu_RST,		// 350-357 (0XE8-0XEF)
  cpu_RP,  cpu_POPA,cpu_JP,  cpu_DI,  cpu_CP,  cpu_PUSA,cpu_ORI, cpu_RST,		// 360-367 (0XF0-0XF7)
  cpu_RM,  cpu_SPHL,cpu_JM,  cpu_EI,  cpu_CM,  cpu_NUL, cpu_CPI, cpu_RST		// 370-377 (0XF8-0XFF)
  };

#endif
