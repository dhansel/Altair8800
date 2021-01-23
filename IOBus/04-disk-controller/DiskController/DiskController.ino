// -----------------------------------------------------------------------------
// Copyright (C) 2020 David Hansel
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

/*

Program as ATMega328P @16MHz
Fuse bytes (Arduino standard): LOW=0xFF, HIGH=0xDA, EXTENDED=0xFD


Arduino   Atmega Register   Direction  Function
RESET     1      PC6        in         RESET
D0        2      PD0        in/out     D0
D1        3      PD1        in/out     D1

D2        4      PD2        in/out     D2
D3        5      PD3        in/out     D3
D4        6      PD4        in/out     D4
VIn       7                            VCC
GND       8                            GND
-         9      PB6                   Crystal
-         10     PB7                   Crystal
D5        11     PD5        in/out     D5
D6        12     PD6        in/out     D6
D7        13     PD7        in/out     D7
D8        14     PB0        in         READ      (ICP1)
D9        15     PB1        out        WRITE     (OC1A)
D10       16     PB2        out        SR latch  (inputs)
D11       17     PB3        in         INDEX     (PCINT3/PCMSK0)
D12       18     PB4        out        SR Latch  (outputs)
D13       19     PB5        out        WAIT
VIn       20                           AVCC
ARef      21                           AREF
GND       22                           GND
A0        23     PC0        in         A0
A1        24     PC1        in         A1
A2        25     PC2        out        SR Clock  (inputs+outputs)
A3        26     PC3        out        SR Data   (inputs+outputs)
A4        27     PC4        in         INP       (PCINT12/PCMSK1)
A5        28     PC5        in         OUT       (PCINT13/PCMSK1)

Shift register (74HC595, output):
Output  Pin  Function
QA      15   DENSITY
QB      1    MOTOR 0 (HEADLOAD for Shugart SA800)
QC      2    SELECT 0
QD      3    STEP
QE      4    STEPDIR
QF      5    MOTOR 1  (HEADLOAD for Shugart SA800)
QG      6    WRITEGATE
QH      7    SELECT 1

Shift register (74HC165, input)
Input   Pin  Function
D0      11   DIP0
D1      12   DIP1
D2      13   DIP2
D3      14   DIP3
D4       3   MONITOR
D5       4   WRITEPROTECT
D6       5   DISKCHANGE
D7       6   TRACK0

DIP switch settings (A/B refer to physical drives on ribbon cable):
DIP1     on=Minidisk (5.25") system, off=8" disk system
DIP2     on=Swap drives A and B, off=don't swap
DIP3     on=drive B is SA-800, off=drive B is standard 5.25inch
DIP4     on=drive A is SA-800, off=drive A is standard 5.25inch

EEPROM setting flags (address 4):
Bit    Function
0      Drive A force sync after step
1      Drive A use relative soft-sector timing
2      Drive A TBD
3      Drvie B force sync after step
4      Drive B use relative soft-sector timing
5      Drive B TBD
6      Controller TBD
7      Controller TBD

*/

#include "EEPROM.h"

// optimize code for performance (speed)
#pragma GCC optimize ("-O2")


// 0 = monitor disabled
// 1 = always enter monitor at boot
// 2 = enter monitor if MONITOR button is pressed at boot
#define MONITOR 2


// maximum number of drives is 2, could be increased but that will require
// changing the function of the SELECTx/MOTORx output signals
#define MAX_DRIVES 2

// un-comment this if using a true double-density drive instead of using a
// high-density drive to read double-density disks - true DD drives are rare
// these days so this is disabled by default
//#define TRUE_DD


// un-comment to introduce debug pulses on certain pins:
// WRITEDATA (PB1, AtMega pin 15): goes LOW during SECTOR TRUE, for hard-sectored media short LOW pulse for sector 0
// SRDATA    (PC3, AtMega pin 26): goes LOW while reading sector data
//#define DEBUG


// pins 0-7 are hardwired to PD0-7 in functions busINP() and busOUT()
#define PIN_READDATA      8   // must be pin 8 (ICP1 for timer1)
#define PIN_WRITEDATA     9   // hardwired to PB1 in function write_sector_data()
#define PIN_SRILATCH     10   // hardwired to PB2 in function shift_in()
#define PIN_INDEX        11   // hardwired to PB3 (PCINT3)
#define PIN_SROLATCH     12   // hardwired to PB4 in function shift_out()
#define PIN_WAIT         13   // hardwired to PB5 in functions busINP() and busOUT()
#define PIN_A0           A0   // hardwired to PC0 in functions busINP() and busOUT()
#define PIN_A1           A1   // hardwired to PC1 in functions busINP() and busOUT()
#define PIN_SRCLOCK      A2   // hardwired to PC2 in function shift_out() 
#define PIN_SRDATA       A3   // hardwired to PC3 in function shift_out()
#define PIN_INP          A4   // hardwired to PC4 (PCINT12)
#define PIN_OUT          A5   // hardwired to PC5 (PCINT13)

// output shift register pins
#define PIN_SRO_DENSITY    0
#define PIN_SRO_MOTOR0     1
#define PIN_SRO_SELECT1    2
#define PIN_SRO_SELECT0    3
#define PIN_SRO_MOTOR1     4
#define PIN_SRO_STEP       5
#define PIN_SRO_WRITEGATE  6
#define PIN_SRO_SIDE1      7
#define PIN_SRO_SELECT     pinSelect[selDrive]
#define PIN_SRO_MOTOR      pinMotor[selDrive]

// input shift register pins
#define PIN_SRI_DIP1      0
#define PIN_SRI_DIP2      1
#define PIN_SRI_DIP3      2
#define PIN_SRI_DIP4      3
#define PIN_SRI_MONITOR   4
#define PIN_SRI_WRTPROT   5
#define PIN_SRI_DSKCHG    6
#define PIN_SRI_TRACK0    7
#define SRI_BIT_SET(pin)  (shift_in(8-(pin)) & 1)


// debug macros
#ifdef DEBUG
#define DBGPIN(port, bit, onOff)   if(onOff) port |= 1<<bit; else port &= ~(1<<bit)
#define DBGPULSE(port, bit)        {port &= ~(1<<bit); WAIT(16); port |= 1<<bit; WAIT(16);}
#else
#define DBGPIN(port, bit, onOff)   while(0)
#define DBGPULSE(port, bit)        while(0)
#endif


// common data buffer (for monitor and controller)
#define DATA_BUFFER_SIZE 1300
static byte dataBuffer[DATA_BUFFER_SIZE]; // must hold at least 9 sectors of 137 bytes 

// current content of shift register for drive control
static byte drivectrl;

// currently selected drive (0/1)
static byte selDrive = 0;

// SELECT/MOTOR bits in shift register for drive 0/1
static byte pinSelect[2], pinMotor[2];

// these must be volatile since they are used within the ASM section
volatile byte regStatus, regSector, foundIndex, readFinished;
volatile uint16_t indexTime, readDoneTime;


// -------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------   Input/Output shift registers   ---------------------------------------------
// -------------------------------------------------------------------------------------------------------------------------


// define WAIT assembler macro (creates assembly instructions to wait a specific number of cycles)
// IMPORTANT: clobbers r19!
asm (".macro WAIT cycles:req\n"
     "  .if \\cycles < 9\n"
     "    .rept \\cycles\n"
     "      nop\n" 
     "    .endr\n"
     "  .else\n"
     "    ldi   r19, \\cycles/3 \n"
     "    dec  	r19\n"
     "    brne	.-4\n"
     "    .if (\\cycles % 3)>0\n"
     "      nop\n"
     "      .if (\\cycles % 3)>1\n"
     "        nop\n"
     "      .endif\n"
     "    .endif\n"
     "  .endif\n"
     ".endm\n");


// to use the WAIT macro from within C
#define WAIT(cycles) asm volatile("WAIT " #cycles : : : "r19")


static byte shift_in(byte n = 8)
{
  // this implementation takes about 8us to read all 8 bits
  byte res = 0;
  DDRC  &= ~0x08;  // switch "SR Data" pin to input
  PORTC &= ~0x04;  // set clock low
  PORTB |=  0x04;  // latch inputs

  for(byte i=0; i<n; i++)
    {
      WAIT(4);                       // 4 cycles = 250ns delay
      res = res * 2;                 // shift result
      if( PINC & 0x08 ) res |= 1;    // read bit
      PORTC |= 0x04; PORTC &= ~0x04; // pulse clock
    }
  
  PORTB &= ~0x04;  // release inputs
  DDRC  |=  0x08;  // switch "SR Data" pin back to output
  return res;
}


static void shift_out(byte data, bool delay1us = false)
{
  // this implementation takes about 2us to write the shift register (3 with delay)
  register byte cd = PORTC & ~0x0C;  // cd = clock LOW  data LOW
  register byte cD = cd | 0x08;      // cD = clock LOW  data HIGH
  register byte Cd = cd | 0x04;      // Cd = clock HIGH data LOW
  register byte CD = cd | 0x0C;      // CD = clock HIGH data HIGH
  
  // the shift register reads data on the clock LOW->HIGH edge
  PORTB &= ~0x10;
  if( data & 0x80 ) { PORTC = cD; PORTC = CD; } else { PORTC = cd; PORTC = Cd; }
  if( data & 0x40 ) { PORTC = cD; PORTC = CD; } else { PORTC = cd; PORTC = Cd; }
  if( data & 0x20 ) { PORTC = cD; PORTC = CD; } else { PORTC = cd; PORTC = Cd; }
  if( data & 0x10 ) { PORTC = cD; PORTC = CD; } else { PORTC = cd; PORTC = Cd; }
  if( data & 0x08 ) { PORTC = cD; PORTC = CD; } else { PORTC = cd; PORTC = Cd; }
  if( data & 0x04 ) { PORTC = cD; PORTC = CD; } else { PORTC = cd; PORTC = Cd; }
  if( data & 0x02 ) { PORTC = cD; PORTC = CD; } else { PORTC = cd; PORTC = Cd; }
  if( data & 0x01 ) { PORTC = cD; PORTC = CD; } else { PORTC = cd; PORTC = Cd; }
  if( delay1us ) WAIT(16);  // 16 cycles = 1us delay
  PORTB |= 0x10;
}


inline void drivectrl_set(byte pin, byte state)
{
  if( state )
    drivectrl |= 1<<pin;
  else
    drivectrl &= ~(1<<pin);
  
  shift_out(drivectrl);
}


static void drivectrl_step_pulse(bool stepOut)
{
  // Take STEP line LOW
  shift_out(drivectrl & ~bit(PIN_SRO_STEP));

  // Push the step direction into the first bit of the shift register.
  // After the 8 more shifts done by shift_out() this will end up in the
  // flip-flop that controls the STEPDIR line to the drive
  PORTC &= ~0x04; // clock low
  if( stepOut ) PORTC |= 0x08; else PORTC &= ~0x08;
  PORTC |=  0x04; // clock high

  // Take the STEP line back HIGH, the LOW->HIGH edge on STEP will 
  // trigger the drive to step.
  // Calling shift_out with "wait"=true introduces a 1us delay
  // between the final shift and switching the shift register LATCH high.
  // With the final shift, the 7474 flip-flop (STEPDIR) receives its 
  // output value. The shift register updates its outputs when LATCH
  // goes high. The delay ensures that the STEPDIR signal is stable
  // 1us before the STEP signal edge, as per drive requirements.
  shift_out(drivectrl | bit(PIN_SRO_STEP), true);

  // The STEPDIR flip-flop will change its output during any shift
  // register operation. The drive requires that the step direction 
  // be stable for 5us after initiating the step pulse
  // => delay here to be sure.
  WAIT(5*16); // 5*16 cycles = 5us delay
}


// -------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------  Drive parameters  -------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------------


// supported drive types
#define DT_NONE      0  // no drive
#define DT_5INCH_DD  1  // generic 5.25" drive in DD mode (emulating MITS MiniDisk)
#define DT_5INCH_HD  2  // generic 5.25" drive in HD mode (emulating 8")
#define DT_SA800     3  // 8" Shugart SA-800 drive

// supported spindle rotation standards
#define RPM_300 0
#define RPM_360 1

#define DF_FORCESYNC 1  // force waiting for INDEX hole after stepping
#define DF_RELSECTOR 2  // use relative sector timing instead of absolute (timer-based)

byte driveType[MAX_DRIVES];              // drive type (5 inch/8 inch)
byte driveFlags[MAX_DRIVES];             // drive flags (see DF_* constants)
byte numSectors[MAX_DRIVES];             // number of sectors per track (16 or 32)
byte numTracks[MAX_DRIVES];              // number of tracks (35 or 77)
byte rpm[MAX_DRIVES];                    // rotations per minute (RPM_300 or RPM_360)
bool hardSectored[MAX_DRIVES];           // whether disk is hard or soft-sectored
unsigned int sectorLength[MAX_DRIVES];   // time in microseconds per sector (0=unknown)
unsigned int sectorOffset[MAX_DRIVES];   // time in microseconds each sector is offset from its physical start
unsigned int sectorTail[MAX_DRIVES];     // time in microseconds after sector data ends until start of next sector
unsigned int readClear[MAX_DRIVES];      // read-clear period, see function set_sector_length()


void print_drive_type(byte tp)
{
  switch( tp )
    {
    case DT_NONE     : Serial.print(F("none"));   break;
    case DT_5INCH_DD : Serial.print(F("5.25\" DD emulating Altair 88-MDS (Minidisk system)")); break;
    case DT_5INCH_HD : Serial.print(F("5.25\" HD emulating Altair 88-DCDD (8\" disk system)")); break;
    case DT_SA800    : Serial.print(F("Shugart SA800 (8\")"));    break;
    default          : Serial.print(F("unknown"));   break;
    }
}


void print_drive_type_drive(byte drive)
{
  Serial.print(F("Drive ")); Serial.print(drive);
  Serial.print(F(" is type "));
  Serial.print(driveType[drive]); Serial.print(F(": "));
  print_drive_type(driveType[drive]);
  if( (driveFlags[drive] & DF_FORCESYNC)!=0 ) Serial.print(F(" (forced sync after step)"));
  if( (driveFlags[drive] & DF_RELSECTOR)!=0 ) Serial.print(F(" (relative soft-sector timing)"));
  Serial.println();
}



void print_drive_params(byte drive)
{
  Serial.print(F("Type              : "));
  print_drive_type(driveType[drive]); Serial.println();
  Serial.print(F("Sync after step   : "));
  Serial.println((driveFlags[drive] & DF_FORCESYNC)!=0 ? F("yes") : F("no"));
  Serial.print(F("Number of Tracks  : ")); 
  Serial.println(numTracks[drive]);
  Serial.print(F("Number of Sectors : ")); 
  Serial.println(numSectors[drive]);
}


void print_drive_timing(byte drive)
{
  Serial.print(F("Rotation speed    : "));
  Serial.print(rpm[drive]==RPM_300 ? 300 : 360);
  Serial.println(F(" RPM"));
  Serial.print(F("Sector length     : "));
  Serial.print(sectorLength[drive]);
  Serial.print(F("us "));
  if( hardSectored[drive] )
    Serial.println("(hard-sectored)");
  else
    Serial.println("(soft-sectored)");
  Serial.print(F("Sector offset     : "));
  Serial.print(sectorOffset[drive]);
  Serial.println(F("us "));
  Serial.print(F("Sector tail       : "));
  Serial.print(sectorTail[drive]);
  Serial.println(F("us "));
  Serial.print(F("Read clear        : "));
  Serial.print(readClear[drive]);
  Serial.println(F("us"));
  Serial.print(F("Write protected   : "));
  Serial.println(SRI_BIT_SET(PIN_SRI_WRTPROT) ? F("no") : F("yes"));
}


void set_drive_type(byte drive, byte tp, bool swapped)
{
  driveType[drive] = tp;
  
  switch( tp )
    {
    case DT_5INCH_DD:
      numSectors[drive] = 16;
      numTracks[drive]  = 35;
      break;

    case DT_5INCH_HD:
    case DT_SA800:
      numSectors[drive] = 32;
      numTracks[drive]  = 77;
      break;
    }

  byte flags = EEPROM.read(4);
  if( (drive==1) ^ swapped ) flags = flags >> 3;

  driveFlags[drive] = flags & 0x07;

  if( Serial ) print_drive_type_drive(drive);
}


void set_sector_length(byte drive, unsigned long rotationPeriod)
{
  if( rotationPeriod < 20000 )
    {
      // rotationPeriod is SIGNIFICANTLY too short for a full rotation
      // => must be hard-sectored media (measuring sectors instead of full rotations)
      rotationPeriod = driveType[selDrive]==DT_5INCH_DD ? 200000 : 166666;
      hardSectored[selDrive] = true;
    }
  else
    hardSectored[selDrive] = false;

  sectorLength[drive] = rotationPeriod/numSectors[drive];
  rpm[drive] = rotationPeriod > 180000 ? RPM_300 : RPM_360;

  if( driveType[drive]==DT_5INCH_DD )
    {
      // "read clear" for 5.25" minidisk (at 300RPM) is 500 microseconds
      // we don't know whether this is a 5.25" or 3.5" drive since both
      // run at 300RPM for DD so we just use 500 microseconds for both
      readClear[drive] = 500;
    }
  else
    {
      // "read clear" for 8" and 5.25" HD disk (at 360RPM) is 200 microseconds,
      // for 3.5" HD drives (at 300RPM) we use 400 microseconds
      readClear[drive] = rpm[drive]==RPM_300 ? 400 : 200;
    }

  sectorOffset[drive] = 0;
  if( driveType[drive]==DT_SA800 )
    {
      // for some reason the sector start on a Shugart SA-800 drive is
      // 360us earlier compared to the Pertec drive used by MITS' 88-DCDD
      sectorOffset[drive] += 360;
    }

  if( !hardSectored[drive] )
    {
      // physical start of sector 0 for a soft-sectored disk would be at the
      // track-start index hole but is shifted by half a sector length to be
      // consistent with the layout of a hard-sectored disk (where the track-start
      // index hole sits between two sector-start index holes)
      sectorOffset[drive] += sectorLength[drive] / 2;
    }
  
  // need at least 5us offset, otherwise setting OCR1A in function 
  // handle_index_signal() might not produce a compare match.
  // note that the index hole signal is not  precise to the microsecond 
  // anyways (also all currently supported formats have sector offsets >100us)
  sectorOffset[drive] = max(5, sectorOffset[drive]);

  // calculate tima after sector data ends until beginning of next sector
  sectorTail[drive] = sectorLength[drive] - 2*readClear[drive] - 137*8*(driveType[selDrive]==DT_5INCH_DD ? 8 : 4);
}


// -------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- low-level FM read/write functions ----------------------------------------
// -------------------------------------------------------------------------------------------------------------------------


// see comment in write_sector_data()
static void write_sector_data_hd(const byte *ptr, byte n, byte sector_start_offset)
{
  // calculate how many lead-in bits we have time for. sector_start_offset
  // is microseconds since the sector started (should never be more than read clear period)
  // sector data (sync bit) should start after twice the "read clear" period.
  // each lead-in bit takes 4 microseconds for HD.
  byte li = (readClear[selDrive]*2)/4 - (sector_start_offset/4);

  // make sure WRITE signal is high before we enable write gate
  PORTB |= 0x02;

  // enable write gate (takes about 2 microseconds)
  drivectrl_set(PIN_SRO_WRITEGATE, LOW);

  asm volatile 
    (  // --- initialize
       "        movw    x,   z\n"         // "x" is buffer write pointer, "z" is buffer read pointer
       "        mov     r23, %1\n"        // number of data bytes to receive from CPU
       "        mov     r24, %2\n"        // number of lead-in "0" bits to output
       "        ldi     r25, 0\n"         // all zero bits
       "        inc     %1\n"             // must add 1 to number of bytes (for lead-in bits)
       "        ldi     r16, 0\n"         // bit 1: unfinished bus activity, bit 3: INDEX hole found
       "        ldi     r18, 0xFF\n"      // used when changing bus to OUTPUT
       "        lds     r20, regStatus\n" // get regStatus into r20 for quick access (does not change)
       "        ori     r20, 0x02\n"      // head movement not allowed while writing
       "        andi    r20, 0xFE\n"      // clear ENWD flag (ready to receive data)
       "        lds     r21, regSector\n" // get regStatus into r21 for quick access (does not change)
       "        ori     r21, 0x01\n"      // sector is not true while in here

       // --- write bits loop (64 clock cycles each = 4us per bit)

       // -----------------------------  first half of bit write (2+27+3=32 cycles)

       "nxtbit:	cbi    0x05, 1\n"        // (2)   set "write data" line low (start of clock pulse)
       // (can't set "write data" back high immediately since SA-800 drive requires pulse 
       // length of at least 150ns, 2 cycles are only 125ns)

       // --- wait 32-2-3=27 cycles (use this time to handle bus communication with CPU)

       "LW1:    sbic   0x1b, 0\n"        // (1/2) skip next instruction if PCIF0 is clear
       "        rjmp   LW17\n"           // (2)   PCIF1 set => INDEX signal pin change detected
       "	sbi    0x05, 1\n"        // (2)   set "write data" signal back to HIGH
       "        sbrc   r16, 1\n"         // (1/2) is there unfinished bus activity?
       "        rjmp   LW9\n"            // (2)   yes, try to finish it
       "        sbic   0x06, 5\n"        // (1/2) skip next instruction if PC5 clear
       "        rjmp   LW5\n"            // (2)   PC5 set => found OUT
       "        sbis   0x06, 4\n"        // (1/2) skip next instruction if PC4 set
       "        rjmp   LW11\n"           // (2)   PC4 clear => no bus activity

       // PC4 set => found INP signal (27-10=17 cycles left)
       "        sbic   0x06, 1\n"        // (1/2) address bit 1 (PC0) clear?
       "        rjmp   LWi2\n"           // (2)   => no, reading "data in" register (2 or 3)
       "        sbic   0x06, 0\n"        // (1/2) address bit 0 (PC0) clear?
       "        rjmp   LWi1\n"           // (2)   => no, reading "sector" register (1)

       // reading "status" (0) register (27-14=13 cycles left)
       "        out    0x0B, r20\n"      // (1) set output data on PORTD
       "        out    0x0A, r18\n"      // (1) switch PORTD to output
       "        cbi    0x05, 5\n"        // (2) set WAIT signal to LOW
       "        ori    r16,  2\n"        // (1) set flag for unfinished bus activity
       "        WAIT   6\n"              // (6) wait for 13-7=6 cycles
       "        rjmp   LW13\n"           // (2)

       // reading "sector" (1) register (27-15=12 cycles left)
       "LWi1:   out    0x0B, r21\n"      // (1)  set output data on PORTD
       "        out    0x0A, r18\n"      // (1)  switch PORTD to output
       "        cbi    0x05, 5\n"        // (2)  set WAIT signal to LOW
       "        ori    r16,  2\n"        // (1)  set flag for unfinished bus activity
       "        WAIT   5\n"              // (5)  wait for 12-7=5 cycles
       "        rjmp   LW13\n"           // (2) 

       // reading "data in" (2 or 3) register (27-13=14 cycles left)
       "LWi2:   out    0x0B, r18\n"      // (1)  output 0xFF
       "        out    0x0A, r18\n"      // (1)  switch PORTD to output
       "        cbi    0x05, 5\n"        // (2)  set WAIT signal to LOW
       "        ori    r16,  2\n"        // (1)  set flag for unfinished bus activity
       "        WAIT   7\n"              // (7)  wait for 14-7=7 cycles
       "        rjmp   LW13\n"           // (2) 

       // found OUT signal (27-9=18 cycles left)
       "LW5:    in     r19, 0x06\n"      // (1)   read address
       "        in     r17, 0x09\n"      // (1)   read data from bus
       "        cbi    0x05, 5\n"        // (2)   set WAIT signal to LOW
       "        ori    r16,  2\n"        // (1)   set flag for unfinished bus activity
       "        sbrc   r19, 0\n"         // (1/2) address bit 0 (PC0) clear?
       "        rjmp   LWo1\n"           // (2)   => no, writing "control" register (1 or 3)
       "        sbrs   r19, 1\n"         // (1/2) address bit 1 (PC1) set?
       "        rjmp   LWo0\n"           // (2)   => no, writing "drive select" register (0)

       // writing "data out" (2) register (18-9=9 cycles left)
       "        or     r23, r23\n"       // (1)   if we have already received as...
       "        breq   LW6\n"            // (1/2) ...many bytes as we need then ignore 
       "        st     X+, r17\n"        // (2)   store received data byte
       "        dec    r23\n"            // (1)   decrement receive byte counter
       "        WAIT   2\n"              // (2)   wait for 9-7=2 cycles
       "        rjmp   LW13\n"           // (2)   done
       "LW6:    WAIT   4\n"              // (4)   wait for 9-5=4 cycles
       "        rjmp   LW13\n"           // (2)

       // writing "control" (1) register - ignore (18-8=10 cyles left)
       "LWo1:   WAIT   8\n"              // (8)   wait 10-2=8 cycles
       "        rjmp   LW13\n"           // (2)   done

       // writing "drive select" (0) register - ignore (18-10=8 cyles left)
       "LWo0:   WAIT   6\n"              // (6)   wait 8-2=6 cycles
       "        rjmp   LW13\n"           // (2)   done

       // found INDEX signal pin change (27-3=24 cycles left)
       // note that this happens AT MOST twice per write cycle so it can not
       // starve out the communication
       "LW17:   sbi    0x05, 1\n"        // (2)   set "write data" signal back to HIGH
       "        sbi    0x1b, 0\n"        // (2)   clear PCIF0
       "        sbic   0x03, 3\n"        // (1/2) skip next instruction if PB3 is clear
       "        rjmp   LW17a\n"          // (2)   INDEX signal went HIGH (ignore)
       "        lds    r10, 0x84\n"      // (2)   get current TCNT1L
       "        lds    r11, 0x85\n"      // (2)   get current TCNT1H
       "        ori    r16, 8\n"         // (1)   set flag for "index signal seen"
       "        WAIT   11\n"             // (11)  wait for 24-11-2=11 cycles
       "        rjmp   LW13\n"           // (2)
       "LW17a:  WAIT   15\n"             // (15)  wait for 24-7-2=15 cycles
       "        rjmp   LW13\n"           // (2)

       // found no bus activity (27-11=16 cycles left)
       "LW11:   WAIT   14\n"             // (14) wait for 16-2=14 cycles
       "        rjmp   LW13\n"           // (2)
      
       // have unfinished bus activity (27-5=20 cycles left)
       "LW9:    in     r17, 0x06\n"      // (1)   read PINC
       "        andi   r17, 0x30\n"      // (1)   get bits 4+5
       "        brne   LW12\n"           // (1/2) if either one is set then we can't finish yet
       "        ldi    r17,  0\n"        // (1)   switch PORTD back to input
       "        out    0x0A, r17\n"      // (1)   (in case it was set to output)
       "        sbi    0x05, 5\n"        // (2)   set WAIT signal back to HIGH
       "        sbi    0x1b, 1\n"        // (2)   reset INP/OUT pin change flag (PCIFR & bit(PCI1F1))
       "        andi   r16,  253\n"      // (1)   clear flag for unfinished bus activity
       "        WAIT   8\n"              // (8)   wait for 20-10-2=8 cycles
       "        rjmp   LW13\n"           // (2)
       "LW12:   WAIT   14\n"             // (14)  wait for 20-4-2=14 cycles
       "        rjmp   LW13\n"           // (2)

       // --- check whether to output data pulse (3 cycles)
       "LW13:  	sbrc	r25, 7\n"        // (1/2) check bit 7 of r25, skip next instruction if clear
       "	rjmp	bit1\n"          // (2)   bit is set => output data pulse ("1" bit)
       "        WAIT    1\n"             // (3)   do not output pulse => wait to equalize timing
       "	rjmp	bit0\n"          // (2)   do not output pulse ("0" bit)

       // -----------------------------  second half of bit write (4+17+11 = 32 cycles)

       // --- output data pulse (2 cycles)
       "bit1:	cbi    0x05, 1\n"        // (2)   set write data line to LOW ("1" bit)

       // --- wait 32-2-11=19 cycles (use this time to communicate)
       "bit0:   sbrc   r16, 1\n"         // (1/2) is there unfinished bus activity?
       "        rjmp   LW14\n"           // (2)   yes => try to finish it
       "	sbi    0x05, 1\n"        // (2)   write data line back to HIGH
       "        sbic   0x06, 5\n"        // (1/2) skip next instruction if PC5 clear
       "        rjmp   LW5b\n"           // (2)   PC5 set => found OUT
       "        sbis   0x06, 4\n"        // (1/2) skip next instruction if PC4 set
       "        rjmp   LW15b\n"          // (2)   PC4 clear => no bus activity

       // PC4 set => found INP signal (19-8=11 cycles left)
       "        in     r19, 0x06\n"      // (1)   get address
       "        andi   r19, 0x03\n"      // (1)   isolate bottom 2 bits
       "        brne   LWix\n"           // (1/2) if not zero then skip

       // reading "status" (0) register (11-3=8 cycles left)
       "        out    0x0B, r20\n"      // (1) set output data on PORTD
       "        out    0x0A, r18\n"      // (1) switch PORTD to output
       "        cbi    0x05, 5\n"        // (2) set WAIT signal to LOW
       "        ori    r16,  2\n"        // (1) set flag for unfinished bus activity
       "        WAIT   1\n"              // (1) wait 8-7=1 cycles
       "        rjmp   LW16\n"           // (2)

       // reading other register (11-4=7 cycles left)
       // (ignore here, will be handled in first half of bit write cycle)
       "LWix:   WAIT   4\n"              // (1) wait 6-2=4 cycles
       "        rjmp   LW16\n"           // (2)

       // have unfinished bus activity (19-3=16 cycles left)
       "LW14:   sbi    0x05, 1\n"        // (2)   write data line back to HIGH
       "        in     r17, 0x06\n"      // (1)   read PINC
       "        andi   r17, 0x30\n"      // (1)   get bits 4+5
       "        brne   LW15\n"           // (1/2) if either one is set then we can't finish yet
       "        ldi    r17,  0\n"        // (1)   switch PORTD back to input
       "        out    0x0A, r17\n"      // (1)   (in case it was set for output)
       "        sbi    0x05, 5\n"        // (2)   set WAIT signal back to HIGH
       "        sbi    0x1b, 1\n"        // (2)   reset INP/OUT pin change flag (PCIFR & bit(PCI1F1))
       "        andi   r16,  253\n"      // (1)   clear flag for unfinished bus activity
       "        WAIT   2\n"              // (2)   wait for 16-12=2 cycles
       "        rjmp   LW16\n"           // (2)

       // can't finish bus activity yet (14-4=10 cycles left)
       "LW15:   WAIT   8\n"              // (8)   wait 10-2=8 cycles
       "        rjmp   LW16\n"           // (2)   
 
       // found no bus activity (19-9=10 cycles left)
       "LW15b:  WAIT   8\n"              // (8)   wait for 10-2=8 cycles
       "        rjmp   LW16\n"           // (2)
      
       // CPU writing other register (12-3=9 cycles left)
       // (ignore here, will be handled in first half of bit write cycle)
       "LWox:   WAIT   7\n"              // (7)   wait for 9-2=7 cycles
       "        rjmp   LW16\n"

       // PC5 set => found OUT signal (19-7=12 cycles left)
       "LW5b:   sbis   0x06, 1\n"        // (1/2) is address bit 1 (PC1) set?
       "        rjmp   LWox\n"           // (2)   no => writing other register

       // CPU writing "data out" (2) register (12-2=10 cycles left)
       "        in     r17, 0x09\n"      // (1)   read data from bus
       "        cbi    0x05, 5\n"        // (2)   set WAIT signal to LOW
       "        ori    r16,  2\n"        // (1)   set flag for unfinished bus activity
       "        or     r23, r23\n"       // (1)   if we have not yet received all bytes...
       "        brne   LW6b\n"           // (1/2) ...then store this one
       "        WAIT   5\n"              // (5)   wait for 12-7=5 cycles
       "        rjmp   LW16\n"           // (2)
       "LW6b:   st     X+, r17\n"        // (2)   store received data byte
       "        dec    r23\n"            // (1)   decrement receive byte counter

       // --- prepare for next bit/byte (11 cycles)
       "LW16:   subi	r24, 1\n"        // (1)   subtract 1 from bit counter
       "        breq	nxtbyt\n"        // (1/2) jump if no nore bits
       "        add	r25, r25\n"      // (1)   shift r25 left one bit
       "        WAIT    6\n"             // (6)   to equalize timing
       "        rjmp	nxtbit\n"        // (2)   next bit
       "nxtbyt: ld	r25, Z+\n"       // (2)   get next byte
       "        ldi     r24, 8\n"        // (1)   want to output 8 bits
       "        subi	%1, 1\n"         // (1)   subtract 1 from byte counter
       "        breq	wdone\n"         // (1/2) check if done
       "        WAIT    1\n"             // (1)
       "        rjmp    nxtbit\n"        // (2)   next bit

       // -----------------------------  end of bit write

       // --- write one final clock pulse
       "wdone:  WAIT    2\n"             // (2)   to equalize timing
       "        cbi	0x05, 1\n"       // (2)   output pulse on bit 1 of PORTB
       "        WAIT    3\n"             // (3)   wait 3 cycles
       "	sbi	0x05, 1\n"       // (2)

       // --- finish potential unfinished bus activity
       "        sbrs    r16, 1\n"        // (1/2) do we have unfinished bus activity?
       "        rjmp    wdone2\n"        // (2)   no => done
       "wbus:   in      r17, 0x06\n"     // (1)   read PINC
       "        andi    r17, 0x30\n"     // (1)   mask bits 4+5
       "        brne    wbus\n"          // (1/2) if either one is set then wait
       "        ldi     r17,  0\n"       // (1)   switch PORTD...
       "        out     0x0A, r17\n"     // (1)   back to input
       "        sbi     0x05, 5\n"       // (2)   set WAIT signal back to HIGH
       "        sbi     0x1b, 1\n"       // (2)   reset INP/OUT pin change flag (PCIFR & bit(PCI1F1))
       "wdone2: andi    r16, 8\n"            // set "foundIndex" variable high if we have seen 
       "        sts     foundIndex,  r16\n"  // a high->low edge on INDEX signal
       "        sts     indexTime+0, r10\n"  // set indexTime variable to TCNT1 value
       "        sts     indexTime+1, r11\n"  // when index hole was seen

       :                                  // no outputs
       : "z"(ptr), "d"(n), "r"(li)        // inputs 
       : "r10", "r11", "r16", "r17", "r18", "r19", "r20", "r21", "r23", "r24", "r25", "r26", "r27"); // clobbers (x=r26/r27)

  // disable write gate
  drivectrl_set(PIN_SRO_WRITEGATE, HIGH);
}


// see comment in write_sector_data()
static void write_sector_data_dd(const byte *ptr, byte n, byte sector_start_offset)
{
  // calculate how many lead-in bits we have time for. sector_start_offset
  // is microseconds since the sector started (should never be more than read clear period)
  // sector data (sync bit) should start after twice the "read clear" period.
  // each lead-in bit takes 8 microseconds for DD.
  byte li = (readClear[selDrive]*2)/8 - (sector_start_offset/8);

  // make sure OC1A is high before we enable WRITE_GATE
  DDRB   &= ~0x02;                     // disable OC1A pin
  TCCR1A  = bit(COM1A1) | bit(COM1A0); // set OC1A on compare match
  TCCR1C |= bit(FOC1A);                // force compare match
  TCCR1A  = 0;                         // disable OC1A control by timer
  DDRB   |= 0x02;                      // enable OC1A pin

  // enable write gate (takes about 2 microseconds)
  drivectrl_set(PIN_SRO_WRITEGATE, LOW);

  // enable OC1A output pin control by timer (WRITE_DATA), initially high
  uint16_t OCR1Aprev = OCR1A;
  OCR1A  = TCNT1 + 2*4;   // first pulse in 4 microseconds
  TIFR1  = bit(OCF1A);    // clear output compare flag
  TCCR1A = bit(COM1A0); // COM1A1:0 =  01 => toggle OC1A on compare match

  asm volatile 
    (  // --- initialize
       "        movw    x,   z\n"         // "x" is buffer write pointer, "z" is buffer read pointer
       "        mov     r23, %1\n"        // number of data bytes to receive from CPU
       "        mov     r24, %2\n"        // number of lead-in "0" bits to output
       "        ldi     r25, 0\n"         // all zero bits
       "        inc     %1\n"             // must add 1 to number of bytes to send (for lead-in bits)
       "        lds     r16, regStatus\n" // regStatus does not change while in here
       "        ori     r16, 0x02\n"      // head movement not allowed while writing
       "        andi    r16, 0xFE\n"      // clear ENWD flag (ready to receive data)
       "        mov     r13, r16\n"       // move status to r13 for quick access
       "        lds     r16, regSector\n" // regSector does not change while in here
       "        ori     r16, 0x01\n"      // sector is not true while in here
       "        mov     r14, r16\n"       // move sector to r14 for quick access
       "        ldi     r16, 0xFF\n"      // 
       "        mov     r18, r16\n"       // r18 must be 0xFF
       "        ldi     r16, 0\n"         // 
       "        mov     r15, r16\n"       // r15 must be 0
       "        ldi     r16, 1\n"         // bit 0: sent clock pulse, bit 1: unfinished bus activity, bit 3: INDEX hole found
       "        lds     r20, 0x0088\n"    // read OCR1AL
       "        lds     r21, 0x0089\n"    // read OCR1AH

       // ===========================  start of write loop

       "LWD0:   sbis   0x16, 1\n"        // (1/2) check if pulse write timer expired (OCF1A set in TIFR1)
       "        rjmp   LWD5\n"           // (2)   no => check for bus activity

       // ===========================   pulse write timer expired (max 28 cycles)

       "        ldi    r19, 0x80\n"      // (1)   switch OCP1 pin back high
       "        sts    0x82, r19\n"      // (2)   (by setting FOC1A in TCCR1C)
       "        sbrc   r16, 0\n"         // (1/2) was this a clock pulse?
       "        rjmp   LWD1\n"           // (2)   yes => prepare for possible data pulse

       // just wrote a data pulse => prepare follwing clock pulse
       "        ori    r16, 1\n"         // (1)   next pulse is clock
       "        ldi    r19, 8\n"         // (1)   4us to next (clock) pulse
       "        rjmp   LWD4\n"           // (2)   update OCR1A

       // just wrote a clock pulse => get next data bit ready
       "LWD1:   add    r25, r25\n"       // (1)   shift data byte left one bit
       "	subi   r24, 1\n"         // (1)   subtract 1 from bit counter
       "        brne   LWD2\n"           // (1/2) jump if more bits in current byte
       "        ld     r25, Z+\n"        // (2)   get next byte
       "        ldi    r24, 8\n"         // (1)   want to output 8 bits
       "        subi   %1, 1\n"          // (1)   subtract 1 from byte counter
       "        breq   LWD4a\n"          // (1/2) jump if done sending all bytes

       // decide whether next pulse is clock ("0" data bit) or data ("1" data bit)
       // (worst case 14 cycles so far)
       "LWD2:   sbrc   r25, 7\n"         // (1/2) check bit 7 of data byte, skip next instruction if clear
       "        rjmp   LWD3\n"           // (2)   next data bit is "1" bit
       "	ldi    r19, 16\n"        // (1)   next data bit is "0" => 8us to next (clock) pulse
       "        rjmp   LWD4\n"           // (2)   update OCR1A
       "LWD3:   ldi    r19, 8\n"         // (1)   next data bit is "1" => 4us to next (data) pulse
       "        andi   r16, 254\n"       // (1)   => next bit is data bit

       // update OCR1A for next pulse (current OCR1A value is in r20/r21, number of cycles to add is in r19)
       // (worst case 19 cycles so far)
       // the longest path through the code from here back here is 9+3+24+2+19=57 cycles,
       //  the 4us * 16cyles/us = 64 cycles minimum time between two bits
       // (but too much for the 32 cycle time between bits for writing HD)
       "LWD4:   add    r20, r19\n"       // (1)   add number of cycles
       "        adc    r21, r15\n"       // (1)   add 0 with carry
       "        sts    0x89, r21\n"      // (2)   write OCR1AH
       "        sts    0x88, r20\n"      // (2)   write OCR1AL
       "        sbi    0x16, 1\n"        // (1)   clear output compare flag (set OCF1A in TIFR1)
       "        rjmp   LWD0\n"           // (2)   done (worst case 28 cycles)
       
       // used as a hop when jumping to end of write loop
       "LWD4a:  rjmp    LWD10\n"         // (2)

       // ===========================   check INDEX pin signal change and bus activity (max 27 cycles)

       // note that this happens AT MOST twice per write cycle so it can not
       // starve out the bus communication
       "LWD5:   sbis    0x1b, 0\n"       // (1/2) skip next instruction if PCIF0 is set
       "        rjmp    LWD5b\n"         // (2)   no INDEX signal pin change => check bus activity
       "        sbi     0x1b, 0\n"       // (1)   clear PCIF0
       "        sbic    0x03, 3\n"       // (1/2) skip next instruction if PB3 is clear
       "        rjmp    LWD0\n"          // (2)   INDEX signal went HIGH => back to start (6 cycles)
       "        lds     r10, 0x84\n"     // (2)   get current TCNT1L
       "        lds     r11, 0x85\n"     // (2)   get current TCNT1H
       "        ori     r16, 8\n"        // (1)   set flag for "index signal seen"
       "        rjmp    LWD0\n"          // (2)   back to start (12 cycles)

       // ===========================   handle bus activity (max 24 cycles)

       "LWD5b:  sbrc   r16, 1\n"         // (1/2) is there unfinished bus activity?
       "        rjmp   LWD9\n"           // (2)   yes, try to finish it
       "        sbic   0x06, 5\n"        // (1/2) skip next instruction if PC5 clear
       "        rjmp   LWD8\n"           // (2)   PC5 set => found OUT
       "        sbis   0x06, 4\n"        // (1/2) skip next instruction if PC4 set
       "        rjmp   LWD0\n"           // (2)   PC4 clear => no bus activity, done (7 cycles)
       // PC4 set => found INP signal (6 cycles so far)
       "        sbic   0x06, 1\n"        // (1/2) address bit 1 (PC0) clear?
       "        rjmp   LWD7\n"           // (2)   => no, reading "data in" register (2 or 3)
       "        sbic   0x06, 0\n"        // (1/2) address bit 0 (PC0) clear?
       "        rjmp   LWD6\n"           // (2)   => no, reading "sector" register (1)

       // reading "status" (0) register (10 cycles so far)
       "        mov    r19, r13\n"       // (1)   get status register value
       "        mov    r17, %1\n"        // (1)   get number of bytes left to write
       "        sub    r17, r23\n"       // (1)   subtract number of bytes left to receive
       "        brlo   LWD5a\n"          // (1/2) jump if more bytes written than received
       "        cpi    r17, 11\n"        // (1)   have we received 10+ more bytes than written?
       "        brlo   LWD5a\n"          // (1/2) jump if not
       "        ori    r19,  1\n"        // (1)   yes, not ready to receive (limit buffering)
       "LWD5a:  out    0x0B, r19\n"      // (1)   set output data on PORTD
       "        out    0x0A, r18\n"      // (1)   switch PORTD to output
       "        cbi    0x05, 5\n"        // (2)   set WAIT signal to LOW
       "        ori    r16,  2\n"        // (1)   set flag for unfinished bus activity
       "        rjmp   LWD0\n"           // (2)   done (24 cycles)

       // reading "sector" (1) register (11 cycles so far)
       "LWD6:   out    0x0B, r14\n"      // (1)  set output data on PORTD
       "        out    0x0A, r18\n"      // (1)  switch PORTD to output
       "        cbi    0x05, 5\n"        // (2)  set WAIT signal to LOW
       "        ori    r16,  2\n"        // (1)  set flag for unfinished bus activity
       "        rjmp   LWD0\n"           // (2)  done (18 cycles)

       // reading "data in" (2 or 3) register (9 cycles so far)
       "LWD7:   out    0x0B, r18\n"      // (1)  output 0xFF
       "        out    0x0A, r18\n"      // (1)  switch PORTD to output
       "        cbi    0x05, 5\n"        // (2)  set WAIT signal to LOW
       "        ori    r16,  2\n"        // (1)  set flag for unfinished bus activity
       "        rjmp   LWD0\n"           // (2)  done (16 cycles)

       // found OUT signal (5 cycles so far)
       "LWD8:   in     r19, 0x06\n"      // (1)   read address
       "        in     r17, 0x09\n"      // (1)   read data from bus
       "        cbi    0x05, 5\n"        // (2)   set WAIT signal to LOW
       "        ori    r16,  2\n"        // (1)   set flag for unfinished bus activity
       "        sbrs   r19, 1\n"         // (1/2) address bit 1 (PC1) set?
       "        rjmp   LWD0\n"           // (2)   => NOT writing data register => ignore, done (8 cycles)

       // writing "data out" (2) register (12 cycles so far)
       "        or     r23, r23\n"       // (1)   if we have already received as...
       "        breq   LWD8a\n"          // (1/2) ...many bytes as we need then ignore
       "        st     X+, r17\n"        // (2)   store received data byte
       "        dec    r23\n"            // (1)   decrement receive byte counter
       "LWD8a:  rjmp   LWD0\n"           // (2)   done (19 cycles)

       // have unfinished bus activity (3 cycles so far)
       "LWD9:   in     r17, 0x06\n"      // (1)   read PINC
       "        andi   r17, 0x30\n"      // (1)   get bits 4+5
       "        brne   LWD9a\n"          // (1/2) if either one is set then we can't finish yet
       "        out    0x0A, r15\n"      // (1)   set PORTD to input
       "        sbi    0x05, 5\n"        // (2)   set WAIT signal back to HIGH
       "        sbi    0x1b, 1\n"        // (2)   reset INP/OUT pin change flag (PCIFR & bit(PCI1F1))
       "        andi   r16,  253\n"      // (1)   clear flag for unfinished bus activity
       "LWD9a:  rjmp   LWD0\n"           // (2)   done (14 cycles)

      // ===========================  end of write loop
    
       // --- finish potential unfinished bus activity
       "LWD10:  sbrs    r16, 1\n"        // (1/2) do we have unfinished bus activity?
       "        rjmp    LWD12\n"         // (2)   no => done
       "LWD11:  in      r17, 0x06\n"     // (1)   read PINC
       "        andi    r17, 0x30\n"     // (1)   mask bits 4+5
       "        brne    LWD11\n"         // (1/2) if either one is set then wait
       "        out     0x0A, r15\n"     // (1)   set PORTD to input
       "        sbi     0x05, 5\n"       // (2)   set WAIT signal back to HIGH
       "        sbi     0x1b, 1\n"       // (2)   reset INP/OUT pin change flag (PCIFR & bit(PCI1F1))
       "LWD12:  andi    r16, 8\n"            // set "foundIndex" variable high if we have seen 
       "        sts     foundIndex,  r16\n"  // a high->low edge on INDEX signal
       "        sts     indexTime+0, r10\n"  // set indexTime variable to TCNT1 value
       "        sts     indexTime+1, r11\n"  // when index hole was seen

       :                                  // no outputs
       : "z"(ptr), "d"(n), "r"(li)        // inputs 
       : "r10", "r11", "r13", "r14", "r15", "r16", "r17", "r18", "r19", "r20", "r21", "r23", "r24", "r25", "r26", "r27"); // clobbers (x=r26/r27)
  
  // disable write gate
  drivectrl_set(PIN_SRO_WRITEGATE, HIGH);

  // COM1A1:0 = 00 => disconnect OC1A (will go high)
  TCCR1A = 0;

  // restore previous OCR1A value
  OCR1A = OCR1Aprev;
}


static void write_sector_data(const byte *ptr, byte n, byte sector_start_offset)
{
  // Unfortunately the write_sector_data_dd() implementation for writing is too slow
  // to handle high-density writes (2us minimum time between pulses).
  // The write_sector_data_hd() implementation can manage high-density writes but
  // had to be very optimized for that case so it can not be used for double-density
  // data. Hence we are left with two separate implementations for writing data.
  if( driveType[selDrive]==DT_5INCH_DD )
    write_sector_data_dd(ptr, n, sector_start_offset);
  else
    write_sector_data_hd(ptr, n, sector_start_offset);
}


byte read_sector_data(byte *buffer, byte n)
{
  // calculate threshold between data pulse (2 or 4 microseconds) and clock pulse (4 or 8 microseconds)
  byte threshold;
  if( driveType[selDrive]==DT_5INCH_DD )
    threshold = 12; // 6 microseconds
  else
    threshold = 6;  // 3 microseconds

  // time out after sectorLength-2*readClear microseconds
  // (timer 2 prescaler is /1024, 64us resolution, max time = 16.384ms)
  TCNT2 = 0;
  OCR2A = (sectorLength[selDrive]-2*readClear[selDrive])/64;
  TIFR2 = bit(OCF2A);

  // readFinished will be set "true" if we read the full sector
  readFinished = 0;

  asm volatile 
     ( // initialize
      "        movw    x,   z\n"         // "x" is buffer read pointer, "z" is buffer write pointer
      "        mov     r12, r26\n"       // save beginning value of read pointer (low-byte only, x=r26/r27)
      "        lds     r19, regStatus\n" // get regStatus value for quick access (does not change except for NRDA)
      "        ori     r19, 128\n"       // make sure NRDA bit is set (no data ready)
      "        mov     r13, r19\n"       // into r13
      "        lds     r19, regSector\n" // get regSector value for quick access (does not change)
      "        ori     r19, 0x01\n"      // sector is not true while in here
      "        mov     r14, r19\n"       // into r14
      "        ldi     r19, 0xFF\n"      // used when changing bus to OUTPUT
      "        mov     r15, r19\n"       // into r15
      "        ldi     r16, 4\n"         // bit 0: SYNC bit seen, bit 1: unfinished bus activity, bit 2: expecting clock bit
      "        ldi     r23, 8\n"         // read 8 bits
      "        sbi     0x16, 5\n "       // clear input capture flag

      // ======================  start of read loop
      
      "LR0:    sbic    0x17, 1\n"       // (1/2) skip next instruction if timer 2 output compare NOT set
      "        rjmp    rabrt\n"         // (2)   sector timeout => abort reading
      "LR1:    sbis    0x16, 5\n"       // (1/2) skip next instruction if timer 1 input capture seen
      "        rjmp    LR2\n"           // (2)   no capture => handle bus communication

      // ======================  input capture => handle disk input (max 23 cycles)
      
      // read clock bit : 11 cycles
      // read data bit  : 18 cycles (23 cycles if new byte)
      "        lds     r19, 0x86\n"     // (2)   get time of input capture (ICR1L, lower 8 bits only)
      "        sbi     0x16, 5\n "      // (2)   clear input capture flag
      "        sbrc    r16, 2\n"        // (1/2) skip if we are expecting a data bit
      "        rjmp    LR3\n"           // (2)   expecting a clock bit

      // expecting a data bit (6 cycles so far)
      // timer runs at /8 prescaler => 16MHz/8 = 2MHz => 0.5us per tick
      "        add     r25, r25\n"      // (1)   shift received byte left by one bit
      "        mov     r22, r19\n"      // (1)   calculate time since previous capture...
      "        sub     r22, r17\n"      // (1)   ...into r22
      "        mov     r17, r19\n"      // (1)   set r17 to time of current capture
      "        cp      r22, %3\n"       // (1)   time since previous capture < clock/data threshold?
      "        brcs    LR4\n"           // (1/2) yes => data "1" bit

      // we were expecting a data bit but just found another clock (>3 microseconds) => data "0" bit
      // (still expecting a data bit next, 12 cycles so far)
      "        sbrc    r16, 0\n"        // (1/2) skip next instruction if we are not yet sync'ed
      "        rjmp    rnxtbi\n"        // (2)   received "0" bit
      "        rjmp    LR0\n"           // (2)   not sync'ed => do not record bit, check timeout (17+2 cycles)

      // found data "1" bit (13 cycles so far)
      "LR4:    ori     r16, 5\n"        // (1)   expecting clock bit next AND have now seen SYNC bit
      "        ori     r25, 1\n"        // (1)   received "1" bit
      
      // --- next bit (15 cycles so far)
      "rnxtbi: subi    r23, 1\n"        // (1)   decrement bit counter
      "        brne    LR0\n"           // (1/2) not all bits received => back to start, checking timeout (18+2 cycles)

      // --- next byte (17 cycles so far)
      "rnxtby: st      Z+, r25\n"       // (2)   store received byte
      "        ldi     r23, 8\n"        // (1)   reset bit counter
      "        subi    %2, 1\n"         // (1)   decrement byte counter
      "        brne    LR1\n"           // (1/2) repeat until all bytes received (23 cycles)
      "        rjmp    rdone\n"         // (2)   done

      // --- expecting a clock bit (7 cycles so far)
      "LR3:    andi    r16, 251\n"      // (1)   next bit should be a data bit
      "        mov     r17, r19\n"      // (1)   set r17 to time of current capture
      "        rjmp    LR0\n"           // (2)   back to start (11+2 cycles)

      // ======================  no input capture => handle bus activity (max 20 cycles)

      "LR2:    sbrc   r16, 1\n"         // (1/2) is there unfinished bus activity?
      "        rjmp   LR9\n"            // (2)   yes, try to finish it
      "        sbic   0x06, 4\n"        // (1/2) skip next instruction if PC4 clear
      "        rjmp   LR5\n"            // (2)   PC4 set => found INP
      "        sbis   0x06, 5\n"        // (1/2) skip next instruction if PC5 set
      "        rjmp   LR10\n"           // (2)   PC4+PC5 clear => no bus activity

      // PR5 set => found OUT signal (6 cycles so far)
      "        sbis   0x06, 1\n"        // (1/2) address bit 1 (PC1) set?
      "        rjmp   rdon2\n"          // (2)   no, writing register 0 or 1 => abort reading, do NOT release WAIT
      "        cbi    0x05, 5\n"        // (2)   writing other register, ignore => set WAIT signal to LOW
      "        ori    r16, 2\n"         // (1)   set flag for unfinished bus activity
      "        rjmp   LR0\n"            // (2)   back to start (13+2 cycles)

      // PC4 set => found INP signal (5 cycles so far)
      "LR5:    sbic   0x06, 1\n"        // (1/2) address bit 1 (PC1) clear?
      "        rjmp   LR6\n"            // (2)   => no, reading "data in" register (2 or 3)
      "        sbic   0x06, 0\n"        // (1/2) address bit 0 (PC0) clear?
      "        rjmp   LR7\n"            // (2)   => no, reading "sector" register (1)

      // reading "status" (0) register (9 cycles so far)
      "        mov    r19, r13\n"       // (1)   get status register (NRDA is set, see initialization)
      "        cp     xl, zl\n"         // (1)   compare X and Z, if they are equal then we don't have data
      "        breq   LR8 \n"           // (1/2) no data to send
      "        andi   r19, 127\n"       // (1)   have data => clear NRDA bit
      "LR8:    out    0x0B, r19\n"      // (1)   set output data on PORTD
      "        out    0x0A, r15\n"      // (1)   switch PORTD to output
      "        cbi    0x05, 5\n"        // (2)   set WAIT signal to LOW
      "        ori    r16,  2\n"        // (1)   set flag for unfinished bus activity
      "        rjmp   LR1\n"            // (2)   back to start (20 cycles)

      // reading "sector" (1) register (10 cycles so far)
      "LR7:    out    0x0B, r14\n"      // (1)   set output data on PORTD
      "        out    0x0A, r15\n"      // (1)   switch PORTD to output
      "        cbi    0x05, 5\n"        // (2)   set WAIT signal to LOW
      "        ori    r16,  2\n"        // (1)   set flag for unfinished bus activity
      "        rjmp   LR1\n"            // (2)   back to start (17 cycles)

      // reading "data in" (2) register (8 cycles so far)
      "LR6:    cp     xl, zl\n"         // (1)   compare X and Z, if they are equal then we don't have anything to send
      "        breq   LR6a\n"           // (1/2) nothing to send => DONT release WAIT signal
      "        ld     r19, X+\n"        // (2)   load data byte
      "        out    0x0B, r19\n"      // (1)   set output data on PORTD
      "        out    0x0A, r15\n"      // (1)   switch PORTD to output
      "        cbi    0x05, 5\n"        // (2)   set WAIT signal to LOW
      "        ori    r16,  2\n"        // (1)   set flag for unfinished bus activity
      "LR6a:   rjmp   LR1\n"            // (2)   back to start (19 cycles)

      // found unfinished bus activity (3 cycles so far)
      "LR9:    in     r19, 0x06\n"      // (1)   read PINC
      "        andi   r19, 0x30\n"      // (1)   get bits 4+5 (INP and OUT signals)
      "        breq   LR9a\n"           // (1/2) if both are 0 then finish now
      "        rjmp   LR0\n"            // (2)   can't finish yet (8+2 cycles)
      "LR9a:   ldi    r19,  0\n"        // (1)   switch PORTD...
      "        out    0x0A, r19\n"      // (1)   back to input
      "        sbi    0x05, 5\n"        // (2)   set WAIT signal back to HIGH
      "        sbi    0x1b, 1\n"        // (2)   reset INP/OUT pin change flag (PCIFR & bit(PCI1F1))
      "        andi   r16,  253\n"      // (1)   clear flag for unfinished bus activity
      "        rjmp   LR1\n"            // (2)   back to start (16 cycles)

      // no data from disk and no bus activity => check INDEX signal pin change (7 cycles so far)
      "LR10:   sbis    0x1b, 0\n"       // (1/2) skip next instruction if PCIF0 is set
      "        rjmp    LR0\n"           // (2)   no INDEX signal pin change => back to start (10+2 cycles)
      "        sbi     0x1b, 0\n"       // (2)   clear PCIF0
      "        sbic    0x03, 3\n"       // (1/2) skip next instruction if PB3 is clear
      "        rjmp    LR0\n"           // (2)   INDEX signal went HIGH => back to start (14+2 cycles)
      "        lds     r10, 0x84\n"     // (2)   get current TCNT1L
      "        lds     r11, 0x85\n"     // (2)   get current TCNT1H
      "        ori     r16, 8\n"        // (1)   set flag for "index signal seen"
      "        rjmp    LR1\n"           // (2)   back to start (20 cycles)

      // ======================  done reading - check for unfinished bus activity
      "rdone:  lds     r19, 0x84\n"
      "        sts     readDoneTime+0, r19\n"
      "        lds     r19, 0x85\n"
      "        sts     readDoneTime+1, r19\n"
      "        ldi     r19, 1\n"
      "        sts     readFinished, r19\n"
      "rabrt:  sbrs    r16, 1\n"        // (1/2) do we have unfinished bus activity?
      "        rjmp    rdon2\n"         // (2)   no => done
      "wtbus:  in      r19, 0x06\n"     // (1)   read PINC
      "        andi    r19, 0x30\n"     // (1)   mask bits 4+5
      "        brne    wtbus\n"         // (1/2) if either one is set then wait
      "        ldi     r19,  0\n"       // (1)   switch PORTD...
      "        out     0x0A, r17\n"     // (1)   back to input
      "        sbi     0x05, 5\n"       // (2)   set WAIT signal back to HIGH
      "        sbi     0x1b, 1\n"       // (2)   reset INP/OUT pin change flag (PCIFR & bit(PCI1F1))
      "rdon2:  mov     %0, r26\n"       // (1)   return difference between current value of...
      "        sub     %0, r12\n"       // (1)   ...read pointer and beginning value of read pointer
                                        //       (only looking at low byte because size is < 256)
      "        andi    r16, 8\n"            // set "foundIndex" variable high if we have seen 
      "        sts     foundIndex,  r16\n"  // a high->low edge on INDEX signal
      "        sts     indexTime+0, r10\n"  // set indexTime variable to TCNT1 value
      "        sts     indexTime+1, r11\n"  // when index hole was seen

      : "=r"(n)              // outputs
      : "z"(buffer), "d"(n), "l"(threshold)  // inputs  (z=r30/r31)
      : "r10", "r11", "r12", "r13", "r14", "r15", "r16", "r17", "r19", "r22", "r23", "r24", "r25", "r26", "r27"); // clobbers (x=r26/r27)

  // n now contains the number of bytes that were already sent out while reading the data,
  // i.e. the location in the buffer of the first byte that still needs to be sent.
  return n;
}


// -------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------- monitor functions --------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------------


#define XM_WRITE     1
#define XM_VERIFY    2

#define S_OK         0  // no error
#define S_NOINDEX    1  // No index hole signal
#define S_NOTRACK0   2  // No track0 signal
#define S_VERIFY     3  // verify error


#include "XModem.h"
static int xmodem_datalen = 0, xmodem_sectors = 0;
static byte xmodem_cmd = 0, xmodem_status = S_OK, xmodem_errsec = 0, current_track[MAX_DRIVES];
static unsigned long motor_timeout[MAX_DRIVES];
static bool motor_auto[MAX_DRIVES];



inline void print_hex(byte b)
{
  if( b<16 ) Serial.write('0');
  Serial.print(b, HEX);
}


void print_error(byte n)
{
  Serial.print(F("Error: "));
  switch( n )
    {
    case S_OK        : Serial.print(F("No error")); break;
    case S_NOINDEX   : Serial.print(F("No index hole detected")); break;
    case S_NOTRACK0  : Serial.print(F("No track 0 signal detected")); break;
    case S_VERIFY    : Serial.print(F("Verify failed")); break;
    default          : Serial.print(F("Unknonwn error")); break;
    }
}


static void dump_buffer(byte *buf, int n)
{
  int offset = 0;
  while( offset<n )
    {
      print_hex(offset/256); 
      print_hex(offset&255); 
      Serial.write(':');

      for(int i=0; i<16; i++)
        {
          if( (i&7)==0  ) Serial.write(' ');
          print_hex(buf[offset+i]);
          Serial.write(' ');
        }

      Serial.write(' ');
      for(int i=0; i<16; i++)
        {
          if( (i&7)==0  ) Serial.write(' ');
          Serial.write(isprint(buf[offset+i]) ? buf[offset+i] : '.');
        }

      Serial.println();
      offset += 16;
    }
}


static bool wait_index_hole(bool resetTimer = true)
{
  // reset timer and overrun flags
  if( resetTimer ) TCNT1 = 0;
  TIFR1 = bit(TOV1);
  byte ctr = 4;

  // wait for END of index hole (in case we're on the hole right now)
  while( !(PINB & 0x08) )
    {
      if( TIFR1 & bit(TOV1) )
        {
          // timer overflow happens every 262.144ms (65536 cycles at 16MHz with /64 prescaler)
          // meaning we haven't found an index hole in that amount of time
          if( ctr-- == 0 )
            {
              // we have tried for 4*262 ms = ~1 second to find an index hole
              // one rotation is 200ms so we have tried for 5 full rotations => give up
              return false;
            }
          
          // clear overflow flag
          TIFR1 = bit(TOV1);
        }
    }

  // wait for START of index hole (same as above)
  ctr = 4;
  while( (PINB & 0x08) )
    {
      if( TIFR1 & bit(TOV1) )
        {
          if( ctr-- == 0 ) return false;
          TIFR1 = bit(TOV1);
        }
    }

  return true;
}


static bool wait_track_start()
{
  // find index hole
  if( !wait_index_hole() ) return false;

  // if hard-sectored then we need to find the "track start" index hole
  if( hardSectored[selDrive] )
    {
      // "track start" index hole sits between two sector index holes,
      // so we look for an index hole less than 3/4 of a sector length
      // after the previous one.
      // timer runs at /64 prescaler, i.e. one tick is 4 microseconds
      // sectorLength is in microseconds and never greater than 16000
      unsigned int t = (sectorLength[selDrive]*3) / (4*4);

      do
        {
          // find the "track start" index hole
          do 
            { if( !wait_index_hole() ) return false;}
          while( TCNT1 > t );
          
          // find the "sector start" index hole after "track start"
          if( !wait_index_hole() ) return false;

          // if the interval between "track start" and the next "sector start"
          // was a full sector interval (instead of the expected half one) 
          // then what we thought was "track start" actually was the "sector start" 
          // and we missed the real sector start.
          // => repeat to find the proper start
        }
      while( TCNT1 > t );
    }

  return true;
}


// assumes timer 1 prescaler is /64
static bool wait_sector_start(byte sector, bool read, byte *dummyBuffer)
{
  // wait start of track 
  if( !wait_track_start() ) return false;
  TCNT1 = 0;
  OCR1A = sectorOffset[selDrive] / 4;

  if( hardSectored[selDrive] )
    {
      // hard-sectored => count down index holes
      while( sector-->0 ) { if( !wait_index_hole() ) return false; }
      // reset counter again (after waiting for index holes)
      TCNT1 = 0; 
    }
  else if( (driveFlags[selDrive] & DF_RELSECTOR)!=0 && sector>0 )
    {
      // soft-sectored, relative sectors => count sectors
      TCCR1B = bit(CS11);

      // set up for first sector start
      TCNT1 = 0; 
      OCR1A = (sectorOffset[selDrive]+readClear[selDrive]) * 2;
      while( sector-->0 )
        {
          // wait for start of sector
          TIFR1 = bit(OCF1A);
          while( !(TIFR1 & bit(OCF1A)) );
          
          // read sector data
          DBGPIN(PORTC, 3, 0);
          read_sector_data(dummyBuffer, 137);
          DBGPIN(PORTC, 3, 1);
          
          // if we have more sectors then set up for next sector start
          if( sector>0 )
            {
              // if the sector was successfully read then use relative timing, 
              // otherwise fall back to absolute
              if( readFinished )
                OCR1A = readDoneTime + (sectorTail[selDrive] + readClear[selDrive])*2;
              else
                OCR1A += sectorLength[selDrive];
            }
        }

      // set OCR1A to start of next sector (/16 prescaler)
      // (relative if successful read, otherwise absolute)
      if( readFinished )
        OCR1A = sectorTail[selDrive]/4 - (TCNT1-readDoneTime)/8;
      else
        OCR1A = (sectorLength[selDrive] - (TCNT1-OCR1A)/2 - readClear[selDrive])/4;

      // back to /16 prescaler
      TCNT1 = 0; 
      TCCR1B = bit(CS10) | bit(CS11);
    }
  else
    {
      // soft-sectored, absolute sectors => go by time
      // For HD (32 sectors), sector length is  ~6250us for 300 RPM and  ~5200us for 360 RPM.
      // For DD (16 sectors), sector length is ~12500us for 300 RPM and ~10400us for 360 RPM.
      // => a full track is max 16*12500 = 200000us
      // clock is 16MHz, using /64 prescaler for 4 microsecond resolution
      // Timer1 is 16 bit, maximum value 65535 => with 4 microsecond resolution,
      // at 200000 microseconds timer will be at 50000 (within range)
      OCR1A += ((uint32_t) sectorLength[selDrive] * (uint32_t) sector) / 4;
    }

  // if reading then wait for end of read-clear period
  if( read ) OCR1A += readClear[selDrive] / 4;

  // if we need to wait then do so now
  if( OCR1A>0 )
    {
      TIFR1 = bit(OCF1A);
      while( !(TIFR1 & bit(OCF1A)) );
    }

  return true;
}


// assumes timer 1 prescaler is /8
static void wait_next_sector_start(byte sector, bool read, bool doWait)
{
  if( sector==0 )
    {
      // sector 0 => wait for start of track 
      if( !wait_track_start() ) return;
      TCNT1 = 0;
      OCR1A = sectorOffset[selDrive]*2;

      // if reading then wait through read clear period
      if( read ) OCR1A += readClear[selDrive]*2;
    }
  else if( hardSectored[selDrive] )
    {
      // hard-sectored => wait for next index hole
      if( !wait_index_hole() ) return;
      TCNT1 = 0;
      OCR1A = sectorOffset[selDrive]*2;

      // if reading then wait through read clear period
      if( read ) OCR1A += readClear[selDrive]*2;
    }
  else
    {
      // soft-sectored => assuming that the first sector was found by
      // function wait_sector_start() and we are NOT switching between read/write
      // we can just wait one full sector-length

      if( read && (driveFlags[selDrive] & DF_RELSECTOR)!=0 && readFinished )
        OCR1A = readDoneTime + (sectorTail[selDrive] + readClear[selDrive])*2;
      else
        OCR1A += sectorLength[selDrive]*2;
    }

  // wait if required
  if( doWait )
    {
      TIFR1 = bit(OCF1A);
      while( !(TIFR1 & bit(OCF1A)) );
    }
}


static void step_track(bool stepOut)
{
  drivectrl_step_pulse(stepOut);

#ifndef TRUE_DD
  if( driveType[selDrive]==DT_5INCH_DD )
    {
      // double density disk => do two steps
      delay(4);
      drivectrl_step_pulse(stepOut);
    }
#endif

  if( current_track[selDrive] != 0xFF )
    {
      if( stepOut )
        current_track[selDrive]--;
      else
        current_track[selDrive]++;
    }

  // wait 10ms (minimum time between steps is 3ms for
  // same direction, 10ms for direction change)
  delay(10);
}


static void step_tracks(int tracks)
{
  // if tracks<0 then step outward (towards track 0) otherwise step inward
  bool stepOut = tracks<0;
  tracks = abs(tracks);
  while( tracks-->0 ) step_track(stepOut);
  delay(90); 
}



static bool step_to_track0()
{
  byte n = 82;

  // step outward until TRACK0 line goes low
  while( --n > 0 && SRI_BIT_SET(PIN_SRI_TRACK0) ) 
    { drivectrl_step_pulse(true); delay(driveType[selDrive]==DT_SA800 ? 10 : 3); }

  if( n==0 )
    {
      // we have stpped for more than 80 tracks and are still not 
      // seeing the TRACK0 signal
      return false;
    }

  current_track[selDrive] = 0;
  return true;
}


static byte step_to_track(byte track)
{
  byte res = S_OK;

  if( current_track[selDrive]==0xFF )
    if( !step_to_track0() )
      return S_NOTRACK0;

  step_tracks(track-current_track[selDrive]);
  current_track[selDrive] = track;

  return S_OK;
}


byte write_sectors(byte sector, const byte *buffer, byte num_sectors)
{
  byte b[137];
  byte res = S_OK;

  noInterrupts();
  TCCR1B = bit(CS10) | bit(CS11);  // start timer 1 with /64 prescaler

  // wait for first sector to write
  if( wait_sector_start(sector, false, b) )
    {
      // switch to /8 prescaler (for writing data)
      TCCR1B = bit(CS11);
      TCNT1  = 0;
      OCR1A  = 0;

      while( num_sectors > 0 )
        {
          write_sector_data(buffer, 137, 0);

          // wait for next sector to write
          buffer += 137;
          sector = (sector + 1) % numSectors[selDrive];
          num_sectors--;
          if( num_sectors>0 ) wait_next_sector_start(sector, false, true);
        }
    }
  else
    res = S_NOINDEX;

  TCCR1B = 0; // stop timer 1
  interrupts();

  return res;
}


byte verify_sectors(byte sector, const byte *buffer, byte num_sectors)
{
  byte b[137], res = S_OK;

  noInterrupts();
  TCCR1B = bit(CS10) | bit(CS11);  // start timer 1 with /64 prescaler
  DBGPIN(PORTC, 3, 1);

  // wait for first sector to read
  if( wait_sector_start(sector, true, b) )
    {
      // switch to /8 prescaler (for reading data)
      TCCR1B = bit(CS11);
      TCNT1  = 0;
      OCR1A  = 0;
      
      while( num_sectors > 0 )
        {
          // read data from disk
          DBGPIN(PORTC, 3, 0);
          read_sector_data(b, 137);
          DBGPIN(PORTC, 3, 1);

          // wait_next_sector_start() sets up OCR1A to wait for the start of the next 
          // sector's data (for hard-sectored media it also waits for the index hole)
          if( num_sectors>1 )
            wait_next_sector_start((sector + 1) % numSectors[selDrive], true, false);
         
          // use some of the time before next sector start to compare data
          if( memcmp(buffer, b, 137)!=0 ) 
            { 
              // found a difference => save data and exit
              memmove(dataBuffer, buffer, 137); 
              memcpy(dataBuffer+137, b, 137); 
              xmodem_errsec = sector; 
              res = S_VERIFY; 
              num_sectors = 1;
            }

          // wait for next sector to read
          buffer += 137;
          sector = (sector + 1) % numSectors[selDrive];
          num_sectors--;

          // if we have another sector to verify then wait for its start
          if( numSectors>0 )
            {
              TIFR1 = bit(OCF1A);
              while( !(TIFR1 & bit(OCF1A)) );
            }
        }
    }
  else
    res = S_NOINDEX;

  TCCR1B = 0; // stop timer 1
  interrupts();

  return res;
}


byte read_sector(byte sector, byte *buffer)
{
  byte res = S_OK;

  noInterrupts();
  TCCR1B = bit(CS10) | bit(CS11);  // start timer 1 with /64 prescaler
  DBGPIN(PORTC, 3, 1);

  // wait for sector to read
  if( wait_sector_start(sector, true, buffer) )
    {
      TCCR1B = bit(CS11);   // switch to /8 prescaler (for reading data)
      DBGPIN(PORTC, 3, 0);
      read_sector_data(buffer, 137);
      DBGPIN(PORTC, 3, 1);
    }
  else
    res = S_NOINDEX;

  TCCR1B = 0; // turn off timer 1
  interrupts();

  return res;
}


int recvChar(int msDelay) 
{ 
  unsigned long start = millis();
  while( (int) (millis()-start) < msDelay) 
    { 
      if( Serial.available() ) return (uint8_t) Serial.read(); 
    }

  return -1; 
}


void sendData(const char *data, int size)
{
  Serial.write((const uint8_t *) data, size);
}


bool xmodemDataHandler(unsigned long no, char* d, int size)
{
  // ignore any extra data received after we're done
  if( xmodem_status==S_OK && xmodem_sectors<numSectors[selDrive]*numTracks[selDrive] )
    {
      memcpy(dataBuffer+xmodem_datalen, d, size);
      xmodem_datalen += size;
      
      if( xmodem_datalen >= 8*137 )
        {
          if( xmodem_status==S_OK && (xmodem_cmd & XM_WRITE) )
            xmodem_status = write_sectors(xmodem_sectors % numSectors[selDrive], dataBuffer, 8);
          
          if( xmodem_status==S_OK && (xmodem_cmd & XM_VERIFY) )
            xmodem_status = verify_sectors(xmodem_sectors % numSectors[selDrive], dataBuffer, 8);
          else
            delay(1); // drive needs at least 1ms between end of write and head step
          
          if( xmodem_status == S_OK )
            {          
              xmodem_sectors += 8;
              
              // step to next track
              if( xmodem_sectors<numSectors[selDrive]*numTracks[selDrive] )
                xmodem_status = step_to_track(xmodem_sectors / numSectors[selDrive]);
              
              xmodem_datalen -= 8*137;
              memmove(dataBuffer, dataBuffer+(8*137), xmodem_datalen);
            }
        }
    }

  return xmodem_status==S_OK;
}


static String read_user_cmd()
{
  String s;
  do
    {
      int i = Serial.read();

      if( i==13 || i==10 ) 
        { Serial.println(); break; }
      else if( i==27 )
        { s = ""; Serial.println(); break; }
      else if( i==8 )
        { 
          if( s.length()>0 )
            { Serial.write(8); Serial.write(' '); Serial.write(8); s = s.substring(0, s.length()-1); }
        }
      else if( isprint(i) )
        { s = s + String((char )i); Serial.write(i); }

      if( motor_timeout[0]>0 && millis() > motor_timeout[0] ) { drivectrl_set(PIN_SRO_MOTOR0, HIGH); motor_timeout[0] = 0; }
      if( motor_timeout[1]>0 && millis() > motor_timeout[1] ) { drivectrl_set(PIN_SRO_MOTOR1, HIGH); motor_timeout[1] = 0; }
    }
  while(true);

  return s;
}


// returns average time per rotation in microseconds
unsigned long measure_rotation()
{
  TCCR1B = bit(CS10) | bit(CS11);  // start timer 1 with /64 prescaler

  // return 0 if motor not running
  if( !wait_index_hole() ) return 0;

  // build average cycle count (4us/cycle) over 4 revolutions
  unsigned long l = 0;
  for(byte i=0; i<4; i++)
    {
      if( !wait_index_hole() ) return 0;
      l += TCNT1;
    }

  TCCR1B = 0; // turn off timer 1
  return l;
}


void print_rotation(unsigned long t)
{
  char c;
  Serial.print(t/1000); Serial.print('.'); 
  c = '0' + ((t/100) % 10); Serial.print(c);
  c = '0' + ((t/ 10) % 10); Serial.print(c);
  c = '0' + ((t/  1) % 10); Serial.print(c);
}


void measure_sector(bool waitSpinup)
{
  byte n = waitSpinup ? 0 : 2;
  unsigned long t, tt;

  Serial.print(F("Measuring rotation period for drive "));
  Serial.print(selDrive); Serial.print(F("... "));
  
  t = 0;
  while( n<6 )
    {
      tt = measure_rotation();
      if( tt==0 )
        { Serial.print(F("\r\nplease insert disk... ")); Serial.flush(); }
      else
        { 
          if( n>1 ) t+=tt; 
          n++; 
        }
    }

  set_sector_length(selDrive, t/4);

  if( hardSectored[selDrive] )
    {
      Serial.print(F("hard-sectored, "));
      Serial.print(rpm[selDrive]==RPM_300 ? 300 : 360);
      Serial.println(F(" RPM"));
    }
  else
    {
      print_rotation(t/4);
      Serial.println(F("ms"));
    }
}


void clear_diskchange()
{
  if( !SRI_BIT_SET(PIN_SRI_DSKCHG) )
    {
      if( driveType[selDrive]==DT_SA800 )
        { 
          // "disk change" flag cleared by de-selecting drive
          drivectrl_set(PIN_SRO_SELECT, HIGH);
          drivectrl_set(PIN_SRO_SELECT, LOW);
        }
      else
        {
          // "disk change" flag cleared by moving head
          if( SRI_BIT_SET(PIN_SRI_TRACK0) )
            { step_tracks(-1); step_tracks(1); }
          else
            { step_tracks(1); step_tracks(-1); }
        }
    }
}


void motor(bool on)
{
  if( on )
    {
      drivectrl_set(PIN_SRO_MOTOR, LOW);

      // the motors of 8 inch drives are always on and we map
      // the MOTOR signal to HEADLOAD (which is not present on 5.25" drives)
      // HEADLOAD only requres a 35 millisecond delay before data can be read/written
      delay(driveType[selDrive]==DT_SA800 ? 35 : 500);

      if( !SRI_BIT_SET(PIN_SRI_DSKCHG) ) { sectorLength[selDrive] = 0; clear_diskchange(); }
      if( sectorLength[selDrive]==0 ) { measure_sector(true); }
    }
  else
    {
      drivectrl_set(PIN_SRO_MOTOR, HIGH);
    }
}


bool check_motor()
{
  if( (drivectrl & bit(PIN_SRO_MOTOR)) )
    { motor(true); return true; }
  else if( !SRI_BIT_SET(PIN_SRI_DSKCHG) )
    { clear_diskchange(); measure_sector(false); return true; }

  return false;
}


void measure_rotation_stats()
{
  while( Serial.available() ) Serial.read();
  Serial.println(F("Measuring spindle rotation, press any key to stop..."));

  bool motorOn = (drivectrl & bit(PIN_SRO_MOTOR))==0;
  if( driveType[selDrive]!=DT_SA800 ) drivectrl_set(PIN_SRO_MOTOR,  LOW);

  // start timer 1 with /64 prescaler
  TCCR1B = bit(CS10) | bit(CS11);
  while( !wait_index_hole() && !Serial.available() )
    { Serial.print(F("\r\nplease insert disk... ")); Serial.flush(); }
  unsigned int tp = TCNT1;
      
  unsigned int shortIndexThresh = 0xFFFF;
  bool hardSectored = TCNT1 < (20000/4);
  if( hardSectored )
    {
      // short index threshold is three quarters of a sector length
      // (divide by 4 again because one timer tick is 4us)
      if( driveType[selDrive] == DT_5INCH_DD )
        shortIndexThresh = ((200000ul/16)*3)/(4*4);
      else
        shortIndexThresh = ((166666ul/32)*3)/(4*4);

      // wait for the FIRST short index period (track start)
      do { if( !wait_index_hole() ) return; } while( TCNT1<shortIndexThresh );
      do { if( !wait_index_hole() ) return; } while( TCNT1>shortIndexThresh );
      tp = TCNT1;
    }
  
  if( !Serial.available() )
    {
      bool full = false;
      byte i = 0, timesArraySize = 64;
      unsigned int *times = (unsigned int *) dataBuffer+128;
      unsigned int min = 0xFFFFFFFF, max = 0;
      unsigned long sum = 0, avgbase, avgdiff;
      
      while( !Serial.available() )
        {
          if( hardSectored )
            {
              // hard sectored => wait for the FIRST short index period (track start)
              unsigned int t;
              do { t = TCNT1; if( !wait_index_hole(false) ) return; } while( (TCNT1-t)<shortIndexThresh );
              do { t = TCNT1; if( !wait_index_hole(false) ) return; } while( (TCNT1-t)>shortIndexThresh );
            }
          else
            {
              // soft sectored => wait for index hole
              if( !wait_index_hole(false) ) return;
            }
          
          unsigned int tt = TCNT1; 
          unsigned int t = tt-tp;
          tp = tt;
          
          if( t>max ) max = t;
          if( t<min ) min = t;
          
          if( full )
            {
              // buffer is full
              sum -= times[i];
              times[i] = t;
              sum += times[i];
              i++;
              if( i==timesArraySize ) i = 0;
            }
          else
            {
              // buffer not yet full
              times[i] = t;
              i++;
              sum += t;
              if( i==timesArraySize ) { full = true; i = 0; }
            }

          if( (i & 3)==0 )
            {
              byte n = full ? timesArraySize : i;
              unsigned int avg = sum/n;
              Serial.print(F("Rotation period (ms, ")); Serial.print(n); Serial.print(F(" rotations): "));
              Serial.print(F("avg="));   print_rotation(avg*4ul);
              Serial.print(F(", min=")); print_rotation(min*4ul);
              Serial.print(F("=-")); print_rotation((avg-min)*100000ul/avg);
              Serial.print(F("%, max=")); print_rotation(max*4ul);
              Serial.print(F("=+")); print_rotation((max-avg)*100000ul/avg);
              Serial.print(F("%, avgdiff=")); print_rotation(avgdiff);
              Serial.print(F("=")); print_rotation(avgdiff*1000ul/avgbase);
              Serial.println('%');
            }
          else if( (i & 3)==3 )
            {
              avgdiff = 0;
              byte n = full ? timesArraySize : i;
              avgbase = sum/n;
              for(byte j=0; j<n; j++)
                avgdiff += avgbase>times[j] ? avgbase-times[j] : times[j]-avgbase;
              avgdiff = avgdiff / n;
            }
        }
    }

  Serial.read();
  if( !motorOn && driveType[selDrive]!=DT_SA800 ) motor(false);
}


void print_help()
{
  Serial.println();
  Serial.println(F("r [t,]s       Read track t (default current) sector s into buffer"));
  Serial.println(F("w [t,]s[,0/1] Write buffer content to track t (default current) sector s (1=and compare)"));
  Serial.println(F("b             Print buffer content"));
  Serial.println(F("B d           Fill buffer with decimal value d"));
  Serial.println(F("W [0/1]       Receive disk image via XModem and write disk (1=and compare)"));
  Serial.println(F("V             Receive disk image via XModem and compare to disk"));
  Serial.println(F("m [0/1]       Turn drive motor on or off or show current status"));
  Serial.println(F("M             Measure spindle rotation"));
  Serial.println(F("i [n]         Step n (default 1) tracks in"));
  Serial.println(F("o [n]         Step n (default 1) tracks out"));
  Serial.println(F("0             Step to track 0"));
  Serial.println(F("d 0/1         Select drive 0 or 1"));
  Serial.println(F("s 1/2         Select side 1 or 2"));
  Serial.println(F("I [0/1]       Print information about (current) drive"));
  Serial.println(F("T [n]         Print or set type of current drive"));
  Serial.println(F("F [d,]n       Set drive flags for drive d, set controller flags if d not given"));
  Serial.println(F("x             Exit to controller mode"));
  Serial.println();
}


void monitor()
{
  char cmd;
  int a1, a2, a3, track, sector, n;

  Serial.println(F("\r\n\nEntering disk monitor."));

  byte dip      = (~shift_in()) & 0x0F;
  bool minidisk = (dip & 0x08)!=0;

  // set drive types from DIP switches
  set_drive_type(0, (dip & 1)!=0 ? DT_SA800 : (minidisk ? DT_5INCH_DD : DT_5INCH_HD), false);
  set_drive_type(1, (dip & 2)!=0 ? DT_SA800 : (minidisk ? DT_5INCH_DD : DT_5INCH_HD), false);

  // no drive swapping in monitor
  pinSelect[0] = PIN_SRO_SELECT0; pinMotor[0] = PIN_SRO_MOTOR0;
  pinSelect[1] = PIN_SRO_SELECT1; pinMotor[1] = PIN_SRO_MOTOR1;
  if( (dip & 0x04)!=0 ) Serial.println(F("IGNORING DRIVE SWAP SETTING IN MONITOR"));
  
  selDrive = 0;
  current_track[0] = 0xFF;
  current_track[1] = 0xFF;
  drivectrl_set(PIN_SRO_SELECT, LOW);
  drivectrl_set(PIN_SRO_DENSITY, driveType[selDrive]!=DT_5INCH_DD);
  hardSectored[0] = false;
  hardSectored[1] = false;
  motor_timeout[0] = 0;
  motor_timeout[1] = 0;
  motor_auto[0] = true;
  motor_auto[1] = true;

  while( true )
    {
      Serial.print(F("\r\n\r\nCommand: "));
      String s = read_user_cmd();

      n = sscanf(s.c_str(), "%c%i,%i,%i", &cmd, &a1, &a2, &a3);
      if( n<=0 || isspace(cmd) ) { print_help(); continue; }

      if( cmd=='r' && n>=2 )
        {
          if( n>=3 )
            { track=a1; sector=a2; }
          else
            { track=-1; sector=a1; }

          if( track<numTracks[selDrive] && sector>=0 && sector<numSectors[selDrive] )
            {
              check_motor();
              byte status = track>=0 ? step_to_track(track) : S_OK;
              do
                {
                  if( status==S_OK )
                    {
                      delay(100);
                      memset(dataBuffer, 0, 137);
                      status = read_sector(sector, dataBuffer);
                    }
                  
                  if( status==S_OK ) 
                    dump_buffer(dataBuffer, 137);
                  else
                    print_error(status);
                  
                  if( n>3 && a3>0 ) { delay(1000); Serial.println(); }
                }
              while( status==S_OK && n>3 && a3>0 && !Serial.available() );
              Serial.read();
            }
        }
      else if( cmd=='w' && n>=2 )
        {
          if( n>=3 )
            { track=a1; sector=a2; }
          else
            { track=-1; sector=a1; }

          if( track<numTracks[selDrive] && sector>=0 && sector<numSectors[selDrive] )
            {
              check_motor();
              byte status = track>=0 ? step_to_track(track) : S_OK;
              if( status==S_OK )
                {
                  delay(100);
                  dataBuffer[0] |= 0x80; // set sync bit
                  status = write_sectors(sector, dataBuffer, 1);
                }

              if( status==S_OK && n>3 && a3!=0 )
                status = verify_sectors(sector, dataBuffer, 1);

              if( status==S_OK ) 
                Serial.println(F("Ok."));
              else
                print_error(status);
            }
        }
      else if( cmd=='W' || cmd=='V' )
        {
          xmodem_cmd = 0;
          xmodem_datalen = 0;
          xmodem_sectors = 0;
          xmodem_status = S_OK;
          if( cmd=='W' ) xmodem_cmd = (n>=2 && a1!=0) ? (XM_WRITE|XM_VERIFY) : XM_WRITE;
          if( cmd=='V' ) xmodem_cmd = XM_VERIFY;
          
          check_motor();
          if( step_to_track0() )
            {
              if( xmodem_cmd==XM_WRITE )
                Serial.println(F("Writing disk image."));
              else if( xmodem_cmd==XM_VERIFY )
                Serial.println(F("Verifying disk image."));
              else if( xmodem_cmd==(XM_WRITE|XM_VERIFY) )
                Serial.println(F("Writing and verifying disk image."));
              
              Serial.println(F(" Send image via XModem now..."));
              xmodem_sectors = 0;
              XModem modem(recvChar, sendData, xmodemDataHandler);
              if( modem.receive() )
                {
                  Serial.println(F("\r\nSuccess!"));
                }
              else
                {
                  unsigned long t = millis() + 500;
                  while( millis() < t ) { if( Serial.read()>=0 ) t = millis()+500; }
                  while( Serial.read()<0 );
                  
                  Serial.println('\r');
                  print_error(xmodem_status);
                  if( xmodem_status == S_VERIFY )
                    {
                      Serial.print(F("\r\n\nTrack="));     Serial.print(current_track[selDrive]);
                      Serial.print(F(", Sector="));        Serial.println(xmodem_errsec);
                      Serial.println(F("Expected data:")); dump_buffer(dataBuffer, 137);
                      Serial.println(F("Found data:"));    dump_buffer(dataBuffer+137, 137);
                      Serial.println();
                    }
                }
            }
          else
            { Serial.print(F("Error: ")); print_error(S_NOTRACK0); Serial.println(); }
        }
      else if( cmd=='b' )
        {
          Serial.println(F("Buffer contents:"));
          dump_buffer(dataBuffer, 137);
        }
      else if( cmd=='B' )
        {
          Serial.print(F("Filling buffer"));
          if( n==1 )
            {
              for(int i=0; i<137; i++) dataBuffer[i] = i;
            }
          else
            {
              Serial.print(F(" with 0x"));
              Serial.print(a1, HEX);
              for(int i=0; i<137; i++) dataBuffer[i] = a1;
            }
          Serial.println();
        }
      else if( cmd=='m' )
        {
          if( n==1 )
            {
              if( drivectrl & bit(PIN_SRO_MOTOR) )
                Serial.print(F("Motor is NOT running"));
              else
                Serial.print(F("Motor is running"));

              if( motor_auto[selDrive] ) Serial.print(F(" (auto-off mode)"));
              Serial.println();
            }
          else if( (drivectrl & bit(PIN_SRO_MOTOR))!=0 && a1!=0 )
            {
              Serial.println(F("Turning motor ON."));
              motor(true);
              motor_auto[selDrive] = false;
            }
          else if( (drivectrl & bit(PIN_SRO_MOTOR))==0 && a1==0 )
            {
              Serial.println(F("Turning motor OFF."));
              motor(false);
              motor_auto[selDrive] = false;
            }
          else if( a1==2 )
            {
              Serial.println(F("Setting motor to AUTO mode."));
              motor_auto[selDrive] = true;
            }
        }
      else if( cmd=='M' )
        {
          measure_rotation_stats();
        }
      else if( cmd=='i' )
        {
          int i = n>=2 ? abs(a1) : 1;
          if( current_track[selDrive]!=0xFF && current_track[selDrive]+i >= numTracks[selDrive] ) i = numTracks[selDrive]-current_track[selDrive]-1;
          step_tracks(i);
        }
      else if( cmd=='o' )
        {
          int i = n>=2 ? abs(a1) : 1;
          if( current_track[selDrive]!=0xFF && i > current_track[selDrive] ) i = current_track[selDrive];
          step_tracks(-i);
        }
      else if( cmd=='0' )
        {
          if( !step_to_track0() )
            { print_error(S_NOTRACK0); Serial.println(); }
        }
      else if( cmd=='h' )
        print_help();
      else if( cmd=='d' && n>=2 && a1<MAX_DRIVES )
        {
          Serial.print(F("Selecting drive ")); Serial.println(a1);
          motor(false);
          drivectrl_set(PIN_SRO_SELECT, HIGH);
          selDrive = a1;
          drivectrl_set(PIN_SRO_SELECT, LOW);
          drivectrl_set(PIN_SRO_DENSITY, driveType[selDrive]!=DT_5INCH_DD);
        }
      else if( cmd=='s' )
        {
          if( n>1 )
            {
              drivectrl_set(PIN_SRO_SIDE1, a1!=2);
              Serial.print(F("Selecting side "));
              Serial.println((drivectrl & bit(PIN_SRO_SIDE1)) ? '1' : '2');
            }
          else
            {
              Serial.print(F("Side "));
              Serial.print((drivectrl & bit(PIN_SRO_SIDE1)) ? '1' : '2');
              Serial.println(F(" is selected."));
            }
        }
      else if( cmd=='I' )
        {
          byte drive = n==1 ? selDrive : a1;

          if( sectorLength[drive]==0 || !SRI_BIT_SET(PIN_SRI_DSKCHG) )
            {
              bool motorOn = (drivectrl & bit(PIN_SRO_MOTOR))==0;
              if( !motorOn ) motor(true); else measure_sector(false);
              motor(motorOn);
            }

          Serial.println();
          if( n==1 ) { Serial.print(F("Current Drive     : ")); Serial.println(selDrive); }
          print_drive_params(drive);
          print_drive_timing(drive);
          Serial.print(F("Current Track     : ")); 
          if( current_track[drive]==0xFF ) Serial.println(F("unknown")); else Serial.println(current_track[drive]);
          Serial.println();
        }
      else if( cmd=='T' )
        {
          if( n==1 )
            print_drive_type_drive(selDrive);
          else
            {
              set_drive_type(selDrive, a1, false);
              drivectrl_set(PIN_SRO_DENSITY, a1!=DT_5INCH_DD);
              sectorLength[selDrive]=0;
            }
          Serial.println();
        }
      else if( cmd=='F' )
        {
          byte flags = EEPROM.read(4);
          if( n==2 )
            flags = (flags & ~0xC0) | ((a1 & 0x03) << 6);
          else if( n==3 && a1==0 )
            flags = (flags & ~0x07) | ((a2 & 0x07) << 0);
          else if( n==3 && a1==1 )
            flags = (flags & ~0x38) | ((a2 & 0x07) << 3);
          
          Serial.print(F("Drive 0 flags    : ")); Serial.println((flags >> 0) & 0x07);
          Serial.print(F("Drive 1 flags    : ")); Serial.println((flags >> 3) & 0x07);
          Serial.print(F("Controller flags : ")); Serial.println((flags >> 6) & 0x03);

          if( n>1 )
            {
              EEPROM.write(4, flags);
              
              // re-initialize drive type
              set_drive_type(0, driveType[0], false);
              set_drive_type(1, driveType[1], false);
            }
        }
      else if( cmd=='x' )
        break;

      if( motor_auto[selDrive] && (drivectrl & bit(PIN_SRO_MOTOR))==0 )
        motor_timeout[selDrive] = millis() + (driveType[selDrive]==DT_SA800 ? 100 : 5000);
    }
  
  drivectrl_set(PIN_SRO_SELECT, HIGH);
  drivectrl_set(PIN_SRO_MOTOR,  HIGH);
}


// ----------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------- controller functions --------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------


// status register bits
#define DRIVE_STATUS_ENWD      0x01
#define DRIVE_STATUS_MOVEHEAD  0x02
#define DRIVE_STATUS_HEAD      0x04
#define DRIVE_STATUS_SELECTED  0x08
#define DRIVE_STATUS_INTENBL   0x20
#define DRIVE_STATUS_TRACK0    0x40
#define DRIVE_STATUS_NRDA      0x80


// control register bits
#define DRIVE_CTRL_STEPIN      0x01
#define DRIVE_CTRL_STEPOUT     0x02
#define DRIVE_CTRL_HEADLOAD    0x04
#define DRIVE_CTRL_HEADUNLOAD  0x08
#define DRIVE_CTRL_INTENABLE   0x10
#define DRIVE_CTRL_INTDISABLE  0x20
#define DRIVE_CTRL_HEADCURRENT 0x40
#define DRIVE_CTRL_WRITE       0x80


// drive parameters
#define SECTOR_TRUE_PERIOD       30   // microseconds
#define HEAD_STEP_TIME           10   // milliseconds
#define HEAD_REST_TIME           22   // milliseconds
#define HEAD_LOAD_TIME           37   // milliseconds

#define MOTOR_OFF_DELAY        6400   // milliseconds
#define HEAD_UNLOAD_DELAY       100   // milliseconds
#define MOTOR_RUN_TIME         (driveType[selDrive]==DT_SA800 ? HEAD_UNLOAD_DELAY : MOTOR_OFF_DELAY)


// sector timer states
#define TS_NOT_SYNCED    0  // not synchronized, ignore sector timer events
#define TS_TRACK_START   1  // waiting for start of sector 0
#define TS_SECTOR_START  2  // waiting for start of sector 1+
#define TS_SECTOR_TRUE   3  // "sector true" period
#define TS_READ_CLEAR    4  // "read clear" period
#define TS_SECTOR_END    5  // for hard-sectored media, waiting for next sector index hole


int dataPtr = 0;
byte moveHeadTimeout = 0, sectorTimerState = 0, measureRotation = 0;
unsigned int headReadyTimeout = 0, motorTimeout[MAX_DRIVES];
bool disableSelectAfterStep = false, shortSector = false;
char doubleTrackStep;


inline bool check_measure_rotation()
{
  // if we are currently already measuring then don't start it again
  if( measureRotation>0 ) return false;

  // if disk has changed then re-measure the rotation time
  if( !SRI_BIT_SET(PIN_SRI_DSKCHG) ) 
    {
      if( driveType[selDrive]==DT_SA800 )
        {
          // clear "disk change" flag on Shugart SA-800
          drivectrl_set(PIN_SRO_SELECT, HIGH);
          drivectrl_set(PIN_SRO_SELECT, LOW);
        }
      else if( !SRI_BIT_SET(PIN_SRI_TRACK0) )
        {
          // 5.25" drives clear the "disk change" flag with "step" command.
          // if we are currently on track 0 then issue "step out" which will
          // be ignored by the drive but still reset the "disk change" flag,
          // otherwise just wait for it to get cleared by head movement.
          drivectrl_step_pulse(true);
        }
      
      sectorLength[selDrive] = 0;
    }
  
  if( sectorLength[selDrive]==0 )
    {
      // sector length unknown => perform spindle speed measurement
      sectorTimerState = TS_NOT_SYNCED;
      regStatus |= DRIVE_STATUS_HEAD;

      // set prescaler to /64 for measuring rotation
      TCCR1B = bit(CS11) | bit(CS10);

      // wait 5 index holes to measure (first pulse pluse 4 full rotations)
      measureRotation = 5;

      return true;
    }
  else
    return false;
}


inline void stop_measure_rotation()
{
  // no longer mearuring rotation speed
  measureRotation = 0; 

  // reset prescaler back to /8 for normal operation
  TCCR1B = bit(CS11);

  shortSector = false;
}


// -----------------------------------------------  drive control functions  --------------------------------------------------


void ctrl_step(bool stepOut)
{
  // if SELECT signal is not on then we need to turn it on temporarily
  // (will get turned off after step finishes)
  if( drivectrl & bit(PIN_SRO_SELECT) )
    { drivectrl_set(PIN_SRO_SELECT, LOW); disableSelectAfterStep = true; }
  else
    disableSelectAfterStep = false;

  // set STEPDIR and produce LOW->HIGH pulse on STEP pin
  // (prevent stepping out if already at track 0)
  if( !stepOut || SRI_BIT_SET(PIN_SRI_TRACK0) )
    drivectrl_step_pulse(stepOut);

  // Rotation speed in my 3.5" drive (TEAK FD235HG) sometimes slightly changes during 
  // stepping, causing the disk to be ahead or behind of the timer.
  // The "force sync after step" flag (in the drive settings flags) forces the drive
  // to wait for the INDEX hole after stepping, slowing down operation but making it 
  // much more reliable.
  if( (driveFlags[selDrive] & DF_FORCESYNC)!=0 && !hardSectored[selDrive] ) sectorTimerState = TS_NOT_SYNCED;

  // head is unavailable now for HEAD_REST_TIME period
  if( (regStatus & DRIVE_STATUS_HEAD)==0 || (headReadyTimeout>0) ) headReadyTimeout = HEAD_REST_TIME;

  // head can't be moved for HEAD_STEP_TIME period
#ifndef TRUE_DD
  if( driveType[selDrive]==DT_5INCH_DD )
    {
      // for double density disks (48TPI) we must take two (96TPI) steps
      // drive minimum is 3 us but we don't know where in the low-res timer 
      // cycle we are at the moment, so use 4us which will guarantee a minimum of 3
      moveHeadTimeout = 4;
      doubleTrackStep = stepOut ? 1 : -1;
    }
  else
#endif
    {
      moveHeadTimeout = HEAD_STEP_TIME;
      // Shugart SA800 needs some additional time after stepping if the SELECT
      // signal is getting turned off
      if( disableSelectAfterStep && driveType[selDrive]==DT_SA800 ) moveHeadTimeout += 5;
      doubleTrackStep = 0;
    }

  // reset automatic head timer for minidisk
  if( driveType[selDrive]==DT_5INCH_DD )
    motorTimeout[selDrive] = MOTOR_RUN_TIME;

  // stepping and reading not allowed
  regStatus |= DRIVE_STATUS_MOVEHEAD | DRIVE_STATUS_HEAD | DRIVE_STATUS_NRDA;
}


void ctrl_head_load()
{
  bool hadSelect = (drivectrl & bit(PIN_SRO_SELECT))==0;

  if( (regStatus & DRIVE_STATUS_HEAD)==0 )
    {
      // head is already loaded => nothing to do
    }
  else if( (drivectrl & bit(PIN_SRO_MOTOR)) )
    {
      // drive motor is not currently running => start it
      drivectrl_set(PIN_SRO_SELECT, LOW);
      drivectrl_set(PIN_SRO_MOTOR,  LOW);
      headReadyTimeout = HEAD_LOAD_TIME;

      // initiate disk rotation period measurement if necessary
      if( !check_measure_rotation() && !(driveType[selDrive]==DT_SA800 && hadSelect) )
        {
          // rotation period is known but motor was off
          // => wait for track start to sync up again
          // (except if this is a SA800 drive AND select was also on, i.e. we are still synced)
          // the drive will only produce INDEX pulses if it is ready (i.e. the motor has fully
          // spun up)
          sectorTimerState = TS_NOT_SYNCED;
          shortSector = false;
        }
    }
  else
    {
      // motor still running after previous unload (timeout not met)

      // make sure SELECT signal for current drive is on
      if( !hadSelect ) drivectrl_set(PIN_SRO_SELECT, LOW);

      // initiate disk rotation period measurement if necessary
      if( !check_measure_rotation() && !hadSelect )
        {
          // motor still running and rotation period known but drive SELECT was off
          // => wait for INDEX pulse to sync up again
          sectorTimerState = TS_NOT_SYNCED;
          headReadyTimeout = HEAD_LOAD_TIME;
          shortSector = false;

          // reset INDEX pin change flag
          PCIFR |= bit(PCIF0);
        }
      else
        {
          // we're still good after previous unload => just set "head loaded" flag
          regStatus &= ~DRIVE_STATUS_HEAD;
        }
    }

  // minidisk has an automatic head unload timeout
  if( driveType[selDrive]==DT_5INCH_DD )
    motorTimeout[selDrive] = MOTOR_RUN_TIME;
  else
    motorTimeout[selDrive] = 0;

  disableSelectAfterStep = false;
}


void ctrl_head_unload()
{
  regStatus |= DRIVE_STATUS_HEAD;
  headReadyTimeout = 0;
  if( measureRotation>0 ) stop_measure_rotation();
  
  // if motor is running then set timeout to turn it off
  if( !(drivectrl & bit(PIN_SRO_MOTOR)) && motorTimeout[selDrive]==0 ) 
    motorTimeout[selDrive] = MOTOR_RUN_TIME;
}


void ctrl_write_sector()
{
  // compute time that has passed already since the start of the sector.
  // the beginning of data (end of lead-in) on disk should be ~390 microseconds 
  // after the sector starts. write_sector_data() calculates the lead-in length
  // based on how much time has passed already.
  byte sector_start_offset;
  if( sectorTimerState==TS_READ_CLEAR )
    sector_start_offset = readClear[selDrive] - ((OCR1A-TCNT1)/2);
  else
    sector_start_offset = SECTOR_TRUE_PERIOD - ((OCR1A-TCNT1)/2);

  // receive sector data from CPU via bus and write it to disk
  // (note that SYNC bit is expected to be set in data sent by the CPU).
  // The ENWD status bit is only true while in while_sector_data.
  write_sector_data(dataBuffer, 137, sector_start_offset);

  if( hardSectored[selDrive] )
    {
      // hard sectored => set timeout to detect missed index hole
      OCR1A += sectorLength[selDrive] * 2;
      sectorTimerState = TS_SECTOR_END;
    }
  else
    {
      // soft sectored => adjust OCR1A to the start of the next sector
      if( sectorTimerState==TS_READ_CLEAR )
        OCR1A += (sectorLength[selDrive]-readClear[selDrive]) * 2;
      else
        OCR1A += (sectorLength[selDrive]-SECTOR_TRUE_PERIOD) * 2;

      sectorTimerState = TS_SECTOR_START;
    }
  
  // sector NOT true anymore
  regSector |= 1; 
  DBGPIN(PORTB, 1, 1);
      
  TIFR1 = bit(OCF1A); // clear timer 1 output compare flag
}


void ctrl_read_sector()
{
  // read the sector and send its data to the CPU via the bus
  // read_sector_data returns the index of the first byte
  // that was not already sent out while reading the data
  // read_sector_data aborts if a register is written, the write
  // is then processed above in our regular INP/OUT handler
  DBGPIN(PORTC, 3, 0);
  dataPtr = read_sector_data(dataBuffer, 137);
  DBGPIN(PORTC, 3, 1);
                    
  // if we have more data to send then clear the NRDA bit
  if( dataPtr<137 ) regStatus &= ~DRIVE_STATUS_NRDA;

  // timer 2 is reset at the beginning of read_sector_data and runs at 64us 
  // per tick so  dividing its count by 16 gives us roughly the time 
  // in milliseconds we spent in read_sector_data()
  byte ms = TCNT2 / 16;
                    
  // we miss the low-resolution timer ticks while reading sector data
  // this only affects the motor timeouts since reading/writing is
  // not allowed during head/step timeouts. 
  if( motorTimeout[0]>ms ) motorTimeout[0] -= ms;
  if( motorTimeout[1]>ms ) motorTimeout[1] -= ms;
}


void ctrl_reset()
{
  // resetting controller
  headReadyTimeout = 0;
  moveHeadTimeout = 0;
  regStatus = 0xFF;
  if( measureRotation>0 ) stop_measure_rotation();
  
  // if motor is running then set timeout to turn it off
  if( !(drivectrl & bit(PIN_SRO_MOTOR)) ) motorTimeout[selDrive] = MOTOR_RUN_TIME;
}


void ctrl_select_drive(byte drive)
{
  // selecting a drive
  if( drive!=selDrive )
    {
      // turn off SELECT signal for current drive
      drivectrl_set(PIN_SRO_SELECT, HIGH);
              
      // if motor of current drive is running then set timeout to turn it off
      if( !(drivectrl & bit(PIN_SRO_MOTOR)) ) motorTimeout[selDrive] = MOTOR_RUN_TIME;
              
      // select new drive
      selDrive = drive;
      drivectrl_set(PIN_SRO_DENSITY, driveType[selDrive]!=DT_5INCH_DD);
    }

  regStatus = 0xFF;
  if( driveType[selDrive]!=DT_NONE )
    regStatus &= ~(DRIVE_STATUS_SELECTED | DRIVE_STATUS_MOVEHEAD);
  if( measureRotation>0 ) stop_measure_rotation();
          
  // for 5.25 inch drives we turn on SELECT only while operating the drive
  // since the SELECT line controls the drive LED.
  // for the 8-inch Shugart drive we keep SELECT on as long as the drive is
  // selected because stepping requires extra time otherwise plus we save
  // time by not having to synchronize after loading the head (the
  // drive LED is controlled by the head load signal)
  if( (drivectrl & bit(PIN_SRO_SELECT))!=0 && driveType[selDrive]==DT_SA800 )
    {
      // drive SELECT is off => turn it on
      drivectrl_set(PIN_SRO_SELECT, LOW);

      // initiate disk rotation period measurement if necessary
      if( !check_measure_rotation() )
        {
          // rotation period known but drive SELECT was off
          // => wait for INDEX pulse to sync up again
          sectorTimerState = TS_NOT_SYNCED;
          headReadyTimeout = HEAD_LOAD_TIME;
          shortSector = false;
        }
    }

  // update TRACK0 status bit
  if( drivectrl & bit(PIN_SRO_SELECT) )
    {
      // drive SELECT is currently off => temporarily turn it on
      drivectrl_set(PIN_SRO_SELECT, LOW);
      if( !SRI_BIT_SET(PIN_SRI_TRACK0) ) regStatus &= ~DRIVE_STATUS_TRACK0;
      drivectrl_set(PIN_SRO_SELECT, HIGH);
    }
  else if( !SRI_BIT_SET(PIN_SRI_TRACK0) )
    regStatus &= ~DRIVE_STATUS_TRACK0;
}


// -------------------------------------------- handle bus activity (INP or OUT) ----------------------------------------------


void write_control_register(byte data)
{
  if( (data & (DRIVE_CTRL_STEPIN | DRIVE_CTRL_STEPOUT)) && (regStatus & DRIVE_STATUS_MOVEHEAD)==0 )
    {
      // step in/out
      ctrl_step((data & DRIVE_CTRL_STEPOUT)!=0);
    }

  if( (data & DRIVE_CTRL_HEADLOAD)!=0 )
    {
      // head load (88-DCDD) / head-load timer reset (88-MDS)
      ctrl_head_load();
    }

  if( (data & DRIVE_CTRL_HEADUNLOAD)!=0 && driveType[selDrive]!=DT_5INCH_DD )
    {
      // head unload (not used for minidisk)
      ctrl_head_unload();
    }
      
  if( (data & DRIVE_CTRL_WRITE)!=0 && ((regStatus & DRIVE_STATUS_HEAD)==0) )
    {
      // Write sequence start, only allow starting during "sector true" or "read clear" periods.
      // After that, writing the sector could overrun into the next sector. It does
      // not make sense to start writing later than that since reading what was
      // written would most likely fail. 
      if( (sectorTimerState==TS_SECTOR_TRUE) || (sectorTimerState==TS_READ_CLEAR) )
        ctrl_write_sector();
      else
        {
          // clear (i.e. enable) the ENWD flag which will allow the CPU to write data to register 2
          // all such data received will be ignored in the main loop
          regStatus &= ~DRIVE_STATUS_ENWD;
        }
    }
}


void write_drive_select_register(byte data)
{
  // writing drive select register
  if( (data & 0x80)!=0 )
    {
      // resetting controller
      ctrl_reset();
    }
  else if( (data&0x0F)<MAX_DRIVES && ((regStatus & DRIVE_STATUS_SELECTED)!=0 || (data&0x0F)!=selDrive) )
    {
      // selecting a drive
      ctrl_select_drive(data & 0x0F);
    }
}


void write_register(byte reg, byte data)
{
  if( reg==1 && (regStatus & DRIVE_STATUS_SELECTED)==0 )
    write_control_register(data);
  else if( reg==0 )
    write_drive_select_register(data);
  else if( reg==2 )
    {
      // writing data register => ignore
      // (receiving data is handled in function write_sector_data())
    }
  else if( reg==3 )
    {
      // setting drive flags
      EEPROM.write(4, data);

      // re-initialize drive type
      // determine drive parameters
      byte dip        = (~shift_in()) & 0x0F;
      bool minidisk   = (dip & 0x08)!=0;
      bool swapDrives = (dip & 0x04)!=0;
      
      // if minidisk system and only drive B is 5 inch then map drive 0 to drive B
      if( minidisk && (dip & 3)==1 ) swapDrives = true;

      set_drive_type(0, driveType[0], swapDrives);
      set_drive_type(1, driveType[1], swapDrives);
    }
}


byte read_register(byte reg)
{
  byte res = 0xFF;

  if( reg==2 )
    {
      // reading data register
      res = dataBuffer[dataPtr];
      if( dataPtr < 137 ) 
        dataPtr++;
      else
        regStatus |= DRIVE_STATUS_NRDA;
    }
  else if( reg==1 && (regStatus & DRIVE_STATUS_HEAD)==0 )
    {
      // reading sector register (only enabled if head is ready)
      res = regSector;
    }
  else if( reg==0 )
    {
      // reading status register
      res = regStatus;
    }
  else if( reg==3 )
    {
      // read drive flags
      res = EEPROM.read(4);
    }
  
  return res;
}


void handle_bus_communication()
{
  // this must be either an INP (PC4 high) or an OUT (PC5 high) request, 
  // never both together (CPU can only do one at a time)
  // we always wait for the PC4/PC5 signals to go low again and then
  // reset the notification flag, so at this point they must have gone high
  if( PINC & 0x10 )
    {
      // bus input (controller -> CPU)
      byte reg = PINC & 0x03;     // get register address
      PORTD = read_register(reg); // read register and write value to PORTD
      DDRD = 0xFF;                // switch data bus pins to output
      PORTB &= ~0x20;             // set WAIT signal to LOW
      while( (PINC & 0x10) );     // wait until INP signal ends
      DDRD = 0x00;                // switch data bus pins back to input
      PORTB |= 0x20;              // set WAIT signal back to HIGH
      PCIFR |= bit(PCIF1);        // reset INP/OUT pin change flag
    }
  else if( PINC & 0x20 )
    {
      // bus output (CPU -> controller)
      byte reg = PINC & 0x03;     // get register address
      byte data = PIND;           // get input data
      PORTB &= ~0x20;             // set WAIT signal to LOW
      while( (PINC & 0x20) );     // wait until OUT signal ends
      PORTB |= 0x20;              // set WAIT signal back to HIGH
      PCIFR |= bit(PCIF1);        // reset INP/OUT pin change flag
      write_register(reg, data);  // write register
    }
}


// ---------------------------------------------- handle INDEX signal pin change ------------------------------------------------


void handle_index_signal()
{
  // if BOTH the foundIndex flag and PCIF0 are set then we handle
  // foundIndex first, PCIF0 will be handled the next time we get here
  if( (PINB & 0x08)==0 || foundIndex )
    {
      unsigned int t;

      if( foundIndex )
        {
          // INDEX pin change was detected during a previous
          // read_sector_data or write_sector_data operation
          t = indexTime;
          TCNT1 -= t;
        }
      else
        {
          // INDEX pin change was detected now
          t = TCNT1;
          TCNT1 = 0;
        }

      if( measureRotation>0 )
        {
          // currently measuring rotation period of the spindle to calibrate sector timer
          static unsigned long rotationPeriod;

          measureRotation--;
          rotationPeriod = measureRotation<4 ? rotationPeriod+t : 0;
                  
          if( measureRotation==0 )
            {
              set_sector_length(selDrive, rotationPeriod);
              stop_measure_rotation();
              TCNT1 = 0;
            }
        }

      if( measureRotation==0 )
        {
          if( hardSectored[selDrive] )
            {
              // check if time since last sector hole was less than 3/4 of a sector length
              // (prescaler is at /8, i.e. 2 ticks per microsecond => (x*2)*(3/4) = (x*3)/2)
              if( t < (sectorLength[selDrive]*3)/2 )
                {
                  // two short sector index periods in a row indicate start of sector 0
                  if( shortSector ) 
                    { 
                      OCR1A = sectorOffset[selDrive] * 2;
                      sectorTimerState = TS_TRACK_START;
                    }
                          
                  shortSector = true;
                }
              else 
                {
                  // regular (long) sector start index pulse
                  if( sectorTimerState!=TS_NOT_SYNCED )
                    {
                      OCR1A = sectorOffset[selDrive] * 2;
                      sectorTimerState = TS_SECTOR_START;
                    }

                  shortSector = false;
                }
            }
          else
            {
              // soft-sectored media
              OCR1A = sectorOffset[selDrive] * 2;
              sectorTimerState = TS_TRACK_START;
            }
        }

      // clear timer1 output compare flag
      TIFR1 = bit(OCF1A);
    }

  // reset INDEX pin change flag
  if( foundIndex )
    foundIndex = 0;
  else
    PCIFR |= bit(PCIF0);
}


// -----------------------------------------------  handle low-resolution timer  ------------------------------------------------


void handle_lowres_timer()
{
  // head move timeout
  if( moveHeadTimeout>0 && --moveHeadTimeout==0 )
    {
      if( doubleTrackStep!=0 )
        {
          drivectrl_step_pulse(doubleTrackStep>0);
          moveHeadTimeout = HEAD_STEP_TIME-3;
          doubleTrackStep = 0;
        }
      else
        {
          // check TRACK0 signal
          if( SRI_BIT_SET(PIN_SRI_TRACK0) )
            regStatus |=  DRIVE_STATUS_TRACK0;
          else
            regStatus &= ~DRIVE_STATUS_TRACK0;

          // turn off the SELECT signal again if required
          // (needed to be enabled to do the STEP pulse)
          if( disableSelectAfterStep ) { drivectrl_set(PIN_SRO_SELECT, HIGH); disableSelectAfterStep = false; }
                  
          // ready to move head again
          regStatus &= ~DRIVE_STATUS_MOVEHEAD;
        }
    }

  // head loaded/ready timeout
  if( headReadyTimeout>0 && --headReadyTimeout==0 )
    {
      if( sectorTimerState==TS_NOT_SYNCED )
        headReadyTimeout++; // wait longer
      else
        regStatus &= ~DRIVE_STATUS_HEAD;
    }

  // motor turn-off timeout drive 0
  if( motorTimeout[0]>0 && --motorTimeout[0]==0 ) 
    { 
      drivectrl_set(pinMotor[0], HIGH); 
      if( driveType[0]!=DT_SA800 ) drivectrl_set(pinSelect[0], HIGH); 
      if( selDrive==0 ) regStatus |= DRIVE_STATUS_HEAD; 
    }

  // motor turn-off timeout drive 1
  if( motorTimeout[1]>0 && --motorTimeout[1]==0 ) 
    { 
      drivectrl_set(pinMotor[1], HIGH); 
      if( driveType[1]!=DT_SA800 ) drivectrl_set(pinSelect[1], HIGH); 
      if( selDrive==1 ) regStatus |= DRIVE_STATUS_HEAD; 
    }

  // reset timer compare match flag
  TIFR0 |= bit(OCF0A);
}


// ---------------------------------------------------  handle sector timer  ----------------------------------------------------


void handle_sector_timer()
{
  switch( sectorTimerState )
    {
    case TS_NOT_SYNCED:
      {
        // not synchronized, ignore sector timer events
        // this state is exited when a track-start index hole is found in handle_index_signal()
        break;
      }

    case TS_TRACK_START:
      {
        // regSector will roll over to 0 in TS_SECTOR_START case below 
        regSector = 0xFF;

        // head is ready now
        if( headReadyTimeout==1 ) { regStatus &= ~DRIVE_STATUS_HEAD; headReadyTimeout=0; }

        // (intentional fall-through)
      }

    case TS_SECTOR_START:
      {
        // start of next sector's "sector true" period
        regStatus |= DRIVE_STATUS_NRDA | DRIVE_STATUS_ENWD; // not ready to send or receive data

        if( (regSector != (2*numSectors[selDrive]-1)) || hardSectored[selDrive] )
          {
            // for hard-sectored media we may miss the track-start index hole
            // when the head is loaded and synced because read/write operations
            // are taking place => allow to roll over to sector 0 since the 
            // sector-start index holes keep us in sync with the disk
            regSector++;
            if( regSector==2*numSectors[selDrive] ) regSector = 0;
            if( regSector==0 ) DBGPULSE(PORTB, 1);
            OCR1A += SECTOR_TRUE_PERIOD * 2;
            sectorTimerState = TS_SECTOR_TRUE;
            DBGPIN(PORTB, 1, 0);
          }
        else
          {
            // for soft-sectored media do not roll over from the last sector to sector 0,
            // we will go to sector 0 when the INDEX hole is detected
            regSector |= 1; 
            DBGPIN(PORTB, 1, 1);
            sectorTimerState = TS_NOT_SYNCED;
          }
                
        break;
      }

    case TS_SECTOR_TRUE:
      {
        // end of "sector true" period
        regSector |= 1;
        DBGPIN(PORTB, 1, 1);
                
        if( (regStatus & DRIVE_STATUS_HEAD)==0 )
          { 
            // head is ready and sector has started
            // => wait until the end of the sector's read-clear period
            OCR1A += (readClear[selDrive]-SECTOR_TRUE_PERIOD) * 2;
            sectorTimerState = TS_READ_CLEAR;
          }
        else if( hardSectored[selDrive] )
          {
            // head not ready, hard sectored => set timeout for detecting missed index hole
            OCR1A += sectorLength[selDrive] * 3;
            sectorTimerState = TS_SECTOR_END;
          }
        else
          {
            // head not ready, soft sectored => wait for beginning of next sector
            OCR1A += (sectorLength[selDrive] - SECTOR_TRUE_PERIOD) * 2;
            sectorTimerState = TS_SECTOR_START;
          }

        break;
      }
              
    case TS_READ_CLEAR:
      {
        // readFinished will be set "true" if we read the full sector
        readFinished = 0;

        // end of read-clear period => make sure the head is still ready
        // and if so read the sector
        if( (regStatus & DRIVE_STATUS_HEAD)==0 )
          ctrl_read_sector();
        
        // OCR1A is currently set to the end of the sector's read-clear period
        if( hardSectored[selDrive] )
          {
            // hard sectored => set timeout to detect missed index hole
            OCR1A += sectorLength[selDrive] * 2;
            sectorTimerState = TS_SECTOR_END;
          }
        else
          {
            // soft sectored => adjust OCR1A to the start of the next sector
            if( (driveFlags[selDrive] & DF_RELSECTOR)!=0 && readFinished )
              {
                // we read a full sector => estimate beginning of next sector 
                // based on end time of this sector
                OCR1A = readDoneTime + sectorTail[selDrive] * 2;
              }
            else
              {
                // sector was only partially read => estimate beginning of next sector
                // based on beginning time of this sector
                OCR1A += (sectorLength[selDrive] - readClear[selDrive]) * 2;
              }
            
            sectorTimerState = TS_SECTOR_START;
          }
        break;
      }

    case TS_SECTOR_END:
      {
        // we expected to have seen a sector index hole (hard-sectored media)
        // by now but didn't => lost sync, must re-sync
        // (this will happen if the user removes the disk or we miss an index
        //  hole for unknown reasons)
        DBGPIN(PORTB, 1, 1);
        sectorTimerState = TS_NOT_SYNCED;

        // check whether head was ready before 
        if( (regStatus & DRIVE_STATUS_HEAD)==0 )
          {
            // head not ready now
            regStatus |= DRIVE_STATUS_HEAD;
            
            // head will be ready again as soon as we re-sync
            if( headReadyTimeout==0 ) headReadyTimeout = 1;
          }

        break;
      }
    }

  TIFR1 = bit(OCF1A); // clear OCF1A
}


// -----------------------------------------------------  controller loop ------------------------------------------------------


void controller()
{
  // disable interrupts as they would disrupt sensitive timing
  noInterrupts();

  // WAIT was initially set to LOW to release a potentially waiting CPU
  // => set it to HIGH now for proper operation
  digitalWrite(PIN_WAIT, HIGH);

  // determine drive parameters
  byte dip        = (~shift_in()) & 0x0F;
  bool minidisk   = (dip & 0x08)!=0;
  bool swapDrives = (dip & 0x04)!=0;

  // if minidisk system and only drive B is 5 inch then map drive 0 to drive B
  if( minidisk && (dip & 3)==1 ) swapDrives = true;

  // swap drives if requested
  byte d0 = 0, d1 = 1;
  if( swapDrives ) { d0 = 1; d1 = 0; }

  // set drive types
  set_drive_type(d0, (dip & 1)!=0 ? (minidisk ? DT_NONE : DT_SA800) : (minidisk ? DT_5INCH_DD : DT_5INCH_HD), d0!=0);
  set_drive_type(d1, (dip & 2)!=0 ? (minidisk ? DT_NONE : DT_SA800) : (minidisk ? DT_5INCH_DD : DT_5INCH_HD), d0!=0);
  pinSelect[d0] = PIN_SRO_SELECT0; pinMotor[d0] = PIN_SRO_MOTOR0;
  pinSelect[d1] = PIN_SRO_SELECT1; pinMotor[d1] = PIN_SRO_MOTOR1;

  // don't know spindle rotation time yet
  measureRotation = 0;
  sectorLength[0] = 0;
  sectorLength[1] = 0;
  motorTimeout[0] = 0;
  motorTimeout[1] = 0;
  hardSectored[0] = false;
  hardSectored[1] = false;

  // select drive 0
  selDrive = 0;
  drivectrl_set(PIN_SRO_DENSITY, driveType[selDrive]!=DT_5INCH_DD);

  DDRD = 0x00; // make sure data bus pins are set to input
  PCIFR = bit(PCIF0) | bit(PCIF1); // reset any pin-change flags

  // set timer 0 for a 1000Hz frequency (1 millisecond resolution)
  TCNT0  = 0; // reset timer 0
  TIMSK0 = 0; // disable interrupts for timer 0
  TCCR0A = bit(WGM01); // clear-timer-on-compare-match mode
  TCCR0B = bit(CS00) | bit(CS01); // /64 prescaler (16MHz clock => 250000 timer ticks per second)
  OCR0A  = 249; // output compare match after 250 ticks
  TIFR0 |= bit(OCF0A); // clear timer overflow register

  // not synchronized
  sectorTimerState = TS_NOT_SYNCED;
  foundIndex = 0;

  // start sector timer (prescaler /8)
  TCCR1B = bit(CS11);

  // main controller loop
  while( true )
    {
      // check for bus activity pin change (INP or OUT)
      if( PCIFR & bit(PCIF1) )
        handle_bus_communication();

      // check for INDEX signal pin change
      if( PCIFR & bit(PCIF0) || foundIndex )
        handle_index_signal();
      
      // check for low-resolution timer overflow (timer 0, overflows after 1ms)
      if( TIFR0 & bit(OCF0A) )
        handle_lowres_timer();

      // check for sector timer compare match (timer 1)
      if( TIFR1 & bit(OCF1A) )
        handle_sector_timer();
    }
}


// ----------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------- main functions -----------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------


void setup() 
{
  TCCR1A = 0;  
  TCCR1B = 0;
  TCCR1C = 0;  
  TCCR2A = 0;
  TCCR2B = 0;

  // set up pin modes 
  digitalWrite(PIN_SRDATA,    HIGH);
  digitalWrite(PIN_SRCLOCK,   HIGH);
  digitalWrite(PIN_SROLATCH,  LOW);
  digitalWrite(PIN_SRILATCH,  LOW);
  digitalWrite(PIN_WRITEDATA, HIGH);
  digitalWrite(PIN_WAIT,      LOW);
  pinMode(PIN_SRDATA,    OUTPUT);
  pinMode(PIN_SRCLOCK,   OUTPUT);
  pinMode(PIN_SROLATCH,  OUTPUT);
  pinMode(PIN_WRITEDATA, OUTPUT);
  pinMode(PIN_INDEX,     INPUT);
  pinMode(PIN_SRILATCH,  OUTPUT);
  pinMode(PIN_READDATA,  INPUT);
  pinMode(PIN_A0,        INPUT);
  pinMode(PIN_A1,        INPUT);
  pinMode(PIN_WAIT,      OUTPUT);
  pinMode(PIN_INP,       INPUT);
  pinMode(PIN_OUT,       INPUT);
  DDRD = 0x00;  // set digital pins 0-7 (PORTD) to input

  // set all shift register output pins to HIGH (i.e. disabled)
  drivectrl = 0xFF;
  shift_out(drivectrl);
  
  // initialize drive registers
  regSector  = 0xFF;
  regStatus  = 0xFF;

  // set up pin change notifications for OUT, INP and INDEX signals
  PCMSK1 |= bit(PCINT12);    // INP
  PCMSK1 |= bit(PCINT13);    // OUT
  PCMSK0 |= bit(PCINT3);     // INDEX
  PCIFR = bit(PCIF0) | bit(PCIF1);
  
  // initialize EEPROM configuration if necessary
  if( EEPROM.read(0)!='F' || EEPROM.read(1)!='D' || EEPROM.read(2)!='C' )
    {
      EEPROM.write(0, 'F');
      EEPROM.write(1, 'D');
      EEPROM.write(2, 'C');
      EEPROM.write(3, 0); // config version 0
      EEPROM.write(4, 0); // drive flags=0
    }

  // start timer 2 with prescaler /1024 = 64us tick frequency
  // (used to measure time during read operations)
  TCCR2B = bit(CS22) | bit(CS21) | bit(CS20);
}
 

void loop() 
{
#if MONITOR>0
  Serial.begin(115200);

#if MONITOR==1
  monitor();
#else
  if( !SRI_BIT_SET(PIN_SRI_MONITOR) ) monitor();
#endif
  
  // the data bus pins include D0 and D1 which are used for serial
  // communication => must disable Serial for controller operation
  Serial.println();
  Serial.flush();
  Serial.end();
#endif
 
  // enter controller (infinite loop)
  controller();

}
