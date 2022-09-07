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

#include "Altair8800.h"
#include "config.h"
#include "mem.h"
#include "serial.h"
#include "printer.h"
#include "filesys.h"
#include "numsys.h"
#include "drive.h"
#include "cdrive.h"
#include "tdrive.h"
#include "hdsk.h"
#include "prog.h"
#include "dazzler.h"
#include "sdmanager.h"
#include "vdm1.h"
#include "cpucore.h"
#include "io.h"

#define CONFIG_FILE_VERSION 10

#define BAUD_110     0
#define BAUD_150     1
#define BAUD_300     2
#define BAUD_600     3
#define BAUD_1200    4
#define BAUD_2400    5
#define BAUD_4800    6
#define BAUD_9600    7
#define BAUD_19200   8
#define BAUD_38400   9
#define BAUD_57600   10
#define BAUD_115200  11
#define BAUD_250000  12
#define BAUD_525000  13
#define BAUD_750000  14
#define BAUD_1050000 15

#if HOST_NUM_SERIAL_PORTS>5
#error "Maximum number of host serial interfaces supported is 5"
#endif


// current configuration number
static byte config_current = 0;

// config_flags:
// vvvvvvvv mmmpphrt ttttRRRR dVCDIPFT
// T = Throttle
// t = Throttle delay if throttle is enabled (0=auto)
// F = Profile
// P = Serial Panel
// I = Serial Input
// D = Serial Debug
// C = Clear memory on powerup
// V = VI board installed
// R = RTC rate
// d = force real-time mode for disk drives
// p = printer type (00=NONE, 01=Okidata/88-LPC, 02=C700, 03=Generic)
// m = map printer to host interface (00=NONE, 01=primary, 02=secondary)
// r = real-time mode for printer
// h = real-time mode for hard drives
// v = config file version
uint32_t config_flags;


// config_flags2:
// xxxxxxxx BAPKKKMM MMMMDDDD DDVVVZZZ
// ZZZ  = map dazzler to host interface (000=NONE, 001=1st, 010=2nd, 011=3rd, 100=4th, 101=5th)
// VVV  = map VDM-1   to host interface (see above)
// D    = VDM-1 dip switch settings
// M    = VDM-1 memory address (6 highest bits)
// KKK  = map VDM-1 keyboard to serial device (000=NONE, 1=SIO, 2=ACR, 3=2SIO1, 4=2SIO2, 5=2SIO3, 6=2SIO4)
// P    = Processor (0=i8080, 1=z80)
// B    = B Mode - Aux1 is Display/Load, Aux2 is Input/Output, Examine+Aux1 for classic functions.
//        
uint32_t config_flags2;


// config_serial_settings:
// xxxxxxxx 44443333 2222xPPP 11110000
// 0000 = baud rate for first  host interface (see baud rates above)
// 1111 = baud rate for second host interface (see baud rates above)
// 2222 = baud rate for third  host interface (see baud rates above)
// 3333 = baud rate for fourth host interface (see baud rates above)
// 4444 = baud rate for fifth  host interface (see baud rates above)
// PPP  = primary serial interface (maximum number depends on host)
// x    = unused
uint32_t config_serial_settings, new_config_serial_settings;


// cofig_serial_settings2:
// xxxxxxxB BPPSBBPP SBBPPSBB PPSBBPPS
// for all 5 host interfaces:
// BB = number of bits (0=5, 1=6, 2=7, 3=8)
// PP = parity         (0=none, 1=even, 2=odd)
// S  = stop bits      (0=1, 1=2)
uint32_t config_serial_settings2, new_config_serial_settings2;


// config_serial_device_settings[0-5]
// xxxxxxxx xxxxMMMR TT77UUVV CNNNBBBB
// BBBB = baud rate for serial playback (see baud rates above)
// NNN  = NULs to send after a carriage return when playing back examples
// C    = trap CLOAD/CSAVE in extended BASIC (for CSM_ACR device only)
// MMM  = map device to host interface (000=NONE, 001=first, 010=second, 011=third, 100=fourth, 101=fifth, 111=primary)
// UU   = only uppercase for inputs (00=off, 01=on, 10=autodetect)
// 77   = use 7 bit for serial outputs (00=off [use 8 bit], 01=on, 10=autodetect)
// TT   = translate backspace to (00=off, 01=underscore, 10=autodetect, 11=delete)
// R    = force realtime operation (use baud rate even if not using interrupts)
// VV   = 88-SIO board version (0=rev0, 1=rev1, 2=Cromemco)
uint32_t config_serial_device_settings[NUM_SERIAL_DEVICES];

// map emulated device (SIO/2SIO etc.) to host serial port number
byte config_serial_sim_to_host[NUM_SERIAL_DEVICES];

// masks defining which interrupts (INT_*) are at which vector interrupt levels
uint32_t config_interrupt_vi_mask[8];


// mask defining whch interrupts (INT_*) are connected if VI board is not installed
uint32_t config_interrupt_mask;


// program to be run when AUX1 is raised
byte config_aux1_prog;


// amount of RAM installed
uint32_t config_mem_size;

// status bytes for generic printer emulation
byte config_printer_generic_status_busy;
byte config_printer_generic_status_ready;


// --------------------------------------------------------------------------------


static bool config_read_string(char* buf, byte bufsize)
{
	int l = 0;

	while (true)
	{
		int c = serial_read();
		if (c >= 32 && c < 127)
		{
			if (l < bufsize - 1) { buf[l++] = c; Serial.write(c); }
		}
		else if (c == 8 || c == 127)
		{
			if (l > 0)
			{
				l--;
				Serial.print(F("\010 \010"));
			}
		}
		else if (c == 13)
		{
			Serial.println();
			buf[l] = 0;
			return true;
		}
		else if (c == 27)
		{
			Serial.println();
			return false;
		}
	}

	return true;
}


inline uint32_t get_bits(uint32_t v, byte i, byte n)
{
	return (v >> ((uint32_t)i)) & ((1ul << n) - 1);
}


inline uint32_t set_bits(uint32_t v, byte i, byte n, uint32_t nv)
{
	uint32_t mask = ((1ul << n) - 1) << i;
	return (v & ~mask) | ((nv << i) & mask);
}


static uint32_t toggle_bits(uint32_t v, byte i, byte n, byte min = 0x00, byte max = 0xff)
{
	byte b = get_bits(v, i, n) + 1;
	return set_bits(v, i, n, b > max ? min : (b < min ? min : b));
}


static uint32_t toggle_vdm1_dip(uint32_t v, byte i, bool allowBoth)
{
	switch (get_bits(v, i, 2))
	{
	case 0: return set_bits(v, i, 2, 2);
	case 2: return set_bits(v, i, 2, 1);
	case 1: return set_bits(v, i, 2, allowBoth ? 3 : 0);
	case 3: return set_bits(v, i, 2, 0);
	}

	return v;
}


static byte config_baud_rate_bits(byte iface)
{
	byte n = 0;
	switch (iface)
	{
	case 0: n = 0;  break;
	case 1: n = 4;  break;
	case 2: n = 12; break;
	case 3: n = 16; break;
	case 4: n = 20; break;
	}

	return n;
}

static uint32_t config_baud_rate(byte b)
{
	uint32_t res;
	switch (b)
	{
	case BAUD_110: res = 110;     break;
	case BAUD_150: res = 150;     break;
	case BAUD_300: res = 300;     break;
	case BAUD_600: res = 600;     break;
	case BAUD_1200: res = 1200;    break;
	case BAUD_2400: res = 2400;    break;
	case BAUD_4800: res = 4800;    break;
	case BAUD_9600: res = 9600;    break;
	case BAUD_19200: res = 19200;   break;
	case BAUD_38400: res = 38400;   break;
	case BAUD_57600: res = 57600;   break;
	case BAUD_115200: res = 115200;  break;
	case BAUD_250000: res = 250000;  break;
	case BAUD_525000: res = 525000;  break;
	case BAUD_750000: res = 750000;  break;
	case BAUD_1050000: res = 1050000; break;
	default: res = 115200;  break;
	}

	return res;
}


float config_rtc_rate()
{
	float res = 0.0;

	byte value = get_bits(config_flags, 8, 4);
	if (value & 0x08)
	{
		switch (value & 0x07)
		{
		case 0: res = 0.06f; break;
		case 1: res = 0.60f; break;
		case 2: res = 6.00f; break;
		case 3: res = 10.00f; break;
		case 4: res = 60.00f; break;
		case 5: res = 100.00f; break;
		case 6: res = 1000.00f; break;
		case 7: res = 10000.00f; break;
		}
	}

	return res;
}


#if USE_THROTTLE>0
int config_throttle()
{
	if (config_flags & CF_THROTTLE)
	{
		int i = get_bits(config_flags, 12, 5);
		if (i == 0)
			return -1; // auto
		else
			return i;  // manual
	}
	else
		return 0; // off
}
#endif

byte config_aux1_program()
{
	return config_aux1_prog;
}


byte config_serial_backspace(byte dev, uint16_t PC)
{
	byte b = get_bits(config_serial_device_settings[dev], 14, 2);

	if (b == CSFB_AUTO)
	{
		if (PC == 0x038A || PC == 0x0380 || PC == 0x00A0 || PC == 0x00AC)
		{
			// ALTAIR 4k BASIC I/O routine has "IN" instruction at 0x389 and "OUT" instruction at 0x037F
			// ALTAIR EXTENDED BASIC I/O routine has "IN" instruction at 0x09f and "OUT" instruction at 0x00AB
			b = CSFB_UNDERSCORE;
		}
		else if (PC == 0x0726 || PC == 0x08e5)
		{
			// MITS Programming System II has "IN" instruction at 0x0725 and 0x07E4
			b = CSFB_DELETE;
		}
		else
			b = CSFB_NONE;
	}

	return b;
}


bool config_serial_ucase(byte dev, uint16_t PC)
{
	byte b = get_bits(config_serial_device_settings[dev], 10, 2);
	if (b == CSF_AUTO)
	{
		// ALTAIR 4k BASIC I/O routine has "IN" instruction at 0x0389
		// MITS Programming System II has "IN" instruction at 0x0725 and 0x07E4
		return (PC == 0x038A) || (PC == 0x0726) || (PC == 0x08e5);
	}
	else
		return b == CSF_ON;
}


bool config_serial_7bit(byte dev, uint16_t PC)
{
	byte b = get_bits(config_serial_device_settings[dev], 12, 2);
	if (b == CSF_AUTO)
	{
		// ALTAIR 4k BASIC I/O routine has "OUT" instruction at 0x037F
		// ALTAIR EXTENDED BASIC I/O routine has "OUT" instruction at 0x00AB
		// MITS Programming System II has "OUT" instruction at 0x071B
		return (PC == 0x0380) || (PC == 0x00AC) || (PC == 0x071C);
	}
	else
		return b == CSF_ON;
}


byte config_serial_siorev()
{
	return get_bits(config_serial_device_settings[CSM_SIO], 8, 2);
}

bool config_serial_trap_CLOAD()
{
	return get_bits(config_serial_device_settings[CSM_ACR], 7, 1) > 0;
}


uint32_t config_serial_playback_baud_rate(byte dev)
{
	return config_baud_rate(get_bits(config_serial_device_settings[dev], 0, 4));
}

byte config_serial_playback_example_nuls(byte dev)
{
	return get_bits(config_serial_device_settings[dev], 4, 3);
}

byte config_map_device_to_host_interface(byte s)
{
	if (s == 7)
		return config_host_serial_primary();
	else if (s <= HOST_NUM_SERIAL_PORTS)
		return s - 1;
	else
		return 0xff;
}

bool config_serial_realtime(byte dev)
{
	return get_bits(config_serial_device_settings[dev], 16, 1) ? true : false;
}

byte config_host_serial_primary()
{
	return get_bits(config_serial_settings, 8, 3);
}

uint32_t config_host_serial_baud_rate(uint32_t settings, byte iface)
{
	return config_baud_rate(get_bits(settings, config_baud_rate_bits(iface), 4));
}


uint32_t config_host_serial_config(uint32_t settings2, byte iface)
{
	byte v = get_bits(settings2, iface * 5, 5);
	switch (v)
	{
#ifndef HOST_TEENSY_H // Teensy does not define these constants
	case 0x00: return SERIAL_5N1;
	case 0x01: return SERIAL_5N2;
	case 0x02: return SERIAL_5E1;
	case 0x03: return SERIAL_5E2;
	case 0x04: return SERIAL_5O1;
	case 0x05: return SERIAL_5O2;

	case 0x08: return SERIAL_6N1;
	case 0x09: return SERIAL_6N2;
	case 0x0A: return SERIAL_6E1;
	case 0x0B: return SERIAL_6E2;
	case 0x0C: return SERIAL_6O1;
	case 0x0D: return SERIAL_6O2;

	case 0x10: return SERIAL_7N1;
	case 0x11: return SERIAL_7N2;
	case 0x12: return SERIAL_7E1;
	case 0x13: return SERIAL_7E2;
	case 0x14: return SERIAL_7O1;
	case 0x15: return SERIAL_7O2;
#endif
	case 0x18: return SERIAL_8N1;
	case 0x19: return SERIAL_8N2;
	case 0x1A: return SERIAL_8E1;
	case 0x1B: return SERIAL_8E2;
	case 0x1C: return SERIAL_8O1;
	case 0x1D: return SERIAL_8O2;

		// fall back to default 8N1 settings
	default: return SERIAL_8N1;
	}
}


uint32_t config_host_serial_config(byte iface)
{
	return config_host_serial_config(config_serial_settings2, iface);
}


uint32_t config_host_serial_baud_rate(byte iface)
{
	return config_host_serial_baud_rate(config_serial_settings, iface);
}


byte config_printer_map_to_host_serial()
{
	return config_map_device_to_host_interface(get_bits(config_flags, 21, 3));
}


byte config_printer_type()
{
	return get_bits(config_flags, 19, 2);
}


byte config_printer_generic_get_status(bool busy)
{
	return busy ? config_printer_generic_status_busy : config_printer_generic_status_ready;
}


byte config_dazzler_interface()
{
	return config_map_device_to_host_interface(get_bits(config_flags2, 0, 3));
}


byte config_vdm1_interface()
{
	return config_map_device_to_host_interface(get_bits(config_flags2, 3, 3));
}


byte config_vdm1_dip()
{
	return get_bits(config_flags2, 6, 6);
}


byte config_vdm1_keyboard_device()
{
	return ((byte)get_bits(config_flags2, 18, 3)) - 1;
}


uint16_t config_vdm1_address()
{
	return get_bits(config_flags2, 12, 6) * 1024;
}


// --------------------------------------------------------------------------------


static void set_cursor(byte row, byte col)
{
	Serial.print(F("\033["));
	Serial.print(row);
	Serial.print(';');
	Serial.print(col);
	Serial.print(F("H\033[K"));
}


static void print_cpu()
{
	if (config_use_z80())
		Serial.print(F("Zilog Z80"));
	else
		Serial.print(F("Intel 8080"));
}


static void print_mem_size(uint32_t s, byte row = 0, byte col = 0)
{
	if (row != 0 || col != 0) set_cursor(row, col);

	if ((s & 0x3FF) == 0)
	{
		Serial.print(s / 1024);
		Serial.print(F(" KB"));
	}
	else
	{
		Serial.print(s);
		Serial.print(F(" bytes"));
	}
}


static void print_flag(uint32_t data, uint32_t value, byte row = 0, byte col = 0)
{
	if (row != 0 || col != 0) set_cursor(row, col);
	Serial.print((data & value) != 0 ? F("yes") : F("no"));
}


static void print_flag(uint32_t value, byte row = 0, byte col = 0)
{
	print_flag(config_flags, value, row, col);
}


static void print_flag2(uint32_t value, byte row = 0, byte col = 0)
{
	print_flag(config_flags2, value, row, col);
}


static void print_vi_flag()
{
	Serial.print((config_flags & CF_HAVE_VI) != 0 ? F("Use Vector Interrupt board") : F("Interrupts connected directly to CPU"));
}


static void print_host_serial_config(uint32_t settings2, byte iface)
{
	byte v = get_bits(settings2, iface * 5, 5);
	Serial.print(get_bits(settings2, iface * 5 + 3, 2) + 5);
	switch (get_bits(settings2, iface * 5 + 1, 2))
	{
	case 0: Serial.print('N'); break;
	case 1: Serial.print('E'); break;
	case 2: Serial.print('O'); break;
	case 3: Serial.print('?'); break;
	}
	Serial.print(get_bits(settings2, iface * 5, 1) + 1);
}


static void print_host_serial_config(byte iface, byte row, byte col)
{
	if (row != 0 || col != 0) set_cursor(row, col);
	Serial.print(config_host_serial_baud_rate(new_config_serial_settings, iface));
	Serial.print(F(" baud"));
	if (host_serial_port_has_configs(iface))
	{
		Serial.print(' ');
		print_host_serial_config(new_config_serial_settings2, iface);
	}

	if (config_host_serial_baud_rate(new_config_serial_settings, iface) != config_host_serial_baud_rate(iface)
		||
		get_bits(new_config_serial_settings2, iface * 5, 5) != get_bits(config_serial_settings2, iface * 5, 5))
	{
		Serial.print(F(" (current: "));
		Serial.print(config_host_serial_baud_rate(iface));
		if (host_serial_port_has_configs(iface))
		{
			Serial.print(' ');
			print_host_serial_config(config_serial_settings2, iface);
		}
		Serial.print(')');
	}
}


static void print_throttle(byte row = 0, byte col = 0)
{
	if (row != 0 || col != 0) set_cursor(row, col);

	int i = config_throttle();
	if (i < 0) Serial.print(F("auto adjust"));
	else if (i == 0) Serial.print(F("off"));
	else            Serial.print(i);
}


static void print_serial_device_sim(byte dev)
{
	switch (dev)
	{
	case CSM_SIO:   Serial.print(F("SIO")); break;
	case CSM_ACR:   Serial.print(F("ACR")); break;
	case CSM_2SIO1: Serial.print(F("2-SIO port 1")); break;
	case CSM_2SIO2: Serial.print(F("2-SIO port 2")); break;
	case CSM_2SIO3: Serial.print(F("2-SIO2 port 1")); break;
	case CSM_2SIO4: Serial.print(F("2-SIO2 port 2")); break;
	case 0xff: Serial.print(F("none")); break;
	}
}


static void print_host_primary_interface_aux(byte iface)
{
	Serial.print(host_serial_port_name(iface));
}


static void print_host_primary_interface(byte row = 0, byte col = 0)
{
	if (row != 0 || col != 0) set_cursor(row, col);
	print_host_primary_interface_aux(get_bits(new_config_serial_settings, 8, 3));

	if (get_bits(new_config_serial_settings, 8, 3) != config_host_serial_primary())
	{
		Serial.print(F(" (current: "));
		print_host_primary_interface_aux(config_host_serial_primary());
		Serial.print(')');
	}
}


static void print_serial_flag(uint32_t settings, byte pos, byte bits = 2)
{
	switch (get_bits(settings, pos, bits))
	{
	case CSF_OFF:  Serial.print(F("off"));        break;
	case CSF_ON:   Serial.print(F("on"));         break;
	case CSF_AUTO: Serial.print(F("autodetect")); break;
	}
}


static void print_serial_flag_backspace(uint32_t settings)
{
	switch (get_bits(settings, 14, 2))
	{
	case CSFB_NONE:       Serial.print(F("off"));            break;
	case CSFB_UNDERSCORE: Serial.print(F("underscore (_)")); break;
	case CSFB_DELETE:     Serial.print(F("delete (127)"));   break;
	case CSFB_AUTO:       Serial.print(F("autodetect"));     break;
	}
}


static void print_serial_flag_siorev(uint32_t settings)
{
	switch (get_bits(settings, 8, 2))
	{
	case 0: Serial.print(F("rev0"));    break;
	case 1: Serial.print(F("rev1"));    break;
	case 2: Serial.print(F("Cromemco")); break;
	}
}


static void print_device_mapped_to(byte s)
{
	if (s == 0)
		Serial.print(F("Not mapped"));
	else if (s == 7)
	{
		Serial.print(F("Primary ("));
		print_host_primary_interface();
		Serial.print(')');
	}
	else
		Serial.print(host_serial_port_name(s - 1));
}


static void print_serial_device_mapped_to(uint32_t settings)
{
	print_device_mapped_to(get_bits(settings, 17, 3));
}


static void print_dazzler_mapped_to()
{
	byte s = get_bits(config_flags2, 0, 3);

	if (s == 0)
		Serial.print(F("Disabled"));
	else
	{
		Serial.print(F("On "));
		Serial.print(host_serial_port_name(s - 1));
	}
}


static void print_vdm1_mapped_to()
{
	byte s = get_bits(config_flags2, 3, 3);

	if (s == 0)
		Serial.print(F("Disabled"));
	else
	{
		Serial.print(F("On "));
		Serial.print(host_serial_port_name(s - 1));
	}
}


static void print_vdm1_addr()
{
	uint16_t a = config_vdm1_address();
	numsys_print_word(a);
}


static void print_vdm1_dip12()
{
	switch (config_vdm1_dip() & 3)
	{
	case 0: Serial.print(F("off/off (no display)")); break;
	case 2: Serial.print(F("off/on  (normal video)")); break;
	case 1: Serial.print(F("on /off (inverse video)")); break;
	case 3: Serial.print(F("on /on  [ILLEGAL]")); break;
	}
}


static void print_vdm1_dip34()
{
	switch ((config_vdm1_dip() >> 2) & 3)
	{
	case 0: Serial.print(F("off/off (all cursors suppressed)")); break;
	case 2: Serial.print(F("off/on  (blinking cursor)")); break;
	case 1: Serial.print(F("on /off (non-blinking cursor)")); break;
	case 3: Serial.print(F("on /on  [ILLEGAL]")); break;
	}
}


static void print_vdm1_dip56()
{
	switch ((config_vdm1_dip() >> 4) & 3)
	{
	case 0: Serial.print(F("off/off (all characters suppressed, VT-CR blanking on)")); break;
	case 2: Serial.print(F("off/on  (control characters blanked, VT-CR blanking on)")); break;
	case 1: Serial.print(F("on /off (control characters shown, VT-CR blanking on)")); break;
	case 3: Serial.print(F("on /on  (control characters shown, VT-CR blanking off)")); break;
	}
}


static void print_printer_mapped_to()
{
	print_device_mapped_to(get_bits(config_flags, 21, 3));
}


static void print_rtc_frequency()
{
	float rate = config_rtc_rate();
	if (rate == 0.0)
		Serial.print(F("Disabled"));
	else
	{
		Serial.print(rate); Serial.print(F(" Hz"));
	}
}


static void print_interrupt_conn(uint32_t mask, byte b)
{
	if (config_flags & CF_HAVE_VI)
	{
		if (b == 0xff)
			Serial.print(F("Not connected"));
		else
		{
			Serial.print(F("VI")); Serial.print(b);
		}
	}
	else
	{
		if (config_interrupt_mask & mask)
			Serial.print(F("Connected"));
		else
			Serial.print(F("Not connected"));
	}
}


static void print_aux1_program(byte row = 0, byte col = 0)
{
	if (row != 0 || col != 0) set_cursor(row, col);

	byte b = config_aux1_program();

	if (b < 0x40)
		Serial.print(FP(prog_get_name(b)));
	else
	{
		const char* image;

		if (b < 0x80)
			image = tdrive_get_image_description(b & 0x3f);
		else if (b < 0xC0)
			image = drive_get_image_description(b & 0x3f);
		else
			image = hdsk_get_image_description(b & 0x3f);

		if (image == NULL)
			Serial.print(F("none"));
		else
			Serial.print(image);
	}
}


static void print_drive_mounted()
{
	byte n = 0;
	for (byte i = 0; i < NUM_DRIVES; i++)
		if (drive_get_mounted_image(i) > 0) n++;

	Serial.print(n); Serial.print(F(" mounted"));
}


static void print_cdrive_mounted()
{
	byte n = 0;
	for (byte i = 0; i < NUM_CDRIVES; i++)
		if (cdrive_get_mounted_image(i) > 0) n++;

	Serial.print(n); Serial.print(F(" mounted"));
}


static void print_tdrive_mounted()
{
	byte n = 0;
	for (byte i = 0; i < NUM_TDRIVES; i++)
		if (tdrive_get_mounted_image(i) > 0) n++;

	Serial.print(n); Serial.print(F(" mounted"));
}


static void print_drive_mounted_image(byte d)
{
	if (d == 0)
		Serial.print(F("none"));
	else if (drive_get_image_filename(d) == NULL)
	{
		Serial.print(F("empty disk #")); numsys_print_byte(d);
	}
	else
		Serial.print(drive_get_image_description(d));
}


static void print_cdrive_mounted_image(byte d)
{
	if (d == 0)
		Serial.print(F("none"));
	else if (cdrive_get_image_filename(d) == NULL)
	{
		Serial.print(F("empty disk #")); numsys_print_byte(d);
	}
	else
		Serial.print(cdrive_get_image_description(d));
}


static void print_tdrive_mounted_image(byte d)
{
	if (d == 0)
		Serial.print(F("none"));
	else if (tdrive_get_image_filename(d) == NULL)
	{
		Serial.print(F("empty disk #")); numsys_print_byte(d);
	}
	else
		Serial.print(tdrive_get_image_description(d));
}


static void print_hdsk_mounted()
{
	byte n = 0;
	for (byte i = 0; i < NUM_HDSK_UNITS; i++)
		for (byte j = 0; j < 4; j++)
			if (hdsk_get_mounted_image(i, j) > 0) n++;

	Serial.print(n); Serial.print(F(" mounted"));
}


static void print_hdsk_mounted_image(byte d)
{
	if (d == 0)
		Serial.print(F("none"));
	else if (hdsk_get_image_filename(d) == NULL)
	{
		Serial.print(F("empty disk #")); numsys_print_byte(d);
	}
	else
		Serial.print(hdsk_get_image_description(d));
}


static void print_printer_type()
{
	switch (config_printer_type())
	{
	case 0: Serial.print(F("None")); break;
	case 1: Serial.print(F("Okidata/88-LPC")); break;
	case 2: Serial.print(F("C700")); break;
	case 3: Serial.print(F("Generic")); break;
	}
}


static void print_parity(byte p)
{
	switch (p)
	{
	case 0: Serial.print(F("None")); break;
	case 1: Serial.print(F("Even")); break;
	case 2: Serial.print(F("Odd "));  break;
	default: Serial.print(F("??? "));  break;
	}
}



static void print_mapped_serial_cards()
{
	bool mapped = false;
	if (config_serial_map_sim_to_host(CSM_SIO) < 0xff) { Serial.print(F("SIO")); mapped = true; }
	if (config_serial_map_sim_to_host(CSM_ACR) < 0xff) { if (mapped) Serial.print(','); Serial.print(F("ACR")); mapped = true; }
	if (config_serial_map_sim_to_host(CSM_2SIO1) < 0xff) { if (mapped) Serial.print(','); Serial.print(F("2SIO-P1")); mapped = true; }
	if (config_serial_map_sim_to_host(CSM_2SIO2) < 0xff) { if (mapped) Serial.print(','); Serial.print(F("2SIO-P2")); mapped = true; }
	if (config_serial_map_sim_to_host(CSM_2SIO3) < 0xff) { if (mapped) Serial.print(','); Serial.print(F("2SIO2-P1")); mapped = true; }
	if (config_serial_map_sim_to_host(CSM_2SIO4) < 0xff) { if (mapped) Serial.print(','); Serial.print(F("2SIO2-P2")); mapped = true; }
	if (!mapped) Serial.print(F("None"));
}


// --------------------------------------------------------------------------------


static void apply_host_serial_settings(uint32_t settings, uint32_t settings2)
{
	config_serial_settings = settings;
	config_serial_settings2 = settings2;
	for (byte i = 0; i < HOST_NUM_SERIAL_PORTS; i++)
		host_serial_setup(i, config_host_serial_baud_rate(i),
			config_host_serial_config(settings2, i),
			config_host_serial_primary() == i);

	// mapping for serial devices can have changed if primary serial was changed
	for (byte dev = 0; dev < NUM_SERIAL_DEVICES; dev++)
		config_serial_sim_to_host[dev] = config_map_device_to_host_interface(get_bits(config_serial_device_settings[dev], 17, 3));
}


static byte toggle_host_serial_baud_rate(byte iface, byte n)
{
	uint32_t min, max;
	if (host_serial_port_baud_limits(iface, &min, &max))
	{
		do
		{
			n = (n + 1) % 16;
		} while (config_baud_rate(n) < min || config_baud_rate(n) > max);

#if defined(__SAM3X8E__)
		// connecting to Arduino Due at 1200 baud over USB will cause it to erase its flash memory
		// and go into programming mode => skip 1200 baud setting for USB interface
		if (iface == 0 && n == BAUD_1200) n++;
#endif
	}

	return n;
}


static void toggle_host_primary_interface(byte row, byte col)
{
	new_config_serial_settings = toggle_bits(new_config_serial_settings, 8, 3, 0, HOST_NUM_SERIAL_PORTS - 1);
	print_host_primary_interface(row, col);
}


static void toggle_rtc_rate()
{
	byte value = get_bits(config_flags, 8, 4);
	if ((value & 0x08) == 0)
		value = 0x08;
	else if (value == 0x0f)
		value = 0x00;
	else
		value++;

	config_flags = set_bits(config_flags, 8, 4, value);
}


static void toggle_throttle(byte row, byte col, bool up)
{
	int i = config_throttle();

	if (up)
	{
		if (++i > 31) i = -1;
	}
	else
	{
		if (--i < -1) i = 31;
	}

	config_flags = set_bits(config_flags, 0, 1, i != 0);
	config_flags = set_bits(config_flags, 12, 5, i < 0 ? 0 : i);

	set_cursor(row, col);
	print_throttle();
}


static byte toggle_interrupt_conn(uint32_t mask, byte c, byte row, byte col)
{
	if (config_flags & CF_HAVE_VI)
	{
		if (c == 0xff)
			c = 0;
		else if (c >= 7)
			c = 0xff;
		else
			c++;
	}
	else
		config_interrupt_mask = (config_interrupt_mask & ~mask) | (~config_interrupt_mask & mask);

	set_cursor(row, col);
	print_interrupt_conn(mask, c);
	return c;
}


static uint32_t toggle_map_to_serial(uint32_t settings, byte start_bit)
{
	byte i = get_bits(settings, start_bit, 3);
	if (i == HOST_NUM_SERIAL_PORTS)
		i = 0;
#if HOST_NUM_SERIAL_PORTS>1
	else if (i == 0)
		i = 7;
	else if (i == 7)
		i = 1;
#endif
	else
		i++;

	return set_bits(settings, start_bit, 3, i);
}


static uint32_t toggle_serial_flag(uint32_t settings, byte pos, bool haveAuto = true)
{
	byte b = get_bits(settings, pos, 2);
	switch (b)
	{
	case CSF_OFF:  b = CSF_ON;   break;
	case CSF_ON:   b = haveAuto ? CSF_AUTO : CSF_OFF; break;
	case CSF_AUTO: b = CSF_OFF;  break;
	default:       b = haveAuto ? CSF_AUTO : CSF_OFF; break;
	}

	return set_bits(settings, pos, 2, b);
}


static uint32_t toggle_serial_flag_backspace(uint32_t settings)
{
	byte b = get_bits(settings, 14, 2);
	switch (b)
	{
	case CSFB_NONE:       b = CSFB_UNDERSCORE; break;
	case CSFB_UNDERSCORE: b = CSFB_DELETE; break;
	case CSFB_DELETE:     b = CSFB_AUTO; break;
	case CSFB_AUTO:       b = CSFB_NONE; break;
	}

	return set_bits(settings, 14, 2, b);
}


static byte find_floppy_image(byte n, bool up, byte max = 0xff)
{
	int i = n;

	do
	{
		if (drive_get_image_filename(i) != NULL) return i;
		if (up) i++; else i--;
	} while (i >= 0 && i <= max);

	return 0;
}


static byte find_cfloppy_image(byte n, bool up, byte max = 0xff)
{
	int i = n;
	do
	{
		if (cdrive_get_image_filename(i) != NULL) return i;
		if (up) i++; else i--;
	} while (i >= 0 && i <= max);

	return 0;
}


static byte find_tfloppy_image(byte n, bool up, byte max = 0xff)
{
	int i = n;
	do
	{
		if (tdrive_get_image_filename(i) != NULL) return i;
		if (up) i++; else i--;
	} while (i >= 0 && i <= max);

	return 0;
}


static byte find_hdsk_image(byte n, bool up, byte max = 0xff)
{
	int i = n;
	do
	{
		if (hdsk_get_image_filename(i) != NULL) return i;
		if (up) i++; else i--;
	} while (i >= 0 && i <= max);

	return 0;
}


static void toggle_aux1_program_up(byte row, byte col)
{
	byte b = config_aux1_prog;
	bool found = false;

	while (!found)
	{
		b++;
		if (!found && (b & 0xC0) == 0x00)
		{
			// look for next integrated program
			if (b > 0 && prog_get_name(b))
				found = true;
			else if (b == 0x3F)
				b = 0x41;
		}

		if (!found && (b & 0xC0) == 0x40)
		{
#if NUM_TDRIVES>0
			// look for next tarbell floppy image
			b = find_tfloppy_image(b & 0x3f, true, 0x3f) | 0x40;
			if (b > 0x40)
				found = true;
			else
#endif
				b = 0x81;
		}

		if (!found && (b & 0xC0) == 0x80)
		{
#if NUM_DRIVES>0
			// look for next floppy disk image
			b = find_floppy_image(b & 0x3f, true, 0x3f) | 0x80;
			if (b > 0x80)
				found = true;
			else
#endif
				b = 0xC1;
		}

		if (!found && (b & 0xC0) == 0xC0)
		{
#if NUM_HDSK_UNITS>0
			// look for next hard disk image
			b = find_hdsk_image(b & 0x3f, true, 0x3f) | 0xC0;
			if (b > 0xC0)
				found = true;
			else
#endif
				b = 0x00;
		}
	}

	config_aux1_prog = b;
	print_aux1_program(row, col);
}


static void toggle_aux1_program_down(byte row, byte col)
{
	byte b = config_aux1_prog;
	bool found = false;

	while (!found)
	{
		b--;
		if (!found && (b & 0xC0) == 0xC0)
		{
			// look for previous hard disk image
#if NUM_HDSK_UNITS>0
			b = find_hdsk_image(b & 0x3f, false, 0x3f) | 0xC0;
			if (b > 0xC0)
				found = true;
			else
#endif
				b = 0xBF;
		}

		if (!found && (b & 0xC0) == 0x80)
		{
			// look for previous floppy disk image
#if NUM_DRIVES>0
			b = find_floppy_image(b & 0x3f, false, 0x3f) | 0x80;
			if (b > 0x80)
				found = true;
			else
#endif
				b = 0x7F;
		}

		if (!found && (b & 0xC0) == 0x40)
		{
			// look for previous tarbell floppy disk image
#if NUM_TDRIVES>0
			b = find_tfloppy_image(b & 0x3f, false, 0x3f) | 0x40;
			if (b > 0x40)
				found = true;
			else
#endif
				b = 0x3F;
		}

		if (!found && (b & 0xC0) == 0x00)
		{
			// look for previous integrated program
			if (b > 0 && prog_get_name(b))
				found = true;
		}
	}

	config_aux1_prog = b;
	print_aux1_program(row, col);
}

bool config_use_z80()
{
	return get_bits(config_flags2, 21, 1) ? true : false;
}

// B Mode
// returns True when 8800B emulation is enabled
bool config_b_mode()
{
	return get_bits(config_flags2, 23, 1) ? true : false;
}

void set_b_mode(uint32_t Enabled)
{
	set_bits(config_flags2, 23, 1, Enabled);
}

void toggle_b_mode()
{
	config_flags2 = toggle_bits(config_flags2, 23, 1);
}

void print_b_mode()
{
	if (config_b_mode())
		Serial.print(F("8800B"));
	else
		Serial.print(F("8800/8800A"));
}


static uint32_t toggle_flag(uint32_t flags, uint32_t value, byte row, byte col)
{
	flags = (flags & ~value) | (~(flags & value) & value);
	print_flag(flags, value, row, col);
	return flags;
}


static void toggle_flag(uint32_t value, byte row, byte col)
{
	config_flags = toggle_flag(config_flags, value, row, col);
}


static void toggle_flag2(uint32_t value, byte row, byte col)
{
	config_flags2 = toggle_flag(config_flags2, value, row, col);
}


static bool check_video_interface_settings()
{
	bool ok;
	uint32_t old_config_serial_settings = config_serial_settings;
	byte pi = config_printer_type() == 0 ? 0xff : config_printer_map_to_host_serial();
	byte di = config_dazzler_interface();
	byte vi = config_vdm1_interface();

#if USE_DAZZLER>0
	config_serial_settings = new_config_serial_settings;
	ok = (di != pi) && (di != vi);
	for (byte i = 0; ok && i < NUM_SERIAL_DEVICES; i++)
		ok = config_serial_map_sim_to_host(i) != di;
	config_serial_settings = old_config_serial_settings;

	if (di < 255 && !ok)
	{
		char c;
		Serial.print(F("\r\nDazzler can not use the same serial port as any other device.\nDisable Dazzler and continue (y/n)? "));
		do { delay(50); c = serial_read(); } while (c != 'y' && c != 'n');
		if (c == 'y')
			config_flags2 = set_bits(config_flags2, 0, 3, 0);
		else
			return false;
	}
#endif

#if USE_VDM1>0            
	config_serial_settings = new_config_serial_settings;
	ok = (vi != pi) && (vi != di);
	for (byte i = 0; ok && i < NUM_SERIAL_DEVICES; i++)
		ok = config_serial_map_sim_to_host(i) != vi;
	config_serial_settings = old_config_serial_settings;

	if (vi < 255 && !ok)
	{
		char c;
		Serial.print(F("\r\nVDM-1 can not use the same serial port as any other device.\nDisable VDM-1 and continue (y/n)? "));
		do { delay(50); c = serial_read(); } while (c != 'y' && c != 'n');
		if (c == 'y')
			config_flags2 = set_bits(config_flags2, 3, 3, 0);
		else
			return false;
	}
#endif

	return true;
}


static bool check_primary_interface_serial_change()
{
	byte newprimary = get_bits(new_config_serial_settings, 8, 3);

#if HOST_NUM_SERIAL_PORTS>1
	if (config_host_serial_primary() != newprimary)
		return true;
#endif

	if (config_host_serial_baud_rate(new_config_serial_settings, newprimary)
		!=
		config_host_serial_baud_rate(config_serial_settings, newprimary))
		return true;

	if (config_host_serial_config(new_config_serial_settings2, newprimary)
		!=
		config_host_serial_config(config_serial_settings2, newprimary))
		return true;

	return false;
}


static bool apply_host_serial_settings()
{
	char c;
	byte old_dazzler_interface = dazzler_get_iface();
	byte old_vdm1_interface = vdm1_get_iface();
	uint32_t old_config_serial_settings = config_serial_settings;
	uint32_t old_config_serial_settings2 = config_serial_settings2;

	// if the primary interface has changed or the baud rate or serial paramters
	// of the primary interface have changed then require a confirmation, otherwis
	// revert to the current settings. This prevents the user from locking themselves
	// out by choosing incorrect settings.
	bool must_confirm = check_primary_interface_serial_change();
	if (must_confirm)
	{
#if HOST_NUM_SERIAL_PORTS>1
		if (config_host_serial_primary() != get_bits(new_config_serial_settings, 8, 3))
			Serial.println(F("\r\nChange must be confirmed by answering 'y' through new primary interface.\r\n"
				"[will revert back to this interface if new interface is not confirmed within 30 seconds]"));
		else
#endif
			Serial.println(F("\r\n[will revert to old settings if new settings not confirmed within 30 seconds]\r\n"));
		Serial.flush();
	}

	dazzler_set_iface(config_dazzler_interface());
	vdm1_set_iface(config_vdm1_interface());
	apply_host_serial_settings(new_config_serial_settings, new_config_serial_settings2);

	if (must_confirm)
	{
		uint32_t timeout = millis() + 30000, timeout2 = 0;
		c = 0;

		do
		{
			if (millis() > timeout2)
			{
				Serial.print(F("\r\nKeep new host interface settings (y/n)? "));
				timeout2 = millis() + 2000;
			}

			delay(50);
			c = serial_read();
		} while (c != 'y' && c != 'n' && millis() < timeout);
		Serial.println(c);
		if (c != 'y')
		{
			dazzler_set_iface(old_dazzler_interface);
			vdm1_set_iface(old_vdm1_interface);
			apply_host_serial_settings(old_config_serial_settings, old_config_serial_settings2);
			if (c != 'n')
			{
				delay(100);
				Serial.println(F("\r\nSettings were not confirmed within 30 seconds."));
				delay(3000);

				// flush serial input that may have accumulated while waiting
				while (serial_read() >= 0);
			}
			return false;
		}
	}

	return true;
}


// --------------------------------------------------------------------------------


static bool save_config(byte fileno)
{
	bool res = false;
	byte s = sizeof(uint32_t);
	byte data[4 * 4 + (1 + NUM_SERIAL_DEVICES * 4) + 9 * 4 + 1 + (1 + NUM_DRIVES) + (2 + NUM_CDRIVES) + (1 + NUM_TDRIVES) + (1 + NUM_HDSK_UNITS * 4) + 1 + 1 + 2];

	// check for disabled, non-temporary ROMs that need to be restored before saving
	uint16_t flags;
	bool haveDisabledRoms = false;
	for (byte i = 0; i < mem_get_num_roms() && !haveDisabledRoms; i++)
		if (mem_get_rom_info(i, NULL, NULL, NULL, &flags))
			haveDisabledRoms = (flags & MEM_ROM_FLAG_DISABLED) != 0 && (flags & MEM_ROM_FLAG_TEMP) == 0;

#if MAX_NUM_ROMS>0
	// if there are such ROMS then we need to reset the machine (to re-enable the ROMs) before
	// we can save the configuration.
	if (haveDisabledRoms)
	{
		char c;
		Serial.print(F("Disabled ROMs will be restored before saving configuration. Continue? (y/n)? "));
		do { delay(50); c = serial_read(); } while (c != 'y' && c != 'n');
		Serial.println(c);
		if (c == 'y')
			mem_restore_roms();
		else
			return true;
	}
#endif

	// merge version number into config_flags
	config_flags = (config_flags & 0x00ffffff) | (((uint32_t)CONFIG_FILE_VERSION) << 24);

	word n = 0;
	memcpy(data + n, &config_flags, s); n += s;
	memcpy(data + n, &config_flags2, s); n += s;
	memcpy(data + n, &new_config_serial_settings, s); n += s;
	memcpy(data + n, &new_config_serial_settings2, s); n += s;
	data[n] = NUM_SERIAL_DEVICES; n++;
	memcpy(data + n, config_serial_device_settings, NUM_SERIAL_DEVICES * s); n += NUM_SERIAL_DEVICES * s;
	memcpy(data + n, config_interrupt_vi_mask, 8 * s); n += 8 * s;
	memcpy(data + n, &config_interrupt_mask, s); n += s;

	data[n++] = config_aux1_prog;

	data[n++] = NUM_DRIVES;
	for (byte i = 0; i < NUM_DRIVES; i++) data[n++] = drive_get_mounted_image(i);

	data[n++] = NUM_CDRIVES;
	for (byte i = 0; i < NUM_CDRIVES; i++) data[n++] = cdrive_get_mounted_image(i);
	data[n++] = cdrive_get_switches();

	data[n++] = NUM_TDRIVES;
	for (byte i = 0; i < NUM_TDRIVES; i++) data[n++] = tdrive_get_mounted_image(i);

	data[n++] = NUM_HDSK_UNITS;
	for (byte i = 0; i < NUM_HDSK_UNITS; i++)
		for (byte j = 0; j < 4; j++)
			data[n++] = hdsk_get_mounted_image(i, j);

	data[n++] = config_printer_generic_status_busy;
	data[n++] = config_printer_generic_status_ready;

	data[n++] = config_mem_size / 256;
	data[n++] = mem_get_num_roms(false);

	if (data[n - 1] == 0)
	{
		// better to write all data at once (will overwrite instead
		// of deleting/creating the file)
		res = filesys_write_file('C', fileno, (void*)data, n);
	}
#if MAX_NUM_ROMS>0
	else
	{
		// can't write all at once if we have to store ROM data
		byte fid = filesys_open_write('C', fileno);
		if (fid > 0)
		{
			res = filesys_write_data(fid, (void*)data, n);
			for (byte i = 0; i < mem_get_num_roms() && res; i++)
			{
				char name[9];
				uint16_t start, length;
				mem_get_rom_info(i, name, &start, &length, &flags);
				if (!(flags & MEM_ROM_FLAG_TEMP))
				{
					memcpy(data + 0, &start, 2);
					memcpy(data + 2, &length, 2);
					memcpy(data + 4, &flags, 2);
					memcpy(data + 6, name, 8);
					res &= filesys_write_data(fid, (void*)data, 6 + 8);
					mem_set_rom_filepos(i, filesys_getpos(fid));
					res &= filesys_write_data(fid, Mem + start, length);
				}
			}

			filesys_close(fid);
		}
	}
#endif

	return res;
}


static bool load_config(byte fileno)
{
	bool ok = false;
	byte i, j, n, d, fid = filesys_open_read('C', fileno);
	if (fid)
	{
		// initialize all settings with defaults so configuration settings
		// missing at the end of the file will just be at their defaults
		// (helpful when reading an old config file after adding new settings)
		config_defaults(false);

		byte s = sizeof(uint32_t);
		filesys_read_data(fid, &config_flags, s);

		// highest 8 bits are file version
		byte v = config_flags >> 24;

		if (v >= 5)
		{
			unsigned long vv;

			// config file before version 5 does not have config_flags2
			filesys_read_data(fid, &vv, s);

			// config file version 5 does not have VDM-1 settings
			if (v == 5)
				config_flags2 = (vv & 7) | (config_flags2 & ~7);
			else
				config_flags2 = vv;
		}

		filesys_read_data(fid, &new_config_serial_settings, s);

		if (v < 3)
		{
			// config file before version 3 does not include serial port configuration
			// => default to 8N1
			new_config_serial_settings2 = 0;
			for (i = 0; i < HOST_NUM_SERIAL_PORTS; i++)
				new_config_serial_settings2 |= 0x18 << (i * 5);
		}
		else
			filesys_read_data(fid, &new_config_serial_settings2, s);

		n = 4;
		// in config file version 2, the number of serial devices is in the file, 
		// before it was fixed at 4
		if (v >= 2) filesys_read_data(fid, &n, 1);
		filesys_read_data(fid, config_serial_device_settings, min(NUM_SERIAL_DEVICES, n) * s);

		// skip extra serial device data in config
		if (NUM_SERIAL_DEVICES < n)
			for (i = NUM_SERIAL_DEVICES * s; i < n * s; i++)
				filesys_read_data(fid, &d, 1);

		if (v < 4)
		{
			// version 4 introduces SIO revision, rev1 should be the default
			for (i = 0; i < NUM_SERIAL_DEVICES; i++)
				config_serial_device_settings[i] = (config_serial_device_settings[i] & ~0x300) | 0x100;
		}

		if (v == 0)
		{
			// in config file version 0 the interrupt masks are 8 bits
			byte b[9];
			filesys_read_data(fid, b, 9);
			for (i = 0; i < 8; i++) config_interrupt_vi_mask[i] = b[i];
			config_interrupt_mask = b[8];
		}
		else
		{
			// in config file version 1 and later the interrupt masks are 32 bits
			filesys_read_data(fid, config_interrupt_vi_mask, 8 * s);
			filesys_read_data(fid, &config_interrupt_mask, s);
		}

		// AUX1 UP shortcut program
		filesys_read_data(fid, &config_aux1_prog, 1);

		// disk drive settings
		if (filesys_read_data(fid, &n, 1) != 1) n = 0;
		for (i = 0; i < n; i++)
			if (filesys_read_data(fid, &d, 1) == 1 && i < NUM_DRIVES)
			{
				if (d > 0)
					drive_mount(i, d);
				else
					drive_unmount(i);
			}
		drive_set_realtime((config_flags & CF_DRIVE_RT) != 0);

		if (v >= 10)
		{
			// cromemco disk drive settings (version 10 and up)
			if (filesys_read_data(fid, &n, 1) != 1) n = 0;
			for (i = 0; i < n; i++)
				if (filesys_read_data(fid, &d, 1) == 1 && i < NUM_CDRIVES)
				{
					if (d > 0)
						cdrive_mount(i, d);
					else
						cdrive_unmount(i);
				}

			if (filesys_read_data(fid, &d, 1) == 1)
				cdrive_set_switches(d);
		}

		if (v >= 8)
		{
			// tarbell disk drive settings (version 8 and up)
			if (filesys_read_data(fid, &n, 1) != 1) n = 0;
			for (i = 0; i < n; i++)
				if (filesys_read_data(fid, &d, 1) == 1 && i < NUM_TDRIVES)
				{
					if (d > 0)
						tdrive_mount(i, d);
					else
						tdrive_unmount(i);
				}
		}

		// hard disk settings
		if (filesys_read_data(fid, &n, 1) != 1) n = 0;
		for (i = 0; i < n; i++)
			for (j = 0; j < 4; j++)
				if (filesys_read_data(fid, &d, 1) == 1 && i < NUM_HDSK_UNITS)
				{
					if (d > 0)
						hdsk_mount(i, j, d);
					else
						hdsk_unmount(i, j);
				}
		hdsk_set_realtime((config_flags & CF_HDSK_RT) != 0);

		if (v >= 9)
		{
			// generic printer settings (version 9 and up)
			filesys_read_data(fid, &config_printer_generic_status_busy, 1);
			filesys_read_data(fid, &config_printer_generic_status_ready, 1);
		}

		// memory (RAM) size
		if (filesys_read_data(fid, &d, 1))
			config_mem_size = (d == 0) ? 0x10000 : (d * 256);
		else
			config_mem_size = MEMSIZE;

		// ROMs
		if (filesys_read_data(fid, &n, 1) != 1) n = 0;
		mem_clear_roms();
		for (i = 0; i < n; i++)
		{
			char name[9];
			uint16_t start, length, flags;
			name[8] = 0;
			if (filesys_read_data(fid, &start, 2) == 2)
				if (filesys_read_data(fid, &length, 2) == 2)
					if (filesys_read_data(fid, &flags, 2) == 2)
						if (filesys_read_data(fid, name, 8) == 8)
						{
							if (mem_add_rom(start, length, name, flags, filesys_getpos(fid)))
								filesys_read_data(fid, Mem + start, length);
							else
								for (uint16_t j = 0; j < length; j++)
									filesys_read_data(fid, name, 1);
						}
		}

		filesys_close(fid);

#if defined(__AVR_ATmega2560__)
		if (v == 0)
		{
			// on MEGA, early versions accidentally set bits 16-31 on serial settings
			for (byte dev = 0; dev < 4; dev++)
				if ((config_serial_device_settings[dev] & 0xFFFF0000) == 0xFFFF0000)
					config_serial_device_settings[dev] &= 0x0000FFFF;
		}
#endif

		if (v < 2)
		{
			// in version 2, serial device to host serial mapping changed:
			// previous: bits  8- 9 (00=NONE, 01=primary, 02=secondary)
			// now:      bits 17-19 (000=NONE, 001=first, 010=second, 011=third, 100=fourth, 101=fifth, 111=primary)
			for (i = 0; i < 4; i++)
			{
				byte map = get_bits(config_serial_device_settings[i], 8, 2);
				config_serial_device_settings[i] = set_bits(config_serial_device_settings[i], 8, 2, 0);
				config_serial_device_settings[i] = set_bits(config_serial_device_settings[i], 17, 3, map == 1 && HOST_NUM_SERIAL_PORTS > 1 ? 7 : map);
			}

			// in version 2, printer to host serial mapping changed:
			// previous: bits 17-18 (realtime mode flags bits 21-22)
			// now     : bits 21-23 (realtime mode flags bits 17-18)
			byte map = get_bits(config_flags, 17, 2);
			byte flags = get_bits(config_flags, 21, 2);
			config_flags = set_bits(config_flags, 17, 2, flags);
			config_flags = set_bits(config_flags, 21, 3, map == 1 ? 7 : map);

			// version 2 introduces more host serial interfaces: set them all to 9600 baud
			for (i = 2; i < HOST_NUM_SERIAL_PORTS; i++) new_config_serial_settings |= (BAUD_9600 << (config_baud_rate_bits(i)));
		}
		else
		{
			// make sure nothing is mapped to an illegal host serial port
			for (i = 0; i < NUM_SERIAL_DEVICES; i++)
				if (config_map_device_to_host_interface(get_bits(config_serial_device_settings[i], 17, 3)) >= HOST_NUM_SERIAL_PORTS)
					config_serial_device_settings[i] = set_bits(config_serial_device_settings[i], 17, 3, 0);
		}

		for (byte dev = 0; dev < NUM_SERIAL_DEVICES; dev++)
			config_serial_sim_to_host[dev] = config_map_device_to_host_interface(get_bits(config_serial_device_settings[dev], 17, 3));

#if STANDALONE>0
		config_flags |= CF_SERIAL_INPUT;
#endif
		ok = true;
		config_current = fileno;
	}

	// note: registered ports for drives are controlled by mount/unmount functions
	serial_register_ports();
	printer_register_ports();
	altair_vi_register_ports();
	dazzler_register_ports();
	vdm1_register_ports();

	return ok;
}


// --------------------------------------------------------------------------------

#if USE_PRINTER>0

void config_edit_printer()
{
	byte row, col, r_type, r_iface, r_force, r_cmd;
	bool go = true, redraw = true;

	byte prev_type = config_printer_type();
	byte prev_mapping = config_printer_map_to_host_serial();
	while (go)
	{
		if (redraw)
		{
			Serial.print(F("\033[2J\033[0;0H\n"));

			row = 4;
			col = 33;
			Serial.println(F("Configure printer settings"));
			Serial.print(F("\n(P)rinter type                : ")); r_type = row++; print_printer_type(); Serial.println();
			Serial.print(F("Map printer to (i)nterface    : ")); r_iface = row++; print_printer_mapped_to(); Serial.println();
			Serial.print(F("(F)orce real-time mode        : ")); r_force = row++; print_flag(CF_PRINTER_RT); Serial.println();
			if (config_printer_type() == CP_GENERIC)
			{
				Serial.print(F("Status register (b)usy value  : ")); numsys_print_byte_hex(config_printer_generic_get_status(true)); Serial.println('h');
				Serial.print(F("Status register (r)eady value : ")); numsys_print_byte_hex(config_printer_generic_get_status(false)); Serial.println('h');
				row += 2;
			}

			Serial.println(F("\nE(x)it to main menu"));
			Serial.print(F("\n\nCommand: ")); r_cmd = row + 4;
			redraw = false;
		}
		else
			set_cursor(r_cmd, 10);

		while (!serial_available()) delay(50);
		char c = serial_read();
		if (c > 31 && c < 127) Serial.println(c);

		switch (c)
		{
		case 'P':
			if (config_printer_type() == CP_GENERIC) redraw = true;
			config_flags = toggle_bits(config_flags, 19, 2, 0, 3);
			if (config_printer_type() == CP_GENERIC) redraw = true;
			set_cursor(r_type, col);
			print_printer_type();
			break;

		case 'i':
			config_flags = toggle_map_to_serial(config_flags, 21);
			set_cursor(r_iface, col);
			print_printer_mapped_to();
			break;

		case 'b':
		case 'r':
			if (config_printer_type() == CP_GENERIC)
			{
				byte b;
				Serial.print(F("\r\nEnter status byte: "));
				if (numsys_read_byte(&b))
				{
					if (c == 'b')
						config_printer_generic_status_busy = b;
					else
						config_printer_generic_status_ready = b;
				}
				redraw = true;
			}
			break;

		case 'F': toggle_flag(CF_PRINTER_RT, r_force, col); break;
		case 27:
		case 'x':
			go = false;
			if (config_printer_type() != prev_type || config_printer_map_to_host_serial() != prev_mapping)
				printer_setup();

			break;
		}
	}
}

#endif

// --------------------------------------------------------------------------------

#if NUM_DRIVES>0
void config_edit_drives()
{
	bool go = true;
	byte i, mounted[NUM_DRIVES];

	for (i = 0; i < NUM_DRIVES; i++) mounted[i] = drive_get_mounted_image(i);

	byte row, col, r_realtime, r_drives[NUM_DRIVES], r_cmd;
	row = 4;
	col = 32;
	Serial.print(F("\033[2J\033[0;0H\n"));

	Serial.println(F("Configure disk drive settings"));
	Serial.print(F("\n(F)orce real-time mode : ")); print_flag(CF_DRIVE_RT); Serial.println(); r_realtime = row++;

	for (i = 0; i < NUM_DRIVES; i++)
	{
		Serial.print(F("Drive ("));
		if (i < 10) Serial.write(48 + i); else Serial.write(87 + i);
		Serial.print(F(") mounted disk image : "));
		print_drive_mounted_image(mounted[i]);
		Serial.println();
		r_drives[i] = row++;
	}

	Serial.println(F("\nE(x)it to main menu")); row += 4;
	Serial.print(F("\n\nCommand: ")); r_cmd = row++;

	while (go)
	{
		set_cursor(r_cmd, 10);
		while (!serial_available()) delay(50);
		char c = serial_read();
		if (c > 31 && c < 127) Serial.println(c);

		switch (c)
		{
		case 'F': toggle_flag(CF_DRIVE_RT, r_realtime, 26); break;
		case 27:
		case 'x': go = false; break;

		default:
		{
			int d = -1;
			if (c >= '0' && c <= '9')
				d = c - 48;
			else if (c >= 'a' && c <= 'f')
				d = c - 87;

			if (d >= 0 && d <= NUM_DRIVES)
			{
				byte next = find_floppy_image(mounted[d] + 1, true);
				if (drive_get_mounted_image(d) > 0 &&
					drive_get_image_filename(drive_get_mounted_image(d)) == NULL &&
					mounted[d] < drive_get_mounted_image(d) &&
					(next > drive_get_mounted_image(d) || next < mounted[d]))
					next = drive_get_mounted_image(d);

				mounted[d] = next;
				set_cursor(r_drives[d], col);
				print_drive_mounted_image(mounted[d]);
			}
		}
		}
	}

	drive_set_realtime((config_flags & CF_DRIVE_RT) != 0);
	for (i = 0; i < NUM_DRIVES; i++)
		if (mounted[i] != drive_get_mounted_image(i))
			drive_mount(i, mounted[i]);
}
#endif


// --------------------------------------------------------------------------------


#if NUM_CDRIVES>0
void config_edit_cdrives()
{
	bool go = true;
	byte i, switches = cdrive_get_switches(), mounted[NUM_CDRIVES];

	for (i = 0; i < NUM_CDRIVES; i++) mounted[i] = cdrive_get_mounted_image(i);

	byte row, col, r_drives[NUM_CDRIVES], r_cmd, r_rom, r_romdis, r_auto, r_inhfmt;
	row = 4;
	col = 33;
	Serial.print(F("\033[2J\033[0;0H\n"));

	Serial.println(F("Configure Cromemco disk drive settings\n"));
	Serial.print(F("Enable boot (R)OM             : ")); print_flag(switches, CDRIVE_SWITCH_ROM_ENABLE, 0, 0); Serial.println(); r_rom = row++;
	Serial.print(F("Disable boot ROM after (b)oot : ")); print_flag(switches, CDRIVE_SWITCH_ROM_DISABLE_AFTER_BOOT, 0, 0); Serial.println(); r_romdis = row++;
	Serial.print(F("Enable (a)uto-boot            : ")); print_flag(switches, CDRIVE_SWITCH_AUTOBOOT, 0, 0); Serial.println(); r_auto = row++;
	Serial.print(F("Inhibit disk (f)ormatting     : ")); print_flag(switches, CDRIVE_SWITCH_INHIBIT_INIT, 0, 0); Serial.println(); r_inhfmt = row++;

	row++;
	Serial.print(F("\n"));
	for (i = 0; i < NUM_CDRIVES; i++)
	{
		Serial.print(F("Drive ("));
		if (i < 10) Serial.write(48 + i); else Serial.write(87 + i);
		Serial.print(F(") mounted disk image  : "));
		print_cdrive_mounted_image(mounted[i]);
		Serial.println();
		r_drives[i] = row++;
	}

	Serial.println(F("\nE(x)it to main menu")); row += 4;
	Serial.print(F("\n\nCommand: ")); r_cmd = row++;

	while (go)
	{
		set_cursor(r_cmd, 10);
		while (!serial_available()) delay(50);
		char c = serial_read();
		if (c > 31 && c < 127) Serial.println(c);

		switch (c)
		{
		case 27:
		case 'x': go = false; break;

		case 'R':
			switches = toggle_flag(switches, CDRIVE_SWITCH_ROM_ENABLE, r_rom, col);
			break;

		case 'b':
			switches = toggle_flag(switches, CDRIVE_SWITCH_ROM_DISABLE_AFTER_BOOT, r_romdis, col);
			break;

		case 'a':
			switches = toggle_flag(switches, CDRIVE_SWITCH_AUTOBOOT, r_auto, col);
			break;

		case 'f':
			switches = toggle_flag(switches, CDRIVE_SWITCH_INHIBIT_INIT, r_inhfmt, col);
			break;

		default:
		{
			int d = -1;
			if (c >= '0' && c <= '9')
				d = c - 48;
			else if (c >= 'a' && c <= 'f')
				d = c - 87;

			if (d >= 0 && d <= NUM_CDRIVES)
			{
				byte next = find_cfloppy_image(mounted[d] + 1, true);
				if (cdrive_get_mounted_image(d) > 0 &&
					cdrive_get_image_filename(drive_get_mounted_image(d)) == NULL &&
					mounted[d] < cdrive_get_mounted_image(d) &&
					(next > cdrive_get_mounted_image(d) || next < mounted[d]))
					next = cdrive_get_mounted_image(d);

				mounted[d] = next;
				set_cursor(r_drives[d], col);
				print_cdrive_mounted_image(mounted[d]);
			}
		}
		}
	}

	cdrive_set_switches(switches);
	for (i = 0; i < NUM_CDRIVES; i++)
		if (mounted[i] != cdrive_get_mounted_image(i))
			cdrive_mount(i, mounted[i]);
}
#endif


// --------------------------------------------------------------------------------


#if NUM_TDRIVES>0
void config_edit_tdrives()
{
	bool go = true;
	byte i, mounted[NUM_TDRIVES];

	for (i = 0; i < NUM_TDRIVES; i++) mounted[i] = tdrive_get_mounted_image(i);

	byte row, col, r_drives[NUM_TDRIVES], r_cmd;
	row = 4;
	col = 32;
	Serial.print(F("\033[2J\033[0;0H\n"));

	Serial.println(F("Configure Tarbell disk drive settings"));

	//Serial.print(F("\n(F)orce real-time mode : ")); print_flag(CF_DRIVE_RT); Serial.println(); r_realtime = row++;
	Serial.print(F("\n"));
	for (i = 0; i < NUM_TDRIVES; i++)
	{
		Serial.print(F("Drive ("));
		if (i < 10) Serial.write(48 + i); else Serial.write(87 + i);
		Serial.print(F(") mounted disk image : "));
		print_tdrive_mounted_image(mounted[i]);
		Serial.println();
		r_drives[i] = row++;
	}

	Serial.println(F("\nE(x)it to main menu")); row += 4;
	Serial.print(F("\n\nCommand: ")); r_cmd = row++;

	while (go)
	{
		set_cursor(r_cmd, 10);
		while (!serial_available()) delay(50);
		char c = serial_read();
		if (c > 31 && c < 127) Serial.println(c);

		switch (c)
		{
			//case 'F': toggle_flag(CF_DRIVE_RT, r_realtime, 26); break;
		case 27:
		case 'x': go = false; break;

		default:
		{
			int d = -1;
			if (c >= '0' && c <= '9')
				d = c - 48;
			else if (c >= 'a' && c <= 'f')
				d = c - 87;

			if (d >= 0 && d <= NUM_TDRIVES)
			{
				byte next = find_tfloppy_image(mounted[d] + 1, true);
				if (tdrive_get_mounted_image(d) > 0 &&
					tdrive_get_image_filename(drive_get_mounted_image(d)) == NULL &&
					mounted[d] < tdrive_get_mounted_image(d) &&
					(next > tdrive_get_mounted_image(d) || next < mounted[d]))
					next = tdrive_get_mounted_image(d);

				mounted[d] = next;
				set_cursor(r_drives[d], col);
				print_tdrive_mounted_image(mounted[d]);
			}
		}
		}
	}

	//drive_set_realtime((config_flags & CF_DRIVE_RT)!=0);
	for (i = 0; i < NUM_TDRIVES; i++)
		if (mounted[i] != drive_get_mounted_image(i))
			tdrive_mount(i, mounted[i]);
}
#endif


// --------------------------------------------------------------------------------

#if NUM_HDSK_UNITS>0
void config_edit_hdsk()
{
	bool go = true;
	byte i, j;

	byte mounted[NUM_HDSK_UNITS * 4];
	for (i = 0; i < NUM_HDSK_UNITS; i++)
		for (j = 0; j < 4; j++)
			mounted[i * 4 + j] = hdsk_get_mounted_image(i, j);

	byte row, col, r_drives[NUM_HDSK_UNITS * 4], r_cmd, r_realtime;
	row = 4;
	col = 33;
	Serial.print(F("\033[2J\033[0;0H\n"));

	Serial.println(F("Configure hard disk settings"));
	Serial.print(F("\n(F)orce real-time mode : ")); print_flag(CF_HDSK_RT); Serial.println(); r_realtime = row++;

	for (i = 0; i < NUM_HDSK_UNITS; i++)
		for (j = 0; j < 4; j++)
		{
			byte n = i * 4 + j;
			Serial.print(F("("));
			if (n < 10) Serial.write(48 + n); else Serial.write(87 + n);
			Serial.print(F(") Hard disk"));
#if NUM_HDSK_UNITS>1
			Serial.print(F(" unit "));
			Serial.print(i + 1);
			col = 40;
#endif
			Serial.print(F(" platter "));
			Serial.print(j);
			Serial.print(F(" image : "));
			print_hdsk_mounted_image(mounted[i * 4 + j]);
			Serial.println();
			r_drives[n] = row++;
		}

	Serial.println(F("\n(R)eset hard disk controller")); row += 3;

	Serial.println(F("\nE(x)it to main menu")); row += 3;
	Serial.print(F("\n\nCommand: ")); r_cmd = row++;

	while (go)
	{
		set_cursor(r_cmd, 10);
		while (!serial_available()) delay(50);
		char c = serial_read();
		if (c > 31 && c < 127) Serial.println(c);

		switch (c)
		{
		case 'F':
			toggle_flag(CF_HDSK_RT, r_realtime, 26);
			break;

		case 'R':
			Serial.print(F("\n\nResetting hard disk controller..."));
			hdsk_reset();
			delay(1000);
			Serial.print(F("\033[33D\033[K"));
			break;

		case 27:
		case 'x': go = false; break;

		default:
		{
			int d = -1;
			if (c >= '0' && c <= '9')
				d = c - 48;
			else if (c >= 'a' && c <= 'f')
				d = c - 87;

			if (d >= 0 && d < 4 * NUM_HDSK_UNITS)
			{
				byte u = d / 4, p = d % 4;
				byte next = find_hdsk_image(mounted[d] + 1, true);
				if (hdsk_get_mounted_image(u, p) > 0 &&
					hdsk_get_image_filename(hdsk_get_mounted_image(u, p)) == NULL &&
					mounted[d] < hdsk_get_mounted_image(u, p) &&
					(next > hdsk_get_mounted_image(u, p) || next < mounted[d]))
					next = hdsk_get_mounted_image(u, p);

				mounted[d] = next;
				set_cursor(r_drives[d], col);
				print_hdsk_mounted_image(mounted[d]);
			}
		}
		}
	}

	hdsk_set_realtime((config_flags & CF_HDSK_RT) != 0);
	for (i = 0; i < NUM_HDSK_UNITS; i++)
		for (j = 0; j < 4; j++)
			if (mounted[i * 4 + j] != hdsk_get_mounted_image(i, j))
				hdsk_mount(i, j, mounted[i * 4 + j]);
}
#endif


// --------------------------------------------------------------------------------


byte find_vi_conn(uint32_t interrupt)
{
	for (int i = 0; i < 8; i++)
		if (config_interrupt_vi_mask[i] & interrupt)
			return i;

	return 0xff;
}


void config_edit_interrupts()
{
	byte row, col, r_rtcf, r_vint, r_sio, r_acr, r_2sio1, r_2sio2, r_2sio3, r_2sio4, r_rtc, r_lpc, r_drive, r_hdsk, r_cmd;
	byte conn_sio = find_vi_conn(INT_SIO);
	byte conn_acr = find_vi_conn(INT_ACR);
	byte conn_2sio1 = find_vi_conn(INT_2SIO1);
	byte conn_2sio2 = find_vi_conn(INT_2SIO2);
	byte conn_2sio3 = find_vi_conn(INT_2SIO3);
	byte conn_2sio4 = find_vi_conn(INT_2SIO4);
	byte conn_rtc = find_vi_conn(INT_RTC);
	byte conn_lpc = find_vi_conn(INT_LPC);
	byte conn_drive = find_vi_conn(INT_DRIVE);
	byte conn_hdsk = find_vi_conn(INT_HDSK);

	bool go = true, redraw = true;
	while (go)
	{
		if (redraw)
		{
			row = 4;
			col = 34;
			Serial.print(F("\033[2J\033[0;0H\n"));
			Serial.println(F("Configure interrupt settings"));
			Serial.print(F("\n(R)eal-Time Clock              : ")); r_rtcf = row++; print_rtc_frequency(); Serial.println();
			Serial.print(F("(V)ector Interrupt board       : ")); r_vint = row++; print_vi_flag(); Serial.println();

			Serial.println(); row++;
			Serial.print(F("(0) Real-Time Clock interrupt  : ")); r_rtc = row++; print_interrupt_conn(INT_RTC, conn_rtc); Serial.println();
			Serial.print(F("(1) 88-SIO interrupt           : ")); r_sio = row++; print_interrupt_conn(INT_SIO, conn_sio); Serial.println();
			Serial.print(F("(2) 88-ACR interrupt           : ")); r_acr = row++; print_interrupt_conn(INT_ACR, conn_acr); Serial.println();
			Serial.print(F("(3) 88-LPC interrupt           : ")); r_lpc = row++; print_interrupt_conn(INT_LPC, conn_lpc); Serial.println();
			Serial.print(F("(4) 88-2SIO port 1 interrupt   : ")); r_2sio1 = row++; print_interrupt_conn(INT_2SIO1, conn_2sio1); Serial.println();
			Serial.print(F("(5) 88-2SIO port 2 interrupt   : ")); r_2sio2 = row++; print_interrupt_conn(INT_2SIO2, conn_2sio2); Serial.println();
#if USE_SECOND_2SIO>0
			Serial.print(F("(6) 88-2SIO-2 port 1 interrupt : ")); r_2sio3 = row++; print_interrupt_conn(INT_2SIO3, conn_2sio3); Serial.println();
			Serial.print(F("(7) 88-2SIO-2 port 2 interrupt : ")); r_2sio4 = row++; print_interrupt_conn(INT_2SIO4, conn_2sio4); Serial.println();
#endif
#if NUM_DRIVES>0
			Serial.print(F("(8) Disk drive interrupt       : ")); r_drive = row++; print_interrupt_conn(INT_DRIVE, conn_drive); Serial.println();
#endif
#if NUM_HDSK_UNITS>0
			Serial.print(F("(9) 88-HDSK interrupt          : ")); r_hdsk = row++; print_interrupt_conn(INT_HDSK, conn_hdsk); Serial.println();
#endif

			Serial.println(F("\nE(x)it to main menu"));

			Serial.print(F("\n\nCommand: ")); r_cmd = row + 4;
			redraw = false;
		}
		else
			set_cursor(r_cmd, 10);

		while (!serial_available()) delay(50);
		char c = serial_read();
		if (c > 31 && c < 127) Serial.println(c);

		switch (c)
		{
		case 'R':
		case 'r':
		case 'C':
		case 'c':
		{
			toggle_rtc_rate();
			set_cursor(r_rtcf, col);
			print_rtc_frequency();
			break;
		}

		case 'V':
		case 'v': toggle_flag(CF_HAVE_VI, 0, 0); redraw = true; break;

		case '0': conn_rtc = toggle_interrupt_conn(INT_RTC, conn_rtc, r_rtc, col);     break;
		case '1': conn_sio = toggle_interrupt_conn(INT_SIO, conn_sio, r_sio, col);     break;
		case '2': conn_acr = toggle_interrupt_conn(INT_ACR, conn_acr, r_acr, col);     break;
		case '3': conn_lpc = toggle_interrupt_conn(INT_LPC, conn_lpc, r_lpc, col);     break;
		case '4': conn_2sio1 = toggle_interrupt_conn(INT_2SIO1, conn_2sio1, r_2sio1, col); break;
		case '5': conn_2sio2 = toggle_interrupt_conn(INT_2SIO2, conn_2sio2, r_2sio2, col); break;
#if USE_SECOND_2SIO>0
		case '6': conn_2sio3 = toggle_interrupt_conn(INT_2SIO3, conn_2sio3, r_2sio3, col); break;
		case '7': conn_2sio4 = toggle_interrupt_conn(INT_2SIO4, conn_2sio4, r_2sio4, col); break;
#endif
#if NUM_DRIVES>0
		case '8': conn_drive = toggle_interrupt_conn(INT_DRIVE, conn_drive, r_drive, col); break;
#endif
#if NUM_HDSK_UNITS>0
		case '9': conn_hdsk = toggle_interrupt_conn(INT_HDSK, conn_hdsk, r_hdsk, col);   break;
#endif
		case 27:
		case 'x': go = false; break;
		}
	}

	for (int i = 0; i < 8; i++)  config_interrupt_vi_mask[i] = 0;
	if (conn_sio < 0xff) config_interrupt_vi_mask[conn_sio] |= INT_SIO;
	if (conn_acr < 0xff) config_interrupt_vi_mask[conn_acr] |= INT_ACR;
	if (conn_2sio1 < 0xff) config_interrupt_vi_mask[conn_2sio1] |= INT_2SIO1;
	if (conn_2sio2 < 0xff) config_interrupt_vi_mask[conn_2sio1] |= INT_2SIO2;
	if (conn_2sio3 < 0xff) config_interrupt_vi_mask[conn_2sio3] |= INT_2SIO3;
	if (conn_2sio4 < 0xff) config_interrupt_vi_mask[conn_2sio4] |= INT_2SIO4;
	if (conn_rtc < 0xff) config_interrupt_vi_mask[conn_rtc] |= INT_RTC;
	if (conn_drive < 0xff) config_interrupt_vi_mask[conn_drive] |= INT_DRIVE;
	if (conn_lpc < 0xff) config_interrupt_vi_mask[conn_lpc] |= INT_LPC;
	if (conn_hdsk < 0xff) config_interrupt_vi_mask[conn_hdsk] |= INT_HDSK;
	altair_vi_register_ports();
}


// --------------------------------------------------------------------------------

#if USE_VDM1>0
void config_edit_vdm1()
{
	byte row, col, r_iface, r_dip12, r_dip34, r_dip56, r_addr, r_cmd, r_kbd;
	bool go = true;

	// if a VDM-1 client is connected then this will initialize it
	// in case it is not already initialized
	vdm1_set_dip(config_vdm1_dip());

	row = 4;
	col = 22;
	Serial.print(F("\033[2J\033[0;0H\n"));

	Serial.println(F("Configure VDM-1 settings"));
	Serial.print(F("\nMap to (i)nterface : ")); r_iface = row++; print_vdm1_mapped_to(); Serial.println();
	Serial.print(F("Memory (a)ddress   : ")); r_addr = row++; print_vdm1_addr(); Serial.println();
	Serial.print(F("DIP switch (1)+2   : ")); r_dip12 = row++; print_vdm1_dip12(); Serial.println();
	Serial.print(F("DIP switch (3)+4   : ")); r_dip34 = row++; print_vdm1_dip34(); Serial.println();
	Serial.print(F("DIP switch (5)+6   : ")); r_dip56 = row++; print_vdm1_dip56(); Serial.println();
	Serial.print(F("Map (k)eyboard to  : ")); r_kbd = row++; print_serial_device_sim(config_vdm1_keyboard_device()); Serial.println();
	Serial.println(F("\nE(x)it to main menu")); row += 2;
	Serial.print(F("\n\nCommand: ")); r_cmd = row + 2;

	while (go)
	{
		set_cursor(r_cmd, 10);
		while (!serial_available()) delay(50);
		char c = serial_read();
		if (c > 31 && c < 127) Serial.println(c);

		switch (c)
		{
		case 'i':
			config_flags2 = toggle_bits(config_flags2, 3, 3, 0, HOST_NUM_SERIAL_PORTS);
			set_cursor(r_iface, col);
			print_vdm1_mapped_to();
			break;

		case '1':
		case '2':
			config_flags2 = toggle_vdm1_dip(config_flags2, 6, false);
			set_cursor(r_dip12, col);
			print_vdm1_dip12();
			vdm1_set_dip(config_vdm1_dip());
			break;

		case '3':
		case '4':
			config_flags2 = toggle_vdm1_dip(config_flags2, 8, false);
			set_cursor(r_dip34, col);
			print_vdm1_dip34();
			vdm1_set_dip(config_vdm1_dip());
			break;

		case '5':
		case '6':
			config_flags2 = toggle_vdm1_dip(config_flags2, 10, true);
			set_cursor(r_dip56, col);
			print_vdm1_dip56();
			vdm1_set_dip(config_vdm1_dip());
			break;

		case 'a':
			config_flags2 = toggle_bits(config_flags2, 12, 6);
			set_cursor(r_addr, col);
			print_vdm1_addr();
			vdm1_set_address(config_vdm1_address());
			break;

		case 'k':
		{
			byte dev = config_vdm1_keyboard_device();
			if (dev == 0xff)
				dev = 0;
			else if (dev + 1 < NUM_SERIAL_DEVICES)
				dev++;
			else
				dev = 0xff;

			config_flags2 = set_bits(config_flags2, 18, 3, dev + 1);
			set_cursor(r_kbd, col);
			print_serial_device_sim(config_vdm1_keyboard_device());
			break;
		}

		case 27:
		case 'x': go = false; break;
		}
	}

	vdm1_register_ports();
}
#endif


// --------------------------------------------------------------------------------


void config_edit_serial_device(byte dev)
{
	uint32_t settings = config_serial_device_settings[dev];
	byte row, col, r_iface, r_baud, r_force, r_nuls, r_7bits, r_ucase, r_bspace, r_traps, r_cmd, r_rev;
	bool redraw = true;

	while (true)
	{
		if (redraw)
		{
			row = 4;
			col = 30;
			Serial.print(F("\033[2J\033[0;0H\n"));
			Serial.print(F("Configure serial device ")); print_serial_device_sim(dev); Serial.println();
			Serial.print(F("\nMap to host (i)nterface    : ")); r_iface = row++; print_serial_device_mapped_to(settings); Serial.println();
			Serial.print(F("Simulated (b)aud rate      : ")); r_baud = row++; Serial.println(config_baud_rate(get_bits(settings, 0, 4)));
			Serial.print(F("(F)orce baud rate          : ")); r_force = row++; print_flag(settings, 1ul << 16, 0, 0); Serial.println();
			Serial.print(F("Example playback (N)ULs    : ")); r_nuls = row++; Serial.println(get_bits(settings, 4, 3));
			Serial.print(F("Use (7) bits               : ")); r_7bits = row++; print_serial_flag(settings, 12); Serial.println();
			Serial.print(F("Serial input (u)ppercase   : ")); r_ucase = row++; print_serial_flag(settings, 10); Serial.println();
			Serial.print(F("Translate (B)ackspace to   : ")); r_bspace = row++; print_serial_flag_backspace(settings); Serial.println();
			if (dev == CSM_ACR)
			{
				Serial.print(F("Enable CLOAD/CSAVE (t)raps : ")); r_traps = row++; print_serial_flag(settings, 7, 1); Serial.println();
			}
			else if (dev == CSM_SIO)
			{
				Serial.print(F("SIO board re(v)ision       : ")); r_rev = row++; print_serial_flag_siorev(settings); Serial.println();
			}

			Serial.println(F("\nE(x)it to previous menu"));

			Serial.print(F("\n\nCommand: ")); r_cmd = row + 4;
			redraw = false;
		}
		else
			set_cursor(r_cmd, 10);

		while (!serial_available()) delay(50);
		char c = serial_read();
		if (c > 31 && c < 127) Serial.println(c);

		switch (c)
		{
		case 'i':
		{
			bool ok = true;
			if (get_bits(settings, 17, 3) == 7)
			{
				ok = false;
				for (byte d = 0; d < 4 && !ok; d++)
					if (d != dev && get_bits(config_serial_device_settings[d], 17, 3) == 7)
						ok = true;

				if (!ok)
				{
					Serial.println(F("\n\nCan not change mapping. At least one device must"));
					Serial.println(F("be mapped to the host's (primary) serial interface."));
					redraw = true;
					delay(4000);
				}
			}

			if (ok)
			{
				settings = toggle_map_to_serial(settings, 17);
				set_cursor(r_iface, col);
				print_serial_device_mapped_to(settings);
			}

			break;
		}

		case 'b':
		{
			settings = toggle_bits(settings, 0, 4, BAUD_110, BAUD_19200);
			set_cursor(r_baud, col);
			Serial.print(config_baud_rate(get_bits(settings, 0, 4)));
			break;
		}

		case 'F':
		{
			settings = toggle_bits(settings, 16, 1);
			print_flag(settings, 1ul << 16, r_force, col);
			break;
		}

		case 'N':
		{
			settings = toggle_bits(settings, 4, 3);
			set_cursor(r_nuls, col);
			Serial.println(get_bits(settings, 4, 3));
			break;
		}

		case '7':
		{
			settings = toggle_serial_flag(settings, 12);
			set_cursor(r_7bits, col);
			print_serial_flag(settings, 12);
			break;
		}

		case 'u':
		{
			settings = toggle_serial_flag(settings, 10);
			set_cursor(r_ucase, col);
			print_serial_flag(settings, 10);
			break;
		}

		case 'B':
		{
			settings = toggle_serial_flag_backspace(settings);
			set_cursor(r_bspace, col);
			print_serial_flag_backspace(settings);
			break;
		}

		case 't':
		{
			settings = toggle_bits(settings, 7, 1);
			set_cursor(r_traps, col);
			print_serial_flag(settings, 7, 1);
			break;
		}

		case 'v':
		{
			settings = toggle_bits(settings, 8, 2, 0, 2);
			set_cursor(r_rev, col);
			print_serial_flag_siorev(settings);
			break;
		}

		case 27:
		case 'x':
			config_serial_device_settings[dev] = settings;
			config_serial_sim_to_host[dev] = config_map_device_to_host_interface(get_bits(settings, 17, 3));
			serial_timer_interrupt_setup(dev);
			serial_register_ports();
			return;
		}
	}
}


void config_serial_devices()
{
	bool redraw = true;

	while (true)
	{
		char c;
		if (redraw)
		{
			Serial.print(F("\033[2J\033[0;0H"));
			Serial.println(F("\nConfigure serial cards\n"));

			Serial.print(F("(1) Configure SIO             : ")); print_serial_device_mapped_to(config_serial_device_settings[CSM_SIO]); Serial.println();
			Serial.print(F("(2) Configure ACR             : ")); print_serial_device_mapped_to(config_serial_device_settings[CSM_ACR]); Serial.println();
			Serial.print(F("(3) Configure 2SIO port 1     : ")); print_serial_device_mapped_to(config_serial_device_settings[CSM_2SIO1]); Serial.println();
			Serial.print(F("(4) Configure 2SIO port 2     : ")); print_serial_device_mapped_to(config_serial_device_settings[CSM_2SIO2]); Serial.println();
#if USE_SECOND_2SIO>0
			Serial.print(F("(5) Configure 2nd 2SIO port 1 : ")); print_serial_device_mapped_to(config_serial_device_settings[CSM_2SIO3]); Serial.println();
			Serial.print(F("(6) Configure 2nd 2SIO port 2 : ")); print_serial_device_mapped_to(config_serial_device_settings[CSM_2SIO4]); Serial.println();
#endif
			Serial.println(F("\nE(x)it to main menu"));
			Serial.print(F("\n\nCommand: "));
		}
		else
			set_cursor(14, 10);

		while (!serial_available()) delay(50);
		c = serial_read();
		if (c > 31 && c < 127) Serial.println(c);

		redraw = true;
		switch (c)
		{
		case '1': config_edit_serial_device(CSM_SIO); break;
		case '2': config_edit_serial_device(CSM_ACR); break;
		case '3': config_edit_serial_device(CSM_2SIO1); break;
		case '4': config_edit_serial_device(CSM_2SIO2); break;
#if USE_SECOND_2SIO>0
		case '5': config_edit_serial_device(CSM_2SIO3); break;
		case '6': config_edit_serial_device(CSM_2SIO4); break;
#endif
#if USE_PRINTER>0
		case 'P': config_edit_printer(); break;
#endif

		case 27:
		case 'x':
			return;

		default:
			redraw = false;
			break;
		}
	}
}


// --------------------------------------------------------------------------------


void config_host_serial_details(byte iface)
{
	bool go = true, redraw = true;
	byte row, col, r_baud, r_bits, r_parity, r_stop, r_cmd;

	byte baud = get_bits(new_config_serial_settings, config_baud_rate_bits(iface), 4);
	byte bits = get_bits(new_config_serial_settings2, iface * 5 + 3, 2) + 5;
	byte parity = get_bits(new_config_serial_settings2, iface * 5 + 1, 2);
	byte stop = get_bits(new_config_serial_settings2, iface * 5, 1) + 1;

	row = 4;
	col = 15;
	while (go)
	{
		if (redraw)
		{
			Serial.print(F("\033[2J\033[0;0H\n"));
#if HOST_NUM_SERIAL_PORTS>1
			Serial.print(F("Configure host serial settings for interface: "));
			Serial.print(host_serial_port_name(iface));
			Serial.println();
#else
			Serial.println(F("Configure host serial settings"));
#endif
			Serial.print(F("\n(B)aud rate : ")); Serial.println(config_baud_rate(baud)); r_baud = row++;
			Serial.print(F("(b)its      : ")); Serial.println(bits); r_bits = row++;
			Serial.print(F("(P)arity    : ")); print_parity(parity); Serial.println(); r_parity = row++;
			Serial.print(F("(S)top bits : ")); Serial.println(stop); r_stop = row++;

			Serial.println(F("\nE(x)it to parent menu"));
			row += 2;

			Serial.print(F("\n\nCommand: ")); r_cmd = row + 2;
			redraw = false;
		}
		else
			set_cursor(r_cmd, 10);

		while (!serial_available()) delay(50);
		char c = serial_read();
		if (c > 31 && c < 127) Serial.println(c);

		switch (c)
		{
		case 'B':
		{
			baud = toggle_host_serial_baud_rate(iface, baud);
			set_cursor(r_baud, col);
			Serial.print(config_baud_rate(baud)); Serial.print(F("    "));
			break;
		}

		case 'b':
		{
			bits = bits + 1;
			if (bits > 8) bits = 5;
			set_cursor(r_bits, col);
			Serial.print(bits);
			break;
		}

		case 'P':
		{
			parity = (parity + 1) % 3;
			set_cursor(r_parity, col);
			print_parity(parity);
			break;
		}

		case 'S':
		{
			stop = stop + 1;
			if (stop > 2) stop = 1;
			set_cursor(r_stop, col);
			Serial.print(stop);
			break;
		}

		case 27:
		case 'x':
		{
			go = false;
			break;
		}
		}
	}

	byte config = (bits - 5) * 8 + (parity * 2) + (stop - 1);
	new_config_serial_settings = set_bits(new_config_serial_settings, config_baud_rate_bits(iface), 4, baud);
	new_config_serial_settings2 = set_bits(new_config_serial_settings2, iface * 5, 5, config);
}


// --------------------------------------------------------------------------------


void config_host_serial()
{
	int i;
	bool go = true, redraw = true;
	byte row, col, r_baud[HOST_NUM_SERIAL_PORTS], r_primary, r_cmd;

	col = 0;
	for (i = 0; i < HOST_NUM_SERIAL_PORTS; i++)
		col = (byte)max(col, strlen(host_serial_port_name(i)));
	col += 8;

	while (go)
	{
		if (redraw)
		{
			Serial.print(F("\033[2J\033[0;0H\n"));
			Serial.println(F("Configure host serial settings\n"));

			row = 4;
			for (i = 0; i < HOST_NUM_SERIAL_PORTS; i++)
			{
				r_baud[i] = row++;
				Serial.print('('); Serial.print(i); Serial.print(F(") "));
				Serial.print(host_serial_port_name(i));
				set_cursor(r_baud[i], col - 3);
				Serial.print(F(" : "));
				print_host_serial_config(i, 0, 0);
				Serial.println();
			}

#if HOST_NUM_SERIAL_PORTS>1
			Serial.print(F("\n(P)rimary host serial : ")); print_host_primary_interface(); row++; r_primary = row++; Serial.println();
#endif

			Serial.println(F("\n(A)pply host serial settings")); row += 2;

			Serial.println(F("\nE(x)it to main menu"));
			row += 2;

			Serial.print(F("\n\nCommand: ")); r_cmd = row + 2;
			redraw = false;
		}
		else
			set_cursor(r_cmd, 10);

		while (!serial_available()) delay(50);
		char c = serial_read();
		if (c > 31 && c < 127) Serial.println(c);

		switch (c)
		{
#if HOST_NUM_SERIAL_PORTS>1
		case 'P': toggle_host_primary_interface(r_primary, 25); Serial.print(F("\033[K")); break;
#endif

		case 'A':
		{
			if (check_video_interface_settings()) apply_host_serial_settings();
			redraw = true;
			break;
		}

		case 27:
		case 'x':
		{
			go = false;
			break;
		}

		default:
		{
			i = c - '0';
			if (i >= 0 && i < HOST_NUM_SERIAL_PORTS)
			{
				if (host_serial_port_has_configs(i))
				{
					config_host_serial_details(i);
					redraw = true;
				}
				else
				{
					byte baud = get_bits(new_config_serial_settings, config_baud_rate_bits(i), 4);
					baud = toggle_host_serial_baud_rate(i, baud);
					new_config_serial_settings = set_bits(new_config_serial_settings, config_baud_rate_bits(i), 4, baud);
					print_host_serial_config(i, r_baud[i], col);
				}
			}
			break;
		}
		}
	}
}


// --------------------------------------------------------------------------------


static void config_toggle_rom_flag(byte i, uint16_t f)
{
	uint16_t flags;
	mem_get_rom_info(i, NULL, NULL, NULL, &flags);
	mem_set_rom_flags(i, flags ^ f);
}


void config_memory()
{
	byte row, col, r_memsize, r_clearmem, r_cmd;
	bool redraw = true;

	while (true)
	{
		if (redraw)
		{
			row = 4;
			col = 30;
			Serial.print(F("\033[2J\033[0;0H\n"));
			Serial.println(F("Configure memory"));
			if (mem_get_num_roms() > 0)
			{
				Serial.println(F("\nInstalled ROMS: "));
				row += 2;

				byte i = 0;
				char name[9];
				uint16_t start, length, flags;
				while (mem_get_rom_info(i++, name, &start, &length, &flags))
				{
					numsys_print_byte(i);
					Serial.print(F(") "));
					numsys_print_word(start);
					Serial.print('-');
					numsys_print_word(start + length - 1);
					Serial.print(F(": "));
					Serial.print(name);
					if (flags != 0)
					{
						bool comma = false;
						Serial.print(F(" ("));
						if (flags & MEM_ROM_FLAG_AUTOSTART) { if (comma) Serial.print(','); else comma = true; Serial.print(F("auto-start")); }
						if (flags & MEM_ROM_FLAG_TEMP) { if (comma) Serial.print(','); else comma = true; Serial.print(F("temporary")); }
						if (flags & MEM_ROM_FLAG_DISABLED) { if (comma) Serial.print(','); else comma = true; Serial.print(F("disabled")); }
						Serial.print(')');
					}
					Serial.println();
					row++;
				}
			}

			Serial.print(F("\nRAM size (+/-)             : ")); print_mem_size(config_mem_size, row, col); Serial.println(); r_memsize = row++;
			Serial.print(F("(c)lear memory on powerup  : ")); print_flag(CF_CLEARMEM); Serial.println(); r_clearmem = row++;
			Serial.println(F("(C)lear memory now")); row++;
#if MAX_NUM_ROMS>0
			Serial.println(F("(A)dd ROM")); row++;
			if (mem_get_num_roms() > 0)
			{
				Serial.println(F("(R)emove ROM")); row++;
				Serial.println(F("(a)uto-start ROM")); row++;
			}
#endif          
			Serial.println(F("\nE(x)it to previous menu"));
			Serial.print(F("\n\nCommand: ")); r_cmd = row + 4;
			redraw = false;
		}
		else
			set_cursor(r_cmd, 10);

		while (!serial_available()) delay(50);
		char c = serial_read();
		if (c > 31 && c < 127) Serial.println(c);

		switch (c)
		{
		case 'c':
			toggle_flag(CF_CLEARMEM, r_clearmem, col);
			redraw = false;
			break;

		case 'C':
		{
			mem_reset_roms();
			mem_ram_init(0, MEMSIZE - 1, true);
			Serial.print(F("\r\nMemory cleared."));
			delay(1500);
			redraw = true;
			break;
		}

#if MAX_NUM_ROMS>0
		case 'A':
		{
			bool ESC = false, ok = false;
			uint16_t start, end;
			char name[9];
			redraw = true;

			Serial.println(F("\r\nSend ROM content in Intel HEX format now...\n"));
			ok = altair_read_intel_hex(&start, &end);
			while (true) { if (serial_read() < 0) { delay(15); if (serial_read() < 0) { delay(150); if (serial_read() < 0) break; } } }

			if (ok)
			{
				Serial.print(F("\r\nName (max 8 characters): "));
				if (config_read_string(name, 9))
					ok = mem_add_rom(start, end - start + 1, name);
			}
			else
				Serial.println(F("Error!"));

			if (!ok)
			{
				Serial.print(F("\n\nPress any key to continue..."));
				while (!serial_available());
				while (serial_available()) serial_read();
			}

			break;
		}

		case 'R':
		{
			if (mem_get_num_roms() > 0)
			{
				byte i;
				redraw = true;
				Serial.print(F("Remove ROM number: "));
				if (numsys_read_byte(&i))
					if (i > 0 && i <= mem_get_num_roms())
						mem_remove_rom(i - 1);
			}
			break;
		}

		case 'a':
		{
			if (mem_get_num_roms() > 0)
			{
				byte i;
				redraw = true;
				Serial.print(F("ROM to auto-start (0 for none): "));
				if (numsys_read_byte(&i))
					for (byte j = 0; j < mem_get_num_roms(); j++)
					{
						uint16_t flags;
						mem_get_rom_info(j, NULL, NULL, NULL, &flags);
						mem_set_rom_flags(j, j == (i - 1) ? (flags | MEM_ROM_FLAG_AUTOSTART) : (flags & ~MEM_ROM_FLAG_AUTOSTART));
					}
			}
			break;
		}
#endif

		case '-':
		{
			if (config_mem_size > 1024)
				config_mem_size -= 1024;
			else if (config_mem_size > 256)
				config_mem_size -= 256;
			else
				config_mem_size = MEMSIZE;

			print_mem_size(config_mem_size, r_memsize, col); Serial.print(F("     "));
			redraw = false;
			break;
		}

		case '+':
		{
			if (config_mem_size < 1024)
				config_mem_size += 256;
			else if (config_mem_size < MEMSIZE)
				config_mem_size += 1024;
			else
				config_mem_size = 256;

			print_mem_size(config_mem_size, r_memsize, col); Serial.print(F("     "));
			redraw = false;
			break;
		}

		case 27:
		case 'x':
			return;
		}
	}
}


// --------------------------------------------------------------------------------


void config_edit()
{
	new_config_serial_settings = config_serial_settings;
	new_config_serial_settings2 = config_serial_settings2;

	// flush serial input
	while (serial_read() >= 0);

	bool redraw = true;

	config_mem_size = ((uint32_t)mem_get_ram_limit_usr()) + 1;
	byte row, col, r_cpu, r_profile, r_throttle, r_panel, r_debug, r_aux1, r_cmd, r_input, r_dazzler, r_b_mode;
	while (true)
	{
		char c;

		if (redraw)
		{
			row = 1;
			col = 31;
			Serial.print(F("\033[2J\033[0;0H"));

			Serial.print(F("8800(B) Mode                : ")); print_b_mode(); Serial.println(); r_b_mode = row++;
			Serial.print(F("Enable pro(f)iling          : ")); print_flag(CF_PROFILE); Serial.println(); r_profile = row++;
#if USE_THROTTLE>0
			Serial.print(F("Set throttle delay (t/T)    : ")); print_throttle(); Serial.println(); r_throttle = row++;
#endif
			Serial.print(F("Enable serial (p)anel       : ")); print_flag(CF_SERIAL_PANEL); Serial.println(); r_panel = row++;
#if STANDALONE==0
			Serial.print(F("Enable serial (i)nput       : ")); print_flag(CF_SERIAL_INPUT); Serial.println(); r_input = row++;
#endif
			Serial.print(F("Enable serial (d)ebug       : ")); print_flag(CF_SERIAL_DEBUG); Serial.println(); r_debug = row++;
			Serial.print(F("Configure (m)emory          : ")); print_mem_size(config_mem_size, row, col); row++; Serial.print(F(" RAM"));
#if MAX_NUM_ROMS>0
			Serial.print(F(", ")); Serial.print(mem_get_num_roms(false)); Serial.print(F(" ROMs"));
#endif
			Serial.println();
#if USE_Z80==2
			Serial.print(F("Pro(c)essor                 : ")); print_cpu(); Serial.println(); r_cpu = row++;
#endif
			Serial.print(F("Aux1 shortcut program (u/U) : ")); print_aux1_program(); Serial.println(); r_aux1 = row++;
			Serial.print(F("Configure host (s)erial     : "));
#if HOST_NUM_SERIAL_PORTS>1
			Serial.print(F("Primary: "));
			print_host_primary_interface_aux(get_bits(new_config_serial_settings, 8, 3));
			Serial.println();
			if (get_bits(new_config_serial_settings, 8, 3) != config_host_serial_primary())
			{
				set_cursor(row + 1, col);
				Serial.print(F("Current: "));
				print_host_primary_interface_aux(config_host_serial_primary());
			}
#else
			print_host_serial_config(0, 0, 0); Serial.println();
#endif
			row += 2;
			Serial.println();
			Serial.print(F("(E) Configure serial cards  : ")); print_mapped_serial_cards(); Serial.println(F(" mapped")); row++;

#if USE_PRINTER>0
			Serial.print(F("(P) Configure printer       : "));
			print_printer_type();
			if (config_printer_type() != CP_NONE) { Serial.print(F(" on ")); print_printer_mapped_to(); }
			Serial.println(); row++;
#endif
#if NUM_DRIVES>0
			Serial.print(F("(D) Configure disk drives   : ")); print_drive_mounted(); Serial.println(); row++;
#endif
#if NUM_CDRIVES>0
			Serial.print(F("(C) Configure cromemco drive: ")); print_cdrive_mounted(); Serial.println(); row++;
#endif
#if NUM_TDRIVES>0
			Serial.print(F("(B) Configure tarbell drive : ")); print_tdrive_mounted(); Serial.println(); row++;
#endif
#if NUM_HDSK_UNITS>0
			Serial.print(F("(H) Configure hard disks    : ")); print_hdsk_mounted(); Serial.println(); row++;
#endif
#if USE_DAZZLER>0
			Serial.print(F("(Z) Configure Dazzler       : ")); print_dazzler_mapped_to(); Serial.println(); r_dazzler = row++;
#endif
#if USE_VDM1>0
			Serial.print(F("(V) Configure VDM-1         : ")); print_vdm1_mapped_to(); Serial.println(); row++;
#endif
			Serial.print(F("(I) Configure interrupts    : ")); print_vi_flag(); Serial.println(); row++;
			row += 1;

			Serial.println();
#if USE_HOST_FILESYS==0
			Serial.print(F("(M)anage Filesystem     "));
#endif
#if (NUM_DRIVES>0 || NUM_HDSK_UNITS>0 || USE_HOST_FILESYS>0) && defined(HOST_HAS_FILESYS)
			if (host_filesys_ok()) Serial.print(F("(F)ile manager for SD card"));
#endif
			Serial.println(); row++;
			Serial.println(F("(S)ave configuration    (L)oad configuration"));
			Serial.println(F("(R)eset to defaults     E(x)it"));
			row += 2;

			Serial.print(F("\nCommand: ")); r_cmd = row + 1;
		}
		else
			set_cursor(r_cmd, 10);

		while (!serial_available()) delay(50);
		c = serial_read();
		if (c > 31 && c < 127) Serial.print(c);

		redraw = true;
		switch (c)
		{
			// B Mode
		case 'B':
			toggle_b_mode();
			set_cursor(r_b_mode, col);
			print_b_mode();
			redraw = false;
			break;
#if USE_Z80==2
		case 'c':
			config_flags2 = toggle_bits(config_flags2, 21, 1);
			set_cursor(r_cpu, col);
			print_cpu();
			redraw = false;
			break;
#endif

		case 'f': toggle_flag(CF_PROFILE, r_profile, col); redraw = false; break;
#if USE_THROTTLE>0
		case 't': toggle_throttle(r_throttle, col, true); redraw = false; break;
		case 'T': toggle_throttle(r_throttle, col, false); redraw = false; break;
#endif
		case 'p': toggle_flag(CF_SERIAL_PANEL, r_panel, col); redraw = false; break;
#if STANDALONE==0
		case 'i': toggle_flag(CF_SERIAL_INPUT, r_input, col); redraw = false; break;
#endif
		case 'd': toggle_flag(CF_SERIAL_DEBUG, r_debug, col); redraw = false; break;
		case 'u': toggle_aux1_program_up(r_aux1, col); redraw = false; break;
		case 'U': toggle_aux1_program_down(r_aux1, col); redraw = false; break;
		case 's': config_host_serial(); break;
		case 'E': config_serial_devices(); break;
		case 'm': config_memory(); break;

#if USE_PRINTER>0
		case 'P': config_edit_printer(); break;
#endif

		case 'I': config_edit_interrupts(); break;
#if NUM_DRIVES>0
		case 'D': config_edit_drives(); break;
#endif
#if NUM_CDRIVES>0
		case 'C': config_edit_cdrives(); break;
#endif
#if NUM_TDRIVES>0
		case 'B': config_edit_tdrives(); break;
#endif
#if NUM_HDSK_UNITS>0
		case 'H': config_edit_hdsk(); break;
#endif
#if USE_DAZZLER>0
		case 'Z':
			config_flags2 = toggle_bits(config_flags2, 0, 3, 0, HOST_NUM_SERIAL_PORTS);
			set_cursor(r_dazzler, col);
			print_dazzler_mapped_to();
			dazzler_register_ports();
			redraw = false;
			break;
#endif
#if USE_VDM1>0
		case 'V':
			config_edit_vdm1(); break;
#endif

		case 'h':
		{
			Serial.println(F("\n\n"));
			Serial.println(F("Altair 8800 Simulator (C) 2017-2020 David Hansel"));
			Serial.println(F("https://www.hackster.io/david-hansel/arduino-altair-8800-simulator-3594a6"));
			Serial.println(F("https://github.com/dhansel/Altair8800"));
			Serial.println(F("Firmware compiled on: " __DATE__ ", " __TIME__ "\n"));
			host_system_info();
			Serial.println();
			io_print_registered_ports();
			Serial.print(F("\n\nPress any key to continue..."));
			while (!serial_available());
			while (serial_available()) serial_read();
			break;
		}

#if USE_HOST_FILESYS==0
		case 'M': filesys_manage(); break;
#endif

#if (NUM_DRIVES>0 || NUM_HDSK_UNITS>0 || USE_HOST_FILESYS>0) && defined(HOST_HAS_FILESYS)
		case 'F': if (host_filesys_ok()) sd_manager(); break;
#endif

		case 'S':
		{
			if (check_video_interface_settings())
			{
				byte i;
				Serial.print(F("\r\nSave as config # (0=default): "));
				if (numsys_read_byte(&i))
				{
					bool ok = true;
					Serial.println();
					if (filesys_exists('C', i))
					{
						char c;
						Serial.print(F("Configuration #")); numsys_print_byte(i);
						Serial.print(F(" exists. Overwrite (y/n)? "));
						do { delay(50); c = serial_read(); } while (c != 'y' && c != 'n');
						Serial.println(c);
						ok = (c == 'y');
					}

					if (ok && !save_config(i))
					{
#if USE_HOST_FILESYS>0
						Serial.println(F("Saving failed.?"));
#else
						Serial.println(F("Saving failed. Capture/replay in progress?"));
#endif
						delay(2000);
					}
				}
			}

			break;
		}
		case 'L':
		{
			byte i;
			Serial.print(F("\r\nLoad config #: "));
			if (numsys_read_byte(&i))
			{
				Serial.println();
				if (!load_config(i & 0xff))
				{
					Serial.println(F("Load failed. File does not exist?"));
					delay(2000);
				}
			}
			break;
		}
		case 'R': config_defaults(false); break;

		case '?':
			redraw = true;
			break;

		case 27:
		case 'x':
		{
			bool exit = false;

			if (check_video_interface_settings())
			{
				if ((config_serial_settings & 0xFFF7FF) != (new_config_serial_settings & 0xFFF7FF) ||
					(config_serial_settings2) != (new_config_serial_settings2))
				{
					Serial.print(F("\r\nApply new host serial settings (y/n/ESC)? "));
					do { delay(50); c = serial_read(); } while (c != 'y' && c != 'n' && c != 27);
					Serial.println(c);
					if (c == 'n' || (c == 'y' && apply_host_serial_settings()))
						exit = true;
				}
				else
				{
					dazzler_set_iface(config_dazzler_interface());
					vdm1_set_iface(config_vdm1_interface());
					exit = true;
				}
			}

			if (exit)
			{
				mem_set_ram_limit_usr(config_mem_size - 1);
				serial_set_config();
#if USE_Z80==2
				cpu_set_processor(config_use_z80() ? PROC_Z80 : PROC_I8080);
#endif
				Serial.print(F("\033[2J\033[0;0H"));
				return;
			}
			break;
		}
		default:
			redraw = false;
			break;
		}
	}
}


void config_defaults(bool apply)
{
	byte i, j;
	// default settings:
	// - SERIAL_DEBUG, SERIAL_INPUT, SERIAL_PANEL enabled if in STANDALONE mode, otherwise disabled
	// - Profiling disabled
	// - Throttling enabled (on Due)

	config_current = 0;
	config_flags = 0;
#if STANDALONE>0
	config_flags |= CF_SERIAL_DEBUG;
	config_flags |= CF_SERIAL_INPUT;
	config_flags |= CF_SERIAL_PANEL;
#endif
	config_flags |= CF_THROTTLE;

	config_flags2 = 0;              // Dazzler not enabled
	config_flags2 |= 0 << 3;         // VDM-1 not enabled
	config_flags2 |= B00110110 << 6; // VDM-1 DIP: 1=off, 2=on, 3=on, 4=off, 5=on, 6=on
	config_flags2 |= (0xCC00 >> 10) << 12; // VDM-1 base address : CC00

	new_config_serial_settings = 0;
	new_config_serial_settings |= (0 << 8); // USB Programming port is primary interface

	// USB ports 115200 baud, serial interfaces 9600 baud
	for (i = 0; i < HOST_NUM_SERIAL_PORTS; i++)
		if (strstr(host_serial_port_name(i), "USB") || HOST_NUM_SERIAL_PORTS == 1)
			new_config_serial_settings |= (BAUD_115200 << (config_baud_rate_bits(i)));
		else
			new_config_serial_settings |= (BAUD_9600 << (config_baud_rate_bits(i)));

	// serial configuration defaults to 8N1
	new_config_serial_settings2 = 0;
	for (i = 0; i < HOST_NUM_SERIAL_PORTS; i++)
		new_config_serial_settings2 |= 0x18ul << (i * 5);

	uint32_t s = 0;
	s |= (BAUD_1200 << 0); // serial playback baud rate: 1200
	s |= (4 << 4); // 4 NUL characters after newline
	s |= (1 << 8); // rev1 SIO board
	s |= (CSF_AUTO << 10); // autodetect uppercase inputs
	s |= (CSF_AUTO << 12); // autodetect 7 bit 
	s |= (CSFB_NONE << 14); // no backspace translation
	s |= (0l << 17); // not mapped to any host interface

	for (byte dev = 0; dev < NUM_SERIAL_DEVICES; dev++)
		config_serial_device_settings[dev] = s;

#if HOST_NUM_SERIAL_PORTS>1
	config_serial_device_settings[CSM_SIO] |= (7l << 17); // map to SIO to primary host interface
	config_serial_device_settings[CSM_2SIO1] |= (7l << 17); // map to 2SIO-1 to primary host interface
#else
	config_serial_device_settings[CSM_SIO] |= (1l << 17); // map to SIO to host interface
	config_serial_device_settings[CSM_2SIO1] |= (1l << 17); // map to 2SIO-1 to host interface
#endif
	config_serial_device_settings[CSM_ACR] |= (1 << 7);  // enable CLOAD traps

	for (byte dev = 0; dev < NUM_SERIAL_DEVICES; dev++)
		config_serial_sim_to_host[dev] = config_map_device_to_host_interface(get_bits(config_serial_device_settings[dev], 17, 3));

	config_interrupt_vi_mask[0] = INT_DRIVE;
	config_interrupt_vi_mask[1] = INT_RTC;
	config_interrupt_vi_mask[2] = INT_2SIO1 | INT_2SIO2;
	config_interrupt_vi_mask[3] = INT_2SIO3 | INT_2SIO4;
	config_interrupt_vi_mask[4] = 0;
	config_interrupt_vi_mask[5] = 0;
	config_interrupt_vi_mask[6] = 0;
	config_interrupt_vi_mask[7] = 0;

	config_interrupt_mask = INT_SIO | INT_2SIO1;

	config_aux1_prog = prog_find("16k ROM Basic");

	drive_set_realtime((config_flags & CF_DRIVE_RT) != 0);
	hdsk_set_realtime((config_flags & CF_DRIVE_RT) != 0);
	cdrive_set_switches(CDRIVE_SWITCH_ROM_DISABLE_AFTER_BOOT);
	for (i = 0; i < NUM_DRIVES; i++) drive_unmount(i);
	for (i = 0; i < NUM_CDRIVES; i++) cdrive_unmount(i);
	for (i = 0; i < NUM_TDRIVES; i++) tdrive_unmount(i);
	for (i = 0; i < NUM_HDSK_UNITS; i++)
		for (j = 0; j < 4; j++)
			hdsk_unmount(i, j);

	// maximum amount of RAM supported by host
	config_mem_size = MEMSIZE;
	mem_clear_roms();

	config_printer_generic_status_busy = 0x00;
	config_printer_generic_status_ready = 0xFF;

	// note: registered ports for drives are controlled by mount/unmount functions
	serial_register_ports();
	printer_register_ports();
	altair_vi_register_ports();
	dazzler_register_ports();
	dazzler_register_ports();

	if (apply)
	{
		apply_host_serial_settings(new_config_serial_settings, new_config_serial_settings2);
		mem_set_ram_limit_usr(config_mem_size - 1);
	}
}


byte config_get_current()
{
	return config_current;
}


void config_setup(int n)
{
	config_defaults(true);

	bool ok = true;
	if (n >= 0)
	{
		ok = load_config(n);
		if (!ok && n > 0)
		{
			Serial.print(F("Configuration ")); Serial.print(n);
			Serial.println(F(" does not exist => using default configuration (0)"));
			ok = load_config(0);
		}
	}

	if (ok)
	{
		apply_host_serial_settings(new_config_serial_settings, new_config_serial_settings2);
		mem_set_ram_limit_usr(config_mem_size - 1);
	}
}
