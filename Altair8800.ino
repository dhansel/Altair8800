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
#include "cpucore.h"
#include "host.h"
#include "mem.h"
#include "serial.h"
#include "printer.h"
#include "profile.h"
#include "breakpoint.h"
#include "disassembler.h"
#include "numsys.h"
#include "filesys.h"
#include "drive.h"
#include "tdrive.h"
#include "cdrive.h"
#include "hdsk.h"
#include "timer.h"
#include "prog.h"
#include "dazzler.h"
#include "vdm1.h"
#include "io.h"

#define BIT(n) (1<<(n))
#define b2s numsys_byte2string

uint16_t cswitch = 0;
uint16_t dswitch = 0;

static uint16_t p_regPC = 0xFFFF;

volatile uint32_t altair_interrupts_buf = 0;
volatile uint32_t altair_interrupts = 0;
volatile byte     altair_vi_level_cur = 0;
volatile byte     altair_vi_level = 0;
volatile bool     altair_interrupts_enabled = false;
static   bool     altair_rtc_available = false;
static   bool     altair_rtc_running = false;
volatile bool     sticky_slow = false;

word status_wait = false;
bool have_ps2 = false;

void print_panel_serial(bool force = false);
void print_dbg_info();
void read_inputs_panel();
void read_inputs_serial();
void process_inputs();
void empty_input_buffer();
void rtc_setup();
void print_help_menu();

#if USE_THROTTLE>0
uint32_t  throttle_micros = 0;
uint16_t  throttle_delay = 0;

#define THROTTLE_TIMER_PERIOD 25000
void update_throttle()
{
	uint32_t now = micros();
	uint32_t ratio = now == throttle_micros ? 0xffffffff : (2000 * THROTTLE_TIMER_PERIOD) / (now - throttle_micros);
	uint32_t t = cpu_clock_KHz();

	if (ratio > t + 10)
		throttle_delay += (uint16_t)HOST_PERFORMANCE_FACTOR;
	else if (ratio < t && throttle_delay>0)
		throttle_delay -= (uint16_t)HOST_PERFORMANCE_FACTOR;

	throttle_micros = now;
}
#endif


void altair_set_outputs(uint16_t a, byte v)
{
	host_set_addr_leds(a);
	host_set_data_leds(v);
	if (mem_is_protected(a))
		host_set_status_led_PROT();
	else
		host_clr_status_led_PROT();

	if (host_read_status_led_M1()) print_dbg_info();
	print_panel_serial();
}


void altair_wait_reset()
{
	// prevent printing processor status now (will print after reset)
	p_regPC = regPC;
	// set bus/data LEDs on, status LEDs off
	altair_set_outputs(0xffff, 0xff);
	host_clr_status_led_INT();
	host_set_status_led_WO();
	host_clr_status_led_STACK();
	host_clr_status_led_HLTA();
	host_clr_status_led_OUT();
	host_clr_status_led_M1();
	host_clr_status_led_INP();
	host_clr_status_led_MEMR();
	host_clr_status_led_PROT();
	altair_interrupt_disable();

	// wait while RESET switch is held
	while (host_read_function_switch_debounced(SW_RESET));

	// if in single-step mode we need to set a flag so we know
	// to exit out of the currently executing instruction
	if (host_read_status_led_WAIT()) cswitch |= BIT(SW_RESET);
}


void read_inputs()
{
	cswitch = 0;
	read_inputs_panel();
	read_inputs_serial();
	print_panel_serial();
	process_inputs();
}


byte get_device(byte switches)
{
	byte dev;
	if ((switches & 0x80) == 0)
		dev = serial_last_active_primary_device();
	else
		dev = (switches & 0x60) >> 5;

	return dev;
}


void process_inputs()
{
	if (cswitch & BIT(SW_DEPOSIT))
	{
		MWRITE(regPC, dswitch & 0xff);
		altair_set_outputs(regPC, MREAD(regPC));
	}
	else if (cswitch & BIT(SW_DEPNEXT))
	{
		regPC++;
		MWRITE(regPC, dswitch & 0xff);
		altair_set_outputs(regPC, MREAD(regPC));
	}

	if (cswitch & BIT(SW_EXAMINE))
	{
		regPC = dswitch;
		p_regPC = ~regPC;
		altair_set_outputs(regPC, MREAD(regPC));
	}
	else if (cswitch & BIT(SW_EXNEXT))
	{
		regPC = regPC + 1;
		altair_set_outputs(regPC, MREAD(regPC));
	}

	if (cswitch & BIT(SW_AUX1DOWN))
	{
		// B Mode: set accumulator to value in switches A0-A7
		if (config_b_mode() && (cswitch & BIT(SW_EXAMINE)) == 0 && (host_read_function_switch_debounced(SW_EXAMINE) == 0))
		{
			regA = host_read_addr_switches() & 0xff;
			if (config_serial_debug_enabled())
			{
				cpu_print_registers();
			}
		}
		// non-B mode, or EXAMINE switch is held up, load program selected from switches
		else
		{
			serial_acr_check_cload_timeout();

			if (dswitch & 0x80)
			{
				// SW7 is up => memory page operation
				host_set_status_led_HLDA();

				uint16_t page = dswitch & 0xff00;
				byte filenum = dswitch & 0x003f;
				if (dswitch & 0x40)
				{
					// SW6 is up => save memory page
					if (filesys_write_file('M', filenum, Mem + page, 256))
						DBG_FILEOPS4(3, F("saved memory page "), b2s(page >> 8), F(" to file #"), b2s(filenum));
					else
						DBG_FILEOPS4(2, F("unable to save memory page "), b2s(page >> 8), F(" to file #"), b2s(filenum));
				}
				else
				{
					// SW6 is down => load memory page
					if (!MEM_IS_WRITABLE(page))
						DBG_FILEOPS3(2, F("memory page "), b2s(page >> 8), F(" is not writable"));
					else if (filesys_read_file('M', filenum, Mem + page, 256) == 256)
					{
						DBG_FILEOPS4(3, F("loaded memory page "), b2s(page >> 8), F(" from file #"), b2s(filenum));
						regPC = page;
					}
					else
						DBG_FILEOPS4(2, F("file not found loading memory page "), b2s(page >> 8), F(" from file #"), b2s(filenum));

					altair_set_outputs(regPC, MREAD(regPC));
				}

				serial_update_hlda_led();
			}
			else if (dswitch & 0x40)
			{
				Serial.println(F("\r\nReceiving Intel HEX data..."));
				if (altair_read_intel_hex())
					Serial.println(F("Success."));
				else
					Serial.println(F("Error!"));

				// skip any remaining input
				empty_input_buffer();
			}
			else
			{
				// SW7 is down => load program
				if (prog_load(dswitch & 0xff, &regPC))
				{
					// program exists and is supposed to run immediately
					host_set_data_leds(MREAD(regPC));
					host_clr_status_led_WAIT();
					host_clr_status_led_PROT();
				}
				else
				{
					// either program does not exist or is not supposed to run immediately
					p_regPC = regPC;
					altair_set_outputs(regPC, MREAD(regPC));
				}
			}
		}
	}
	else if (cswitch & BIT(SW_AUX1UP))
	{
		if ((cswitch & BIT(SW_STOP)) || host_read_function_switch_debounced(SW_STOP))
		{
			// STOP is up => edit configuration
			config_edit();
			cswitch = 0;
			if (config_serial_panel_enabled())
			{
				Serial.print(F("\033[14B"));
				print_panel_serial(true);
			}

			p_regPC = ~regPC;
			altair_set_outputs(regPC, MREAD(regPC));
			rtc_setup();
		}
		// B Mode: examine the Accumulator if EXAMINE is not up
		else if (config_b_mode() && (cswitch & BIT(SW_EXAMINE)) == 0 && (host_read_function_switch_debounced(SW_EXAMINE) == 0))
		{
			altair_set_outputs(regPC, regA);

			if (config_serial_debug_enabled())
			{
				cpu_print_registers();
			}
		}
		else
		{
			// program shortcut
			byte p = config_aux1_program();
			if (p < 0x40)
			{
				if (prog_get_name(p) != NULL)
				{
					// run program
					dswitch = (dswitch & 0xff00) | p;
					cswitch = BIT(SW_AUX1DOWN);
					process_inputs();
				}
			}
			else if (p < 0x80)
			{
				// run tarbell disk (first mount disk then install and run tarbell disk boot ROM)
				tdrive_reset();
				if (tdrive_mount(0, p & 0x3f))
				{
					dswitch = (dswitch & 0xff00) | 16;
					cswitch = BIT(SW_AUX1DOWN);
					process_inputs();
				}
			}
			else if (p < 0xC0)
			{
				// run disk (first mount disk then install and run disk boot ROM)
				if (drive_mount(0, p & 0x3f))
				{
					dswitch = (dswitch & 0xff00) | 8;
					cswitch = BIT(SW_AUX1DOWN);
					process_inputs();
				}
			}
			else
			{
				// run hard disk (first mount disk then install and run hard disk boot ROM)
				if (hdsk_mount(0, 0, p & 0x3f))
				{
					dswitch = (dswitch & 0xff00) | 14;
					cswitch = BIT(SW_AUX1DOWN);
					process_inputs();
				}
			}
		}
	}

	if (cswitch & BIT(SW_AUX2DOWN))
	{
		if (config_b_mode() && (cswitch & BIT(SW_EXAMINE)) == 0 && (host_read_function_switch_debounced(SW_EXAMINE) == 0))
		{
			byte port = host_read_sense_switches();
			host_clr_status_led_WAIT(); // workaround to prevent altair_out from blocking
			altair_out(port, regA);
			host_set_status_led_WAIT();

			Serial.print(F("Write port "));
			numsys_print_byte_hex(port);
			Serial.print(F(" data "));
			numsys_print_byte_hex(regA);
			Serial.println();
		}
#if NUM_DRIVES>0
		else if ((dswitch & 0xF000) == 0x1000)
		{
			if ((dswitch & 0xff) == 0)
				drive_dir();
			else
			{
				const char* desc = drive_get_image_description(dswitch & 0xff);
				if (drive_mount((dswitch >> 8) & 0x0f, dswitch & 0xff))
				{
					if (desc == NULL)
						DBG_FILEOPS4(2, F("mounted new disk image "), drive_get_image_filename(dswitch & 0xff, false), F(" in drive "), (dswitch >> 8) & 0x0f);
					else
						DBG_FILEOPS4(2, F("mounted disk image '"), desc, F("' in drive "), (dswitch >> 8) & 0x0f);
				}
				else
					DBG_FILEOPS4(1, F("error mounting disk image "), drive_get_image_filename(dswitch & 0xff, false), F(" in drive "), (dswitch >> 8) & 0x0f);
			}
		}
#endif
#if NUM_CDRIVES>0
		else if ((dswitch & 0xF000) == 0x7000)
		{
			if ((dswitch & 0xff) == 0)
				cdrive_dir();
			else
			{
				const char* desc = cdrive_get_image_description(dswitch & 0xff);
				if (cdrive_mount((dswitch >> 8) & 0x03, dswitch & 0xff))
				{
					if (desc == NULL)
						DBG_FILEOPS4(2, F("mounted new disk image "), cdrive_get_image_filename(dswitch & 0xff, false), F(" in cromemco drive "), (dswitch >> 8) & 0x0f);
					else
						DBG_FILEOPS4(2, F("mounted disk image '"), desc, F("' in cromemco drive "), (dswitch >> 8) & 0x0f);
				}
				else
					DBG_FILEOPS4(1, F("error mounting disk image "), cdrive_get_image_filename(dswitch & 0xff, false), F(" in cromemco drive "), (dswitch >> 8) & 0x0f);
			}
		}
#endif
#if NUM_TDRIVES>0
		else if ((dswitch & 0xF000) == 0x5000)
		{
			if ((dswitch & 0xff) == 0)
				tdrive_dir();
			else
			{
				const char* desc = tdrive_get_image_description(dswitch & 0xff);
				if (tdrive_mount((dswitch >> 8) & 0x03, dswitch & 0xff))
				{
					if (desc == NULL)
						DBG_FILEOPS4(2, F("mounted new disk image "), drive_get_image_filename(dswitch & 0xff, false), F(" in tarbell drive "), (dswitch >> 8) & 0x0f);
					else
						DBG_FILEOPS4(2, F("mounted disk image '"), desc, F("' in tarbell drive "), (dswitch >> 8) & 0x0f);
				}
				else
					DBG_FILEOPS4(1, F("error mounting disk image "), drive_get_image_filename(dswitch & 0xff, false), F(" in tarbell drive "), (dswitch >> 8) & 0x0f);
			}
		}
#endif
#if NUM_HDSK_UNITS>0
		else if ((dswitch & 0xF000) == 0x3000)
		{
			if ((dswitch & 0xff) == 0)
				hdsk_dir();
			else
			{
				char buf[50];
				byte unit = (dswitch >> 10) & 0x03;
				byte platter = (dswitch >> 8) & 0x03;
				sprintf(buf, "' in platter %i of unit %i", platter, unit + 1);

				const char* desc = hdsk_get_image_description(dswitch & 0xff);
				if (hdsk_mount(unit, platter, dswitch & 0xff))
				{
					if (desc == NULL)
						DBG_FILEOPS3(2, F("mounted new hard disk image '"), hdsk_get_image_filename(dswitch & 0xff, false), buf);
					else
						DBG_FILEOPS3(2, F("mounted hard disk image '"), desc, buf);
				}
				else
					DBG_FILEOPS3(1, F("error mounting hard disk image '"), hdsk_get_image_filename(dswitch & 0xff, false), buf);
			}
		}
#endif
		else
		{
			print_panel_serial();
			byte dev = get_device(dswitch >> 8);
			if (serial_capture_running(dev))
				DBG_FILEOPS(1, F("unable to replay data (capture operation in progress)"));
			else if (serial_replay_running(dev))
				serial_stop(dev);
			else
				serial_replay_start(dev, (dswitch & 0x100) == 0, dswitch & 0xff);
		}
	}
	else if (cswitch & BIT(SW_AUX2UP))
	{
		// B Mode: Input from port to A register
		if (config_b_mode() && (cswitch & BIT(SW_EXAMINE)) == 0 && (host_read_function_switch_debounced(SW_EXAMINE) == 0))
		{
			host_clr_status_led_WAIT(); // workaround to prevent altair_out from blocking
			byte port = host_read_sense_switches();
			//host_set_status_led_INP();
			//byte data = io_inp(port);
			//altair_set_outputs(port << 8 || port, data);
			//host_clr_status_led_INP();
			//regA = data;
			regA = altair_in(port);
			host_set_status_led_WAIT(); // workaround to prevent altair_out from blocking

			if (config_serial_debug_enabled())
			{
				Serial.print(F("Read port "));
				numsys_print_byte_hex(port);
				Serial.print(F(" data "));
				numsys_print_byte_hex(regA);
				Serial.println();

				//cpu_print_registers();
			}
		}
#if NUM_DRIVES>0
		else if ((dswitch & 0xF000) == 0x1000)
		{
			if (drive_unmount((dswitch >> 8) & 0x0f))
				DBG_FILEOPS2(2, F("unmounted drive "), (dswitch >> 8) & 0x0f);
			else
				DBG_FILEOPS2(1, F("error unmounting drive "), (dswitch >> 8) & 0x0f);
		}
#endif
#if NUM_CDRIVES>0
		else if ((dswitch & 0xF000) == 0x7000)
		{
			if (cdrive_unmount((dswitch >> 8) & 0x03))
				DBG_FILEOPS2(2, F("unmounted cromemco drive "), (dswitch >> 8) & 0x0f);
			else
				DBG_FILEOPS2(1, F("error unmounting cromemco drive "), (dswitch >> 8) & 0x0f);
		}
#endif
#if NUM_TDRIVES>0
		else if ((dswitch & 0xF000) == 0x5000)
		{
			if (tdrive_unmount((dswitch >> 8) & 0x03))
				DBG_FILEOPS2(2, F("unmounted tarbell drive "), (dswitch >> 8) & 0x0f);
			else
				DBG_FILEOPS2(1, F("error unmounting tarbell drive "), (dswitch >> 8) & 0x0f);
		}
#endif
#if NUM_HDSK_UNITS>0
		else if ((dswitch & 0xF000) == 0x3000)
		{
			char buf[50];
			byte unit = (dswitch >> 10) & 0x03;
			byte platter = (dswitch >> 8) & 0x03;
			sprintf(buf, "platter %i of unit %i", platter, unit + 1);

			if (hdsk_unmount(unit, platter))
				DBG_FILEOPS2(1, F("unmounted "), buf);
			else
				DBG_FILEOPS2(1, F("error unmounting "), buf);
		}
#endif
		else
		{
			print_panel_serial();
			byte dev = get_device(dswitch >> 8);
			if (serial_replay_running(dev))
				DBG_FILEOPS(1, F("cannot start capture (replay operation in progress)"));
			else if (serial_capture_running(dev))
				serial_stop(dev);
			else
				serial_capture_start(dev, dswitch & 0xff);
		}
	}

	if (cswitch & BIT(SW_RESET))
	{
		altair_wait_reset();
	}

	if (cswitch & BIT(SW_CLR))
	{
		serial_close_files();
#if USE_DAZZLER>0
		// turn off Dazzler and reset it
		byte i = dazzler_get_iface();
		dazzler_set_iface(0xff);
		dazzler_set_iface(i);
#endif
	}

	if ((cswitch & BIT(SW_RUN)) && (cswitch & BIT(SW_SLOW)))
		sticky_slow = true;
	else if (cswitch & BIT(SW_RUN))
	{
		if (config_serial_debug_enabled() && config_serial_input_enabled())
			Serial.print(F("\r\n\n--- RUNNING (press ESC twice to stop) ---\r\n\n"));
		host_clr_status_led_WAIT();
		host_clr_status_led_PROT();
	}
	if (cswitch & (BIT(SW_PROTECT)))
	{
		mem_protect(regPC);
		if (mem_is_protected(regPC)) host_set_status_led_PROT();
	}
	if (cswitch & (BIT(SW_UNPROTECT)))
	{
		mem_unprotect(regPC);
		if (!mem_is_protected(regPC)) host_clr_status_led_PROT();
	}
}


void altair_wait_step()
{
	cswitch &= BIT(SW_RESET); // clear everything but RESET status
	while (host_read_status_led_WAIT() && (cswitch & (BIT(SW_STEP) | BIT(SW_SLOW) | BIT(SW_RESET))) == 0)
	{
		read_inputs(); delay(10);
	}

	if (cswitch & BIT(SW_SLOW)) delay(500);
}


void read_inputs_panel()
{
	// we react on positive edges on the function switches...
	cswitch = host_read_function_switches_edge();

	// ...except for the SLOW switch which is active as long as it is held down
	if (sticky_slow || host_read_function_switch_debounced(SW_SLOW)) cswitch |= BIT(SW_SLOW);

#if STANDALONE==0
	// address switches on Mega are connected to analog inputs which are free
	// floating and therefore random when not connected
	dswitch = host_read_addr_switches();
#endif
}


byte read_device()
{
	byte dev = 0xff;

#if USE_SECOND_2SIO>0
	Serial.print(F("(s=SIO,a=ACR,1=2SIO-1,2=2SIO-2,3=2nd 2SIO-1,4=2nd 2SIO-2,SPACE=last,ESC=abort): "));
#else
	Serial.print(F("(s=SIO,a=ACR,1=2SIO-1,2=2SIO-2,SPACE=last,ESC=abort): "));
#endif

	int c = -1;
#if USE_SECOND_2SIO>0
	while (c != 's' && c != 'a' && c != '1' && c != '2' && c != '3' && c != '4' && c != ' ' && c != 27)
#else
	while (c != 's' && c != 'a' && c != '1' && c != '2' && c != ' ' && c != 27)
#endif
		c = serial_read();

	if (c != 27)
	{
		switch (c)
		{
		case 's': dev = CSM_SIO;   break;
		case 'a': dev = CSM_ACR;   break;
		case '1': dev = CSM_2SIO1; break;
		case '2': dev = CSM_2SIO2; break;
#if USE_SECOND_2SIO>0
		case '3': dev = CSM_2SIO3; break;
		case '4': dev = CSM_2SIO4; break;
#endif
		case ' ': dev = 0x80;      break;
		}

		Serial.print((char)c);
	}

	return dev;
}


void altair_dump_intel_hex(uint16_t start, uint16_t end)
{
	uint16_t a = start;
	while (a <= end)
	{
		byte cs = 0;
		Serial.write(':');
		byte n = min(16, end - a + 1);
		numsys_print_byte_hex(n); cs -= n; // record length
		numsys_print_byte_hex(a >> 8); cs -= a >> 8; // address high byte
		numsys_print_byte_hex(a & 0xff); cs -= a & 0xff; // address low byte
		numsys_print_byte_hex(0); // record type (0=data record)
		for (uint16_t i = 0; i < n; i++)
		{
			byte b = MREAD(a + i);
			numsys_print_byte_hex(b);
			cs -= b;
		}
		numsys_print_byte_hex(cs);
		Serial.println();
		a += n;
		if (a == 0) break;
	}
	Serial.println(F(":0000000000"));
}


bool altair_read_intel_hex(uint16_t* start, uint16_t* end)
{
	word aa = 0xffff;
	if (start != NULL) *start = 0xFFFF;
	if (end != NULL) *end = 0x0000;

	while (1)
	{
		// expect line to start with a colon
		int c = -1;
		while (c < 0 || c == 10 || c == 13 || c == ' ' || c == '\t') c = serial_read();
		if (c != ':') return false;

		// initialize checksum
		byte cs = 0;

		// first byte is record length
		byte n = numsys_read_hex_byte(); cs -= n;

		// next two bytes are address
		byte d = numsys_read_hex_byte(); cs -= d;
		word a = numsys_read_hex_byte(); cs -= a;
		a = a + 256 * d;

		// next byte is record type
		byte r = numsys_read_hex_byte(); cs -= r;

		switch (r)
		{
		case 0:
		{
			// data record

			if (a != aa && n > 0)
			{
				if (aa != 0xffff) { numsys_print_word(aa - 1); Serial.println(); }
				numsys_print_word(a);
				aa = a;
			}
			aa += n;
			Serial.print('.');

			if (n > 0 && start != NULL && a < *start) *start = a;
			for (byte i = 0; i < n; i++)
			{
				d = numsys_read_hex_byte();
				MWRITE(a, d);
				cs -= d;
				a++;
			}
			if (n > 0 && end != NULL && a - 1 > *end) *end = a - 1;

			break;
		}

		default:
		{
			// unhandled record => skip
			for (byte i = 0; i < n; i++) cs += numsys_read_hex_byte();
			break;
		}
		}

		// test checksum
		byte csb = numsys_read_hex_byte();
		if (cs != csb)
			return false; // checksum error

		  // empty record means we're done
		if (n == 0)
		{
			numsys_print_word(aa - 1);
			Serial.println();
			return true;
		}
	}
}


void read_data()
{
	uint16_t addr, len;
	byte b;
	Serial.print(F("\r\n\nStart address: "));
	if (numsys_read_word(&addr))
	{
		Serial.print(F("\r\nNumber of bytes: "));
		if (numsys_read_word(&len))
		{
			Serial.print(F("\r\nData: "));
			while (len > 0)
			{
				Serial.write(' ');
				if (!numsys_read_byte(&b)) return;
				MWRITE(addr, (byte)b);
				++addr;
				--len;
			}
		}
	}

	Serial.println();
}

void empty_input_buffer()
{
	while (true)
	{
		if (serial_read() < 0) { delay(15); if (serial_read() < 0) { delay(150); if (serial_read() < 0) break; } }
	}
}


void read_inputs_serial()
{
	bool b_enabled = false;

	if (!config_serial_input_enabled())
		return;

	int data = serial_read();
	if (data < 0)
		return;
#if STANDALONE>0
	else if (data >= '0' && data <= '9')
		dswitch = dswitch ^ (1 << (data - '0'));
	else if (data >= 'a' && data <= 'f')
		dswitch = dswitch ^ (1 << (data - 'a' + 10));
	else if (data == '/')
	{
		Serial.print(F("\r\nSet Addr switches to: "));
		numsys_read_word(&dswitch);
		Serial.println('\n');
	}
#endif
	else if (data == '?')
		print_help_menu();
	else if (data == 'X')
		cswitch |= BIT(SW_EXAMINE);
	else if (data == 'x')
		cswitch |= BIT(SW_EXNEXT);
	else if (data == 'P')
		cswitch |= BIT(SW_DEPOSIT);
	else if (data == 'p')
		cswitch |= BIT(SW_DEPNEXT);
	else if (data == 'r')
		cswitch |= BIT(SW_RUN);
	else if (data == 'o' || data == 27)
		cswitch |= BIT(SW_STOP);
	else if (data == 't')
		cswitch |= BIT(SW_STEP);
	else if (data == 'R')
		cswitch |= BIT(SW_RESET);
	else if (data == '!')
		cswitch |= BIT(SW_RESET) | BIT(SW_STOP);
	else if (data == 'U')
	{
		if (config_b_mode())
			cswitch |= BIT(SW_EXAMINE);
		cswitch |= BIT(SW_AUX1UP);
	}
	else if (data == 'u')
	{
		if (config_b_mode())
			cswitch |= BIT(SW_EXAMINE);
		cswitch |= BIT(SW_AUX1DOWN);
	}
	else if (data == '>')
	{
		Serial.print(F("\r\nRun from address: "));
		if (numsys_read_word(&regPC))
		{
			regPC--;
			p_regPC = ~regPC;
			if (config_serial_debug_enabled())
				Serial.print(F("\r\n\n--- RUNNING (press ESC twice to stop) ---\r\n\n"));
			Serial.println();
			host_clr_status_led_WAIT();
			host_clr_status_led_PROT();
		}
		else
			Serial.println();
	}
	else if (data == 's')
	{
		//B Mode workaround
		if (config_b_mode())
		{
			b_enabled = true;
			set_b_mode(false);
		}
		Serial.print(F("\r\nCapture from "));
		byte dev = read_device();                                                                                            
		if (dev < 0xff)
		{
			cswitch |= BIT(SW_AUX2UP);
			dswitch = dev == 0x80 ? 0 : 0x8000 | (dev << 13);
			if (!serial_capture_running(dev))
			{
				Serial.print(F(" File #: "));
				if (numsys_read_byte(&dev))
					dswitch |= dev;
				else
					cswitch = 0;
			}
		}
		Serial.println();
		//B Mode workaround
		set_b_mode(b_enabled);
	}
	else if (data == 'l')
	{
		//B Mode workaround
		if (config_b_mode())
		{
			b_enabled = true;
			set_b_mode(false);
		}
		Serial.print(F("\r\nReplay to "));
		byte dev = read_device();
		if (dev < 0xff)
		{
			cswitch |= BIT(SW_AUX2DOWN);

			// if the replay device is mapped to the primary host console
			// then we must run immediately because otherwise the debugging
				// console will consume the replayed characters
			if (dev == 0x80 || config_serial_map_sim_to_host(dev) == config_host_serial_primary())
				cswitch |= BIT(SW_RUN);

			dswitch = dev == 0x80 ? 0 : 0x8000 | (dev << 13);
			if (!serial_replay_running(dev))
			{
				char c;
				Serial.print(F(" (e)xample/(f)ile: "));
				do { c = serial_read(); } while (c != 'e' && c != 'f');
				if (c == 'f') dswitch |= 0x100;
				Serial.print(c);
				Serial.print(c == 'f' ? F(" File #: ") : F(" Example #: "));
				if (numsys_read_byte(&dev))
					dswitch |= dev;
				else
					cswitch = 0;
			}
		}
		Serial.println();
		//B Mode workaround
		set_b_mode(b_enabled);
	}
#if NUM_DRIVES>0 || NUM_CDRIVES>0 || NUM_TDRIVES>0 || NUM_HDSK_UNITS>0
	else if (data == 'm')
	{
		char c = 0;
#if NUM_DRIVES==0 && NUM_TDRIVES==0 && NUM_CDRIVES==0
		c = 'h';
#elif NUM_DRIVES==0 && NUM_HDSK_UNITS==0 && NUM_CDRIVES==0
		c = 't';
#elif NUM_TDRIVES==0 && NUM_HDSK_UNITS==0 && NUM_CDRIVES==0
		c = 'f';
#elif NUM_DRIVES==0 && NUM_HDSK_UNITS==0 && NUM_TDRIVES==0
		c = 'c';
#else
		Serial.print(F("\r\nMount "));
#if NUM_DRIVES>0
		Serial.print(F("(f)loppy "));
#endif
#if NUM_CDRIVES>0
		Serial.print(F("(c)romemco floppy "));
#endif
#if NUM_TDRIVES>0
		Serial.print(F("(t)arbell floppy "));
#endif
#if NUM_HDSK_UNITS>0
		Serial.print(F("(h)ard disk "));
#endif
		Serial.print(F("? "));
#endif
		if (c == 0)
		{
			do { c = serial_read(); } while (c != 'f' && c != 'F' && c != 'h' && c != 'H' && c != 't' && c != 'T' && c != 'c' && c != 'C' && c != 27);
			if (c != 27) { Serial.print(c); Serial.println(' '); }
		}

		if (c == 27)
			Serial.println();
		else
		{
			bool ok = false;
#if NUM_DRIVES>0
			if ((c == 'f' || c == 'F'))
			{
				byte b;
				Serial.print(F("Floppy drive number (0-"));
				Serial.print(NUM_DRIVES - 1);
				Serial.print(F("): "));
				if (numsys_read_byte(&b) && b < NUM_DRIVES)
				{
					dswitch = 0x1000 | (b << 8); ok = true;
				}
			}
#endif
#if NUM_CDRIVES>0
			if ((c == 'c' || c == 'C'))
			{
				byte b;
				Serial.print(F("Cromemco floppy drive number (0-"));
				Serial.print(NUM_CDRIVES - 1);
				Serial.print(F("): "));
				if (numsys_read_byte(&b) && b < 4)
				{
					dswitch = 0x7000 | (b << 8); ok = true;
				}
			}
#endif
#if NUM_TDRIVES>0
			if ((c == 't' || c == 'T'))
			{
				byte b;
				Serial.print(F("Tarbell floppy drive number (0-"));
				Serial.print(NUM_TDRIVES - 1);
				Serial.print(F("): "));
				if (numsys_read_byte(&b) && b < 4)
				{
					dswitch = 0x5000 | (b << 8); ok = true;
				}
			}
#endif
#if NUM_HDSK_UNITS>0
			if ((c == 'h' || c == 'H'))
			{
#if NUM_HDSK_UNITS==1
				c = '1';
#else
				Serial.print(F("Hard disk unit number (1-"));
				Serial.print(NUM_HDSK_UNITS);
				Serial.print(F("): "));
				do { c = serial_read(); } while ((c < '1' || c>'0' + NUM_HDSK_UNITS) && c != 27);
				if (c != 27) { Serial.print(c); Serial.print(' '); }
#endif
				if (c != 27)
				{
					dswitch = 0x3000 | ((c - '1') << 10);

					Serial.print(F("Platter number (0-3): "));
					do { c = serial_read(); } while ((c < '0' || c>'3') && c != 27);
					if (c != 27)
					{
						Serial.print(c); dswitch |= (c - '0') << 8; ok = true;
					}
				}
			}
#endif

			if (ok)
			{
				byte b;
				Serial.print(F(" Image number: "));
				if (numsys_read_byte(&b))
				{
					dswitch |= b;
					cswitch |= BIT(SW_AUX2DOWN);
				}
				else
					dswitch = 0;
			}
			else
				dswitch = 0;

			Serial.println();
		}
	}
#endif
	else if (data == 'Q')
		cswitch |= BIT(SW_PROTECT);
	else if (data == 'q')
		cswitch |= BIT(SW_UNPROTECT);
	else if (data == 'D')
	{
		bool go;
		uint16_t addr;
		Serial.print(F("Start address: "));
		go = numsys_read_word(&addr);
		Serial.println();
		while (go)
		{
			numsys_print_word(addr);
			Serial.print(F(": "));
			addr += disassemble(Mem, addr, true);
			Serial.println();
			while (!serial_available()) delay(100);
			go = (serial_read() == 0x20);
		}
		p_regPC = ~regPC;
		print_dbg_info();
	}
	else if (data == 'M')
	{
		bool go;
		uint16_t addr;
		Serial.print(F("Start address: "));
		go = numsys_read_word(&addr);
		Serial.println();
		while (go)
		{
			numsys_print_word(addr);
			Serial.print(F(": "));
			numsys_print_mem(addr, 8, false);
			Serial.print(F("  "));
			numsys_print_mem(addr + 8, 8, false);
			Serial.print(F("   "));
			for (int i = 0; i < 16; i++)
			{
				if (i == 8) Serial.print(' ');
				Serial.write(MREAD(addr + i) < 32 || MREAD(addr + i) > 126 ? '.' : MREAD(addr + i));
			}
			Serial.println();
			while (!serial_available()) delay(100);
			addr += 16;
			go = (serial_read() == 0x20);
		}
		p_regPC = ~regPC;
		print_dbg_info();
	}
	else if (data == 'H')
	{
		dswitch = 0x40;
		cswitch = BIT(SW_AUX1DOWN);
	}
	else if (data == 'h')
	{
		uint16_t start, end;
		Serial.print(F("Start address: "));
		if (numsys_read_word(&start))
		{
			Serial.print(F("\r\nEnd address: "));
			if (numsys_read_word(&end))
			{
				Serial.println();
				altair_dump_intel_hex(start, end);
			}
		}
		Serial.println();
		p_regPC = ~regPC;
		print_dbg_info();
	}
	else if (data == 'L')
	{
		read_data();
		empty_input_buffer();
		p_regPC = ~regPC;
		print_dbg_info();
	}
	else if (data == 'n')
	{
		numsys_toggle();
		p_regPC = ~regPC;
		print_dbg_info();
	}
	else if (data == 'Y')
		mem_print_layout();
	else if (data == 'C')
	{
		cswitch = BIT(SW_STOP) | BIT(SW_AUX1UP);
	}
	else if (data == 'F')
	{
		byte i, b, len, data[10];
		Serial.print(F("\r\nNumber of bytes (10 max): "));
		if (numsys_read_byte(&len) && len > 0)
		{
			Serial.print(F("\r\nData: "));
			for (i = 0; i < len; i++)
			{
				Serial.write(' ');
				if (!numsys_read_byte(&b)) break;
				data[i] = b;
			}

			Serial.print(F("\r\nFound at: "));
			if (i == len)
			{
				for (uint16_t a = 0; a < MEMSIZE - len; a++)
				{
					for (i = 0; i < len && Mem[a + i] == data[i]; i++);
					if (i == len) { numsys_print_word(a); Serial.print(' '); }
				}
			}
			Serial.println(F(" --  done\n"));
		}
	}
#if MAX_BREAKPOINTS>0
	else if (data == 'B' || data == 'V')
	{
		if (data == 'B')
		{
			if (numBreakpoints < MAX_BREAKPOINTS)
			{
				uint16_t a;
				Serial.print(F("\r\nAdd breakpoint at: "));
				if (numsys_read_word(&a)) breakpoint_add(a);
			}
			else
				Serial.print(F("\r\nToo many breakpoints!"));
		}
		else if (numBreakpoints > 0)
			breakpoint_remove_last();

		Serial.print(F("\r\nBreakpoints at: "));
		breakpoint_print();
		Serial.println('\n');
	}
#endif
}


void print_panel_serial(bool force)
{
	byte dbus;
	static uint16_t p_dswitch = 0, p_cswitch = 0, p_abus = 0xffff, p_dbus = 0xffff, p_status = 0xffff;
	uint16_t status, abus;

	if (!config_serial_panel_enabled())
		return;

	status = host_read_status_leds();
	abus = host_read_addr_leds();
	dbus = host_read_data_leds();

	if (force || p_cswitch != cswitch || p_dswitch != dswitch || p_abus != abus || p_dbus != dbus || p_status != status)
	{
		Serial.print(F("\033[s\033[0;0HINTE PROT MEMR INP M1 OUT HLTA STACK WO INT  D7  D6  D5  D4  D3  D2  D1  D0\r\n"));

		if (status & ST_INTE) Serial.print(F(" *  "));    else Serial.print(F(" .  "));
		if (status & ST_PROT) Serial.print(F("  *  "));   else Serial.print(F("  .  "));
		if (status & ST_MEMR) Serial.print(F("  *  "));   else Serial.print(F("  .  "));
		if (status & ST_INP) Serial.print(F("  * "));    else Serial.print(F("  . "));
		if (status & ST_M1) Serial.print(F(" * "));     else Serial.print(F(" . "));
		if (status & ST_OUT) Serial.print(F("  * "));    else Serial.print(F("  . "));
		if (status & ST_HLTA) Serial.print(F("  *  "));   else Serial.print(F("  .  "));
		if (status & ST_STACK) Serial.print(F("   *  "));  else Serial.print(F("   .  "));
		if (status & ST_WO) Serial.print(F(" * "));     else Serial.print(F(" . "));
		if (status & ST_INT) Serial.print(F("  *"));    else Serial.print(F("  ."));

		if (dbus & 0x80)   Serial.print(F("   *")); else Serial.print(F("   ."));
		if (dbus & 0x40)   Serial.print(F("   *")); else Serial.print(F("   ."));
		if (dbus & 0x20)   Serial.print(F("   *")); else Serial.print(F("   ."));
		if (dbus & 0x10)   Serial.print(F("   *")); else Serial.print(F("   ."));
		if (dbus & 0x08)   Serial.print(F("   *")); else Serial.print(F("   ."));
		if (dbus & 0x04)   Serial.print(F("   *")); else Serial.print(F("   ."));
		if (dbus & 0x02)   Serial.print(F("   *")); else Serial.print(F("   ."));
		if (dbus & 0x01)   Serial.print(F("   *")); else Serial.print(F("   ."));
		Serial.print(("\r\nWAIT HLDA   A15 A14 A13 A12 A11 A10  A9  A8  A7  A6  A5  A4  A3  A2  A1  A0\r\n"));
		if (status & ST_WAIT) Serial.print(F(" *  "));   else Serial.print(F(" .  "));
		if (status & ST_HLDA) Serial.print(F("  *   ")); else Serial.print(F("  .   "));
		if (abus & 0x8000) Serial.print(F("   *")); else Serial.print(F("   ."));
		if (abus & 0x4000) Serial.print(F("   *")); else Serial.print(F("   ."));
		if (abus & 0x2000) Serial.print(F("   *")); else Serial.print(F("   ."));
		if (abus & 0x1000) Serial.print(F("   *")); else Serial.print(F("   ."));
		if (abus & 0x0800) Serial.print(F("   *")); else Serial.print(F("   ."));
		if (abus & 0x0400) Serial.print(F("   *")); else Serial.print(F("   ."));
		if (abus & 0x0200) Serial.print(F("   *")); else Serial.print(F("   ."));
		if (abus & 0x0100) Serial.print(F("   *")); else Serial.print(F("   ."));
		if (abus & 0x0080) Serial.print(F("   *")); else Serial.print(F("   ."));
		if (abus & 0x0040) Serial.print(F("   *")); else Serial.print(F("   ."));
		if (abus & 0x0020) Serial.print(F("   *")); else Serial.print(F("   ."));
		if (abus & 0x0010) Serial.print(F("   *")); else Serial.print(F("   ."));
		if (abus & 0x0008) Serial.print(F("   *")); else Serial.print(F("   ."));
		if (abus & 0x0004) Serial.print(F("   *")); else Serial.print(F("   ."));
		if (abus & 0x0002) Serial.print(F("   *")); else Serial.print(F("   ."));
		if (abus & 0x0001) Serial.print(F("   *")); else Serial.print(F("   ."));
		Serial.print(F("\r\n            S15 S14 S13 S12 S11 S10  S9  S8  S7  S6  S5  S4  S3  S2  S1  S0\r\n"));
		Serial.print(F("          "));
		if (dswitch & 0x8000) Serial.print(F("   ^")); else Serial.print(F("   v"));
		if (dswitch & 0x4000) Serial.print(F("   ^")); else Serial.print(F("   v"));
		if (dswitch & 0x2000) Serial.print(F("   ^")); else Serial.print(F("   v"));
		if (dswitch & 0x1000) Serial.print(F("   ^")); else Serial.print(F("   v"));
		if (dswitch & 0x0800) Serial.print(F("   ^")); else Serial.print(F("   v"));
		if (dswitch & 0x0400) Serial.print(F("   ^")); else Serial.print(F("   v"));
		if (dswitch & 0x0200) Serial.print(F("   ^")); else Serial.print(F("   v"));
		if (dswitch & 0x0100) Serial.print(F("   ^")); else Serial.print(F("   v"));
		if (dswitch & 0x0080) Serial.print(F("   ^")); else Serial.print(F("   v"));
		if (dswitch & 0x0040) Serial.print(F("   ^")); else Serial.print(F("   v"));
		if (dswitch & 0x0020) Serial.print(F("   ^")); else Serial.print(F("   v"));
		if (dswitch & 0x0010) Serial.print(F("   ^")); else Serial.print(F("   v"));
		if (dswitch & 0x0008) Serial.print(F("   ^")); else Serial.print(F("   v"));
		if (dswitch & 0x0004) Serial.print(F("   ^")); else Serial.print(F("   v"));
		if (dswitch & 0x0002) Serial.print(F("   ^")); else Serial.print(F("   v"));
		if (dswitch & 0x0001) Serial.print(F("   ^")); else Serial.print(F("   v"));
		Serial.print(F("\r\n            Stop  Step  Examine  Deposit  Reset  Protect   Aux  Aux\r\n"));
		Serial.print(F("           "));
		if (cswitch & BIT(SW_STOP))    Serial.print(F("  ^ "));       else if (cswitch & BIT(SW_RUN))       Serial.print(F("  v "));      else Serial.print(F("  o "));
		if (cswitch & BIT(SW_STEP))    Serial.print(F("    ^ "));     else if (cswitch & BIT(SW_SLOW))      Serial.print(F("    v "));    else Serial.print(F("    o "));
		if (cswitch & BIT(SW_EXAMINE)) Serial.print(F("     ^   "));  else if (cswitch & BIT(SW_EXNEXT))    Serial.print(F("     v   ")); else Serial.print(F("     o   "));
		if (cswitch & BIT(SW_DEPOSIT)) Serial.print(F("     ^   "));  else if (cswitch & BIT(SW_DEPNEXT))   Serial.print(F("     v   ")); else Serial.print(F("     o   "));
		if (cswitch & BIT(SW_RESET))   Serial.print(F("    ^  "));    else if (cswitch & BIT(SW_CLR))       Serial.print(F("    v  "));   else Serial.print(F("    o  "));
		if (cswitch & BIT(SW_PROTECT)) Serial.print(F("      ^  "));  else if (cswitch & BIT(SW_UNPROTECT)) Serial.print(F("      v  ")); else Serial.print(F("      o  "));
		if (cswitch & BIT(SW_AUX1UP))  Serial.print(F("     ^  "));   else if (cswitch & BIT(SW_AUX1DOWN))  Serial.print(F("     v  "));  else Serial.print(F("     o  "));
		if (cswitch & BIT(SW_AUX2UP))  Serial.print(F("  ^  "));      else if (cswitch & BIT(SW_AUX2DOWN))  Serial.print(F("  v  "));     else Serial.print(F("  o  "));
		Serial.print(F("\r\n            Run         E.Next   D.Next    CLR   Unprotect\r\n\033[K\n\033[K\n\033[K\n\033[K\n\033[K\033[u"));
		p_cswitch = cswitch;
		p_dswitch = dswitch;
		p_abus = abus;
		p_dbus = dbus;
		p_status = status;
	}
}


void print_dbg_info()
{
	if (config_serial_debug_enabled() && host_read_status_led_WAIT() && regPC != p_regPC)
		cpu_print_registers();
}



void reset(bool resetPC)
{
	host_clr_status_led_INT();
	host_set_status_led_WO();
	host_clr_status_led_STACK();
	host_clr_status_led_HLTA();
	host_clr_status_led_OUT();
	host_clr_status_led_M1();
	host_clr_status_led_INP();
	host_clr_status_led_MEMR();
	host_clr_status_led_PROT();
	serial_update_hlda_led();
	altair_interrupt_disable();

	if (resetPC)
	{
		uint16_t a = mem_get_rom_autostart_address();
		regPC = (a == 0xffff) ? 0x0000 : a;
		p_regPC = ~regPC;
	}

	serial_reset();

#if STANDALONE>0
	if (cswitch & BIT(SW_STOP))
#else
	if (host_read_function_switch(SW_STOP))
#endif
	{
		have_ps2 = false;

		// close all open files
		serial_close_files();
	}

	// remove ROMS that were temporarily installed by the Simulator
	// and restore auto-disable ROMS
	mem_reset_roms();

	altair_interrupts = 0;
	altair_interrupts_buf = 0;
	tdrive_reset();
	cdrive_reset();
}


void switch_interrupt_handler()
{
	if (altair_interrupts & INT_SW_STOP)
	{
		altair_interrupts &= ~INT_SW_STOP;
		host_clr_status_led_MEMR();
		host_clr_status_led_WO();
		host_set_status_led_WAIT();
		if (!config_serial_debug_enabled())
		{
		}
		else if (config_serial_panel_enabled())
			Serial.print(F("\033[2J\033[14B\r\n------ STOP ------\r\n\n"));
		else
			Serial.print(F("\r\n\n------ STOP ------\r\n\n"));
		p_regPC = ~regPC;
	}
	else if (altair_interrupts & INT_SW_RESET)
	{
		altair_wait_reset();
		reset(true);
	}
	else
	{
		cswitch = 0;
		if (altair_interrupts & INT_SW_CLR) { cswitch |= BIT(SW_CLR); altair_interrupts &= ~INT_SW_CLR; }
		if (altair_interrupts & INT_SW_AUX2UP) { cswitch |= BIT(SW_AUX2UP); altair_interrupts &= ~INT_SW_AUX2UP; }
		if (altair_interrupts & INT_SW_AUX2DOWN) { cswitch |= BIT(SW_AUX2DOWN); altair_interrupts &= ~INT_SW_AUX2DOWN; }
#if STANDALONE==0
		dswitch = host_read_addr_switches();
#endif
		process_inputs();
	}
}



void altair_rtc_interrupt()
{
	altair_interrupt(INT_RTC);
}


void rtc_setup()
{
	float rate = config_rtc_rate();
	altair_rtc_available = rate > 0.0;

	if (!altair_rtc_available)
	{
		timer_stop(TIMER_RTC);
		altair_rtc_running = false;
		//printf("RTC disabled\n");
	}
	else
	{
		timer_setup(TIMER_RTC, (uint32_t)(1000000.0 / rate), altair_rtc_interrupt);
		if (altair_rtc_running) timer_start(TIMER_RTC, 0, true);
		//printf("RTC enabled: %f %i\n", rate, (int) (1000000.0/rate));
	}
}


void altair_vi_check_interrupt()
{
	byte level = 0xff;

	if (altair_interrupts_buf & config_interrupt_vi_mask[0]) level = 0;
	else if (altair_interrupts_buf & config_interrupt_vi_mask[1]) level = 1;
	else if (altair_interrupts_buf & config_interrupt_vi_mask[2]) level = 2;
	else if (altair_interrupts_buf & config_interrupt_vi_mask[3]) level = 3;
	else if (altair_interrupts_buf & config_interrupt_vi_mask[4]) level = 4;
	else if (altair_interrupts_buf & config_interrupt_vi_mask[5]) level = 5;
	else if (altair_interrupts_buf & config_interrupt_vi_mask[6]) level = 6;
	else if (altair_interrupts_buf & config_interrupt_vi_mask[7]) level = 7;

	if (level < altair_vi_level_cur)
	{
		altair_vi_level = level;
		altair_interrupts |= INT_VECTOR;
	}
}


void altair_vi_out_control(byte port, byte data)
{
	// set current interrupt level for vector interrupts
	altair_vi_level_cur = (data & 8) == 0 ? 0xff : (7 - (data & 7));

	// reset interrupt line from RTC 
	if (data & 0x10)
		altair_interrupt(INT_RTC, false);

	// enable/disable real-time clock
	if (((data & 0x40) && !altair_rtc_running && altair_rtc_available))
	{
		altair_rtc_running = true;
		timer_start(TIMER_RTC, 0, true);
	}
	else if (!(data & 0x40) && altair_rtc_running)
	{
		altair_rtc_running = false;
		timer_stop(TIMER_RTC);
	}

	if (altair_interrupts_enabled)
		altair_vi_check_interrupt();
}


void altair_vi_register_ports()
{
	io_register_port_out(0xfe, config_have_vi() ? altair_vi_out_control : NULL);
}


void altair_interrupt_enable()
{
	host_set_status_led_INTE();
	altair_interrupts_enabled = true;

	if (config_have_vi())
		altair_vi_check_interrupt();
	else
		altair_interrupts = (altair_interrupts & ~INT_DEVICE) | (altair_interrupts_buf & config_interrupt_mask);
}


bool altair_interrupt_enabled()
{
	return altair_interrupts_enabled;
}


void altair_interrupt_disable()
{
	host_clr_status_led_INTE();
	altair_interrupts_enabled = false;
	altair_interrupts &= ~INT_DEVICE;
}


void altair_interrupt(uint32_t i, bool set)
{
	if (i & INT_DEVICE)
	{
		if (set)
			altair_interrupts_buf |= (i & INT_DEVICE);
		else
			altair_interrupts_buf &= ~(i & INT_DEVICE);

		if (!altair_interrupts_enabled)
			altair_interrupts &= ~INT_DEVICE;
		else if (config_have_vi())
			altair_vi_check_interrupt();
		else
			altair_interrupts = (altair_interrupts & ~INT_DEVICE) | (altair_interrupts_buf & config_interrupt_mask);
	}

	if (i & INT_SWITCH)
	{
		altair_interrupts |= (i & INT_SWITCH);
		if (i & INT_SW_STOP) sticky_slow = false;
	}
}


bool altair_interrupt_active(uint32_t i)
{
	return (altair_interrupts_buf & i) != 0;
}


static byte altair_interrupt_handler()
{
	byte opcode = 0xff;

	host_set_status_led_M1();
	host_set_status_led_INT();
	host_clr_status_led_MEMR();

	// Determine the opcode to put on the data bus (if VI enabled).
	// Must do this before calling altair_interrupt_disable, otherwise
	// the INT_VECTOR flag is cleared already
	if (altair_interrupts & INT_VECTOR)
	{
		opcode = 0307 | (altair_vi_level * 8);

		// the VI interrupt output line gets cleared as soon as the 
		// CPU acknowledges the interrupt
		altair_interrupts &= ~INT_VECTOR;
	}

	// disable interrupts now
	altair_interrupt_disable();

	if (host_read_status_led_WAIT())
	{
		if (config_serial_debug_enabled()) { Serial.print(F("\r\nInterrupt! opcode=")); numsys_print_byte(opcode); Serial.println(); }
		altair_set_outputs(regPC, opcode);
		altair_wait_step();
	}
	else
		host_set_data_leds(opcode);

	host_clr_status_led_HLTA();
	host_clr_status_led_INT();
	host_clr_status_led_M1();

	return opcode;
}


bool altair_isreset()
{
	return (cswitch & BIT(SW_RESET)) == 0;
}


void altair_hlt()
{
	host_set_status_led_HLTA();
	sticky_slow = false;

#if STANDALONE>0
	// in standalone mode it is hard to interact with the panel so for a HLT
	// instruction we just stop the CPU to avoid confusion
	regPC--;
	altair_interrupt(INT_SW_STOP);
#else
	if (!host_read_status_led_WAIT())
	{
		host_set_addr_leds(0xffff);
		host_set_data_leds(0xff);
		host_set_status_led_MEMR();
		host_set_status_led_WAIT();
		altair_interrupts = 0;
		while ((altair_interrupts & (~INT_SWITCH | INT_SW_RESET)) == 0)
		{
			// check host interrupts (e.g. switches on Mega)
			host_check_interrupts();

			// advance simulation time (so timers can expire)
			TIMER_ADD_CYCLES(2);
		}

		if (altair_interrupts & INT_SW_RESET)
		{
			cswitch = 0;
			while (!(cswitch & BIT(SW_RESET))) read_inputs_panel();
		}
		else if (altair_interrupts & INT_DEVICE)
		{
			host_clr_status_led_WAIT();
			host_clr_status_led_PROT();
		}
	}
	else
	{
		host_set_status_led_MEMR();
		altair_set_outputs(0xffff, 0xff);

		altair_interrupts = 0;
		while ((altair_interrupts & (~INT_SWITCH | INT_SW_RESET)) == 0)
		{
			read_inputs_panel();
			print_panel_serial();
			host_check_interrupts();

			// advance simulation time (so timers can expire)
			TIMER_ADD_CYCLES(1);
		}
	}
#endif
}


byte altair_read_sense_switches(byte port)
{
	byte data;

	// read SENSE switches
#if STANDALONE>0
	data = dswitch / 256;
#else
	static unsigned long debounceTimeout = 0;
	static byte debounceVal = 0;
	if (millis() > debounceTimeout)
	{
		data = host_read_sense_switches();
		debounceVal = data;
		debounceTimeout = millis() + 20;
	}
	else
		data = debounceVal;
#endif

	return data;
}


void altair_out(byte port, byte data)
{
	host_set_addr_leds(port << 8 | port);

#if USE_IO_BUS>0
	// When using the I/O bus we must put the output data
	// onto the data LED outputs so devices attached to
	// the bus can read it
	host_set_data_leds(data);

	// do this multiple more times to wait ~200ns for
	// address/data signals on bus to settle (NOPs on
	// ARM32 are not necessarily time consuming but this is)
	host_set_data_leds(data);
	host_set_data_leds(data);
	host_set_data_leds(data);
	host_set_data_leds(data);
#else
	// The S-100 bus on the real Altair has separate data
	// buses for data in (to CPU) and data out (from CPU).
	// The data LEDs on the front panel are connected to
	// the "data in" lines. For output operations those
	// lines are in high-Z state and therefore the LEDs
	// are all on regardless of the data. 
	// We simulate the original behavior here even though
	// we could just as well show the proper data.
	host_set_data_leds(0xff);
#endif

	host_set_status_led_OUT();
	host_set_status_led_WO();

	io_out(port, data);

	if (host_read_status_led_WAIT())
	{
#if USE_IO_BUS>0
		altair_set_outputs(port | port * 256, data);
#else
		altair_set_outputs(port | port * 256, 0xff);
#endif
		altair_wait_step();
	}

	host_clr_status_led_OUT();
}


byte altair_in(byte port)
{
	byte data = 0;

	host_set_addr_leds(port | port * 256);

	if (host_read_status_led_WAIT())
	{
		cswitch &= BIT(SW_RESET); // clear everything but RESET status

		// keep reading input data while we are waiting
		while (host_read_status_led_WAIT() && (cswitch & (BIT(SW_STEP) | BIT(SW_SLOW) | BIT(SW_RESET))) == 0)
		{
			host_set_status_led_INP();
			data = io_inp(port);
			altair_set_outputs(port | port * 256, data);
			host_clr_status_led_INP();

			read_inputs();

			// advance simulation time (so timers can expire)
			TIMER_ADD_CYCLES(50);
		}

		if (cswitch & BIT(SW_SLOW)) delay(500);
	}
	else
	{
		host_set_status_led_INP();
		data = io_inp(port);
		host_set_data_leds(data);
		host_clr_status_led_INP();
	}

	return data;
}

void setup()
{
	cswitch = 0;
	dswitch = 0;

	drive_get_image_filename(0x99, true);

	Serial.begin(115200);

	timer_setup();
	mem_setup();
	io_setup();
	host_setup();
	filesys_setup();
	drive_setup();
	tdrive_setup();
	cdrive_setup();
	hdsk_setup();
	if (host_read_function_switch(SW_RESET))
		config_setup(-1);
	else if (host_read_function_switch(SW_DEPOSIT))
		config_setup(host_read_addr_switches());
	else
		config_setup(0);
	cpu_setup();
	serial_setup();
	profile_setup();
	rtc_setup();
	printer_setup();
	dazzler_setup();
	vdm1_setup();

	altair_vi_register_ports();
	io_register_port_inp(0xff, altair_read_sense_switches);

	// if RESET switch is held up during powerup then use default configuration settings
	if (host_read_function_switch(SW_RESET))
	{
		// temporarily reset configuration (also calls host_serial_setup)
		config_defaults(true);
		Serial.println(F("Configuration temporarily reset to defaults"));
		Serial.println(F("Raise and hold STOP and then raise AUX1 to enter configuration menu"));
		while (host_read_function_switch(SW_RESET));
		delay(100);
		host_reset_function_switch_state();
	}
	else
	{
		// set up serial connection on the host
		for (byte i = 0; i < HOST_NUM_SERIAL_PORTS; i++)
			host_serial_setup(i, config_host_serial_baud_rate(i), config_host_serial_config(i),
				config_host_serial_primary() == i);
	}

	// if EXAMINE switch is held up during powerup then show host system info
	if (host_read_function_switch(SW_EXAMINE))
		host_system_info();

	// emulator extra: holding down CLR at powerup will keep all registers
	// and memory content initialized with 0. Otherwise (as would be normal 
	// with the Altair), everything is random.
	mem_ram_init(0, MEMSIZE - 1);
	if (!host_read_function_switch(SW_CLR) && !config_clear_memory())
	{
		regPC = host_get_random();
		regSP = host_get_random();
		regA = host_get_random();
		regS = host_get_random() & (PS_CARRY | PS_PARITY | PS_HALFCARRY | PS_ZERO | PS_SIGN);
		regB = host_get_random();
		regC = host_get_random();
		regD = host_get_random();
		regE = host_get_random();
		regH = host_get_random();
		regL = host_get_random();
	}

	host_set_status_led_WAIT();
	reset(false);

	if (config_serial_panel_enabled()) Serial.print(F("\033[2J\033[14B\r\n"));

	uint16_t a = mem_get_rom_autostart_address();
	if (a != 0xFFFF)
	{
		regPC = a;
		host_clr_status_led_WAIT();
	}
}

void loop()
{
	byte opcode;

	// if we are NOT in WAIT mode then enter the main simulation loop
	if (!host_read_status_led_WAIT())
	{
		// clear all switch-related interrupts before starting loop
		altair_interrupts &= ~INT_SWITCH;

		// enable/disable profiling
		profile_enable(config_profiling_enabled());

#if USE_THROTTLE>0
		if (config_throttle() < 0)
		{
			timer_setup(TIMER_THROTTLE, 0, update_throttle);
			timer_start(TIMER_THROTTLE, THROTTLE_TIMER_PERIOD, true);
			throttle_delay = (uint16_t)(10.0 * HOST_PERFORMANCE_FACTOR);
			throttle_micros = micros();
		}
		else
			throttle_delay = (uint16_t)(config_throttle() * HOST_PERFORMANCE_FACTOR);
#endif

		while (true)
		{
			// put PC on address bus LEDs
			host_set_addr_leds(regPC);

			// check for events that can't be handled by real interrupts
			// on the host (e.g. serial input on Mega)
			host_check_interrupts();

			if (altair_interrupts)
			{
				// interrupt detected
				if (altair_interrupts & INT_SWITCH)
				{
					// switch-related interrupt (simulation handling)
					p_regPC = ~regPC;
					switch_interrupt_handler();
					if (host_read_status_led_WAIT())
						break;    // exit simulation loop
					else
						continue; // start over
				}
				else
				{
					// ALTAIR interrupt
					opcode = altair_interrupt_handler();
				}
			}
			else
			{
				// no interrupt => read opcode, put it on data bus LEDs and advance PC
#if USE_REAL_MREAD_TIMING>0
				host_set_status_led_M1();
				opcode = MEM_READ(regPC);
#else
				host_set_status_leds_READMEM_M1();
				host_set_addr_leds(regPC);
				opcode = MREAD(regPC);
				host_set_data_leds(opcode);
#endif
				regPC++;
#if USE_Z80!=0
				// when emulating Z80 we need to increment the R register at each instruction fetch
				regRL++;
#endif
				host_clr_status_led_M1();
			}

			// take a CPU step
			PROFILE_COUNT_OPCODE(opcode);
			CPU_EXEC(opcode);

			// check for breakpoint hit
			breakpoint_check(regPC);

#if USE_THROTTLE>0
			// delay execution according to current delay
			// (need the NOP, otherwise the compiler will optimize the loop away)
			for (uint32_t i = 0; i < throttle_delay; i++) asm("NOP");
#endif
		}

#if USE_THROTTLE>0
		timer_stop(TIMER_THROTTLE);
#endif

		// flush any characters stuck in the serial buffer 
		// (so we don't accidentally execute commands after stopping)
		if (config_serial_input_enabled()) empty_input_buffer();
	}

	if (cswitch & BIT(SW_RESET))
	{
		reset(true);
		read_inputs();
	}

	print_dbg_info();
	p_regPC = regPC;

	host_set_status_led_M1();

	// check for events that can't be handled by real interrupts
	// on the host (e.g. serial input on Mega)
	host_check_interrupts();

	if (altair_interrupts & INT_DEVICE)
	{
		opcode = altair_interrupt_handler();
	}
	else
	{
		opcode = MEM_READ(regPC); regPC++;
	}

#if USE_Z80!=0
	// when emulating Z80 we need to increment the R register at each instruction fetch
	regRL++;
#endif

	host_clr_status_led_M1();
	if (!(cswitch & BIT(SW_RESET)))
	{
		PROFILE_COUNT_OPCODE(opcode);
		CPU_EXEC(opcode);

		// if the PC has not changed (e.g. jump to the same address) then modify p_regPC 
		// to force debugger display (otherwise there would be no feedback)
		if (regPC == p_regPC) p_regPC = ~regPC;

#if STANDALONE>0
		// handle STOP interrupts generated by HLT opcodes in standalone mode
		if (altair_interrupts & INT_SW_STOP) switch_interrupt_handler();
#endif
	}
}

// Prints the help menu in stop mode (ie: serial input/debug menu)
void print_help_menu()
{
	Serial.println();
	Serial.print(F("Mode: ")); print_b_mode(); Serial.println();
	Serial.println();
	Serial.println(F("Run programs"));
	Serial.println(F("[u] Run Prog (sw)   [U] run preset program"));
	Serial.println(F("[C] Configuration"));
	Serial.println();
	Serial.println(F("Altairduino Serial Console / Debugger"));
	Serial.println(F("[>] Run at addr     [n] Octal / Dec / Hex mode"));
	Serial.println(F("[L] Load Memory     [H] Upload Hex File"));
	Serial.println();
	Serial.println(F("Command Switches"));
	Serial.println(F("[X] Examine(sw)     [x] Examine Next"));
	Serial.println(F("[P] Deposit(sw)     [p] Deposit Next"));
	Serial.println(F("[r] Run             [T] Step"));
	Serial.println(F("[R] Reset           [!] Hard Reset"));
	Serial.println(F("[l] Replay          [s] Capture"));
	Serial.println(F("[Q /q] Protect / Unprotect"));
	Serial.println();
	Serial.println(F("Debug / Monitor commands(Space to continue, Enter to end)"));
	Serial.println(F("[M] Examine Memory  [D] Disassemble"));
	Serial.print(F("Command: "));
}
