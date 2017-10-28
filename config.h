// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef CONFIG_H
#define CONFIG_H


// Allowing breakpoints significantly reduces performance but is helpful
// for debugging.  Also uses 2*MAX_BREAKPOINTS+1 bytes of RAM
#define MAX_BREAKPOINTS 0


// Setting USE_PROFILING_DETAIL to 1 will (every 10 seconds) show a 
// list of which opcodes were executed how many times (if profiling is enabled).
// Reduces performance and uses 1k of RAM
#define USE_PROFILING_DETAIL 0


// Enables throttling of CPU speed. This only makes sense to enable
// on the Due since the Mega is too slow anyways and the throttling 
// checks would only reduce performance further.
#define USE_THROTTLE 1


// Enables function of the PROTECT/UNPROTECT switches.
// Reduces performance and uses 33 bytes of RAM.
#define USE_PROTECT 1


// Enables support for disk drives. Each drive uses about 160 bytes
// of RAM. Set to 0 to completely disable drive support.
#define NUM_DRIVES 4


// Enables support for hard disk (88-HDSK). Hard disk support uses
// about 1100 bytes of RAM plus 56 bytes for each unit.
// Set to 0 to completely disable hard disk support
#define NUM_HDSK_UNITS 1


// Enables printer emulation which uses about 140 bytes of RAM.
#define USE_PRINTER 1


// Enable two 88-2SIO devices (instead of one).
#define USE_SECOND_2SIO 0


// If enabled, Address switch state will be set by issuing the '/'
// serial command.  Actual switches will be ignored.
// Useful when operating while not connected to the front panel hardware.
#define STANDALONE 0


// ------------------------------------------------------------------------------

#include "Arduino.h"

#define CF_THROTTLE     0x01
#define CF_PROFILE      0x02
#define CF_SERIAL_PANEL 0x04
#define CF_SERIAL_INPUT 0x08
#define CF_SERIAL_DEBUG 0x10
#define CF_CLEARMEM     0x20
#define CF_HAVE_VI      0x40
#define CF_DRIVE_RT     0x80
#define CF_PRINTER_RT   0x00020000
#define CF_HDSK_RT      0x00040000

#define CSM_SIO         0
#define CSM_ACR         1
#define CSM_2SIO1       2
#define CSM_2SIO2       3
#define CSM_2SIO3       4
#define CSM_2SIO4       5

#define CSF_OFF         0
#define CSF_ON          1
#define CSF_AUTO        2

#define CSFB_NONE       0
#define CSFB_UNDERSCORE 1
#define CSFB_RUBOUT     3
#define CSFB_AUTO       2

#define CP_NONE         0
#define CP_OKI          1
#define CP_C700         2

extern uint32_t config_flags;
extern uint32_t config_serial_settings;
extern uint32_t config_interrupt_mask;
extern uint32_t config_interrupt_vi_mask[8];

void config_setup();
void config_edit();
void config_defaults(bool apply);

#if USE_THROTTLE>0
int     config_throttle(); // 0=off, <0=auto delay, >0=manual delay
#else
#define config_throttle() 0
#endif

inline bool config_profiling_enabled()        { return (config_flags & CF_PROFILE)!=0; }
inline bool config_clear_memory()             { return (config_flags & CF_CLEARMEM)!=0; }
inline bool config_serial_panel_enabled()     { return (config_flags & CF_SERIAL_PANEL)!=0; }
inline bool config_serial_input_enabled()     { return (config_flags & CF_SERIAL_INPUT)!=0; }
inline bool config_serial_debug_enabled()     { return (config_flags & CF_SERIAL_DEBUG)!=0; }
inline bool config_have_vi()                  { return (config_flags & CF_HAVE_VI)!=0; }

float    config_rtc_rate();
byte     config_aux1_program();

uint32_t config_host_serial_baud_rate(byte iface);
uint32_t config_host_serial_config(byte iface);
byte     config_host_serial_primary();

byte     config_serial_map_sim_to_host(byte dev); 
bool     config_serial_realtime(byte dev);
uint32_t config_serial_playback_baud_rate(byte dev);
byte     config_serial_playback_example_nuls(byte dev);
byte     config_serial_backspace(byte dev);
byte     config_serial_7bit(byte dev);
byte     config_serial_ucase(byte dev);
bool     config_serial_trap_CLOAD();

byte        config_printer_type();
byte        config_printer_map_to_host_serial();
inline byte config_printer_realtime() { return (config_flags & CF_PRINTER_RT)!=0; }

#endif
