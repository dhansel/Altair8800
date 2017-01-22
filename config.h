#ifndef CONFIG_H
#define CONFIG_H

// Allowing breakpoints significantly reduces performance but is helpful
// for debugging.  Also uses 2*MAX_BREAKPOINTS+1 bytes of RAM
#define MAX_BREAKPOINTS 0


// Setting this to 1 enables profiling.  To use profiling, enable
// serial debug input (switch 5 on during reset) and then send 'F'
// (without quotes) over serial. From that point, every second a
// message will be printed to serial indicating the number of executed 
// instructions, 8080 cycles, time and performance in MHz.
// Reduces performance and uses 14 bytes of RAM when enabled.
#define USE_PROFILING 1


// If USE_PROFILING is enabled, setting USE_PROFILING_DETAIL to 1 
// will (every 10 seconds) show a list of which opcodes were executed 
// how many times.
// Reduces performance and uses 1k of RAM
#define USE_PROFILING_DETAIL 0


// Enables throttling of CPU speed. This only makes sense to enable
// on the Due since the Mega is too slow anyways and the throttling 
// checks would only reduce performance further.
// if USE_THROTTLE is enabled, USE_PROFILING must be enabled too.
#define USE_THROTTLE 0


// Enables function of the PROTECT/UNPROTECT switches.
// Reduces performance and uses 33 bytes of RAM.
#define USE_PROTECT 1


// Enables support for disk drives. Each drive uses about 160 bytes
// of memory. Set to 0 to completely disable drive support.
#define NUM_DRIVES 0


// If enabled, Address switch state will be set by issuing the '/'
// serial command.  Actual switches will be ignored.
// Useful when operating while not connected to the front panel hardware.
#define STANDALONE 1


// Two slightly different versions of the CPU core implementation are
// provided. Second version was supposed to be significantly faster
// but turns out that there is only a minor difference.
#define CPUCORE_VERSION 2


#endif
