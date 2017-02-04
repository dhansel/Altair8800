#include "config.h"
#include "mem.h"
#include "serial.h"
#include "filesys.h"
#include "numsys.h"

#define  BAUD_110    0
#define  BAUD_150    1
#define  BAUD_300    2
#define  BAUD_600    3
#define  BAUD_1200   4
#define  BAUD_2400   5
#define  BAUD_4800   6
#define  BAUD_9600   7
#define  BAUD_19200  8
#define  BAUD_38400  9
#define  BAUD_57600  10
#define  BAUD_115200 11

// config_flags:
// xxxxxxxx xxxxxxxx xxxxxxxx xxCDIPFT
// T = Throttle
// F = Profile
// P = Serial Panel
// I = Serial Input
// D = Serial Debug
// C = Clear memory on powerup
uint32_t config_flags;


// cofig_serial_settings:
// xxxxxxxx xxxxxxxx xxxxxxxI bbbbBBBB
// BBBB = baud rate for Serial  host interface (see baud rates above)
// bbbb = baud rate for Serial1 host interface (see baud rates above)
// I    = select primary serial interface (0=Serial, 1=Serial1)
// x    = unused
uint32_t config_serial_settings, new_config_serial_settings;


// config_serial_device_settings[0-3]
// xxxxxxxx xxxxxxxx TT77UUMM CNNNBBBB
// BBBB = baud rate for serial playback (see baud rates above)
// NNN  = NULs to send after a carriage return when playing back examples
// C    = trap CLOAD/CSAVE in extended BASIC (for CSM_ACR device only)
// MM   = map device to host interface (00=NONE, 01=primary, 02=secondary)
// UU   = only uppercase for inputs (00=off, 01=on, 10=autodetect)
// 77   = use 7 bit for serial outputs (00=off [use 8 bit], 01=on, 10=autodetect)
// TT   = translate between backspace and _ (00=off, 01=on, 10=autodetect)
uint32_t config_serial_device_settings[4];


// --------------------------------------------------------------------------------


inline uint32_t get_bits(uint32_t v, byte i, byte n)
{
  return (v >> i) & ((1<<n)-1);
}


inline uint32_t set_bits(uint32_t v, byte i, byte n, uint32_t nv)
{
  uint32_t mask = ((1<<n)-1) << i;
  return (v & ~mask) | ((nv << i) & mask);
}


static uint32_t toggle_bits(uint32_t v, byte i, byte n, byte min = 0x00, byte max = 0xff)
{
  byte b = get_bits(v, i, n) + 1;
  return set_bits(v, i, n, b>max ? min : (b<min ? min : b));
}


static uint32_t config_baud_rate(byte b)
{
  uint32_t res;
  switch(b)
    {
    case BAUD_110   : res = 110;    break;
    case BAUD_150   : res = 150;    break;
    case BAUD_300   : res = 300;    break;
    case BAUD_600   : res = 600;    break;
    case BAUD_1200  : res = 1200;   break;
    case BAUD_2400  : res = 2400;   break;
    case BAUD_4800  : res = 4800;   break;
    case BAUD_9600  : res = 9600;   break;
    case BAUD_19200 : res = 19200;  break;
    case BAUD_38400 : res = 38400;  break;
    case BAUD_57600 : res = 57600;  break;
    case BAUD_115200: 
    default         : res = 115200; break;
    }

  return res;
}


byte config_serial_backspace(byte dev)
{
  return get_bits(config_serial_device_settings[dev], 14, 2);
}

byte config_serial_7bit(byte dev)
{
  return get_bits(config_serial_device_settings[dev], 12, 2);
}

byte config_serial_ucase(byte dev)
{
  return get_bits(config_serial_device_settings[dev], 10, 2);
}

bool config_serial_trap_CLOAD()
{
  return get_bits(config_serial_device_settings[CSM_ACR], 7, 1);
}

uint32_t config_serial_playback_baud_rate(byte dev)
{
  return config_baud_rate(get_bits(config_serial_device_settings[dev], 0, 4));
}

byte config_serial_playback_example_nuls(byte dev)
{
  return get_bits(config_serial_device_settings[dev], 4, 3);
}

byte config_serial_map_sim_to_host(byte dev)
{
  switch( get_bits(config_serial_device_settings[dev], 8, 2) )
    {
    case 1: return   config_host_serial_primary();
    case 2: return 1-config_host_serial_primary();
    }

  return 0xff;
}

byte config_host_serial_primary()
{
  return get_bits(config_serial_settings, 8, 1);
}

uint32_t config_host_serial_baud_rate(byte iface)
{
  return config_baud_rate(get_bits(config_serial_settings, iface==0 ? 0 : 4, 4));
}


uint32_t config_host_serial_baud_rate(uint32_t settings, byte iface)
{
  return config_baud_rate(get_bits(settings, iface==0 ? 0 : 4, 4));
}


// --------------------------------------------------------------------------------


static void toggle_host_serial_baud_rate(byte iface)
{
  new_config_serial_settings = toggle_bits(new_config_serial_settings, iface==0 ? 0 : 4, 4, BAUD_1200, BAUD_115200);
}


static void toggle_host_primary_interface()
{
  new_config_serial_settings = toggle_bits(new_config_serial_settings, 8, 1);
}


static uint32_t toggle_serial_flag(uint32_t settings, byte pos, bool haveAuto = true)
{
  byte b = get_bits(settings, pos, 2);
  switch( b )
    {
    case CSF_OFF:  b = CSF_ON;   break;
    case CSF_ON:   b = haveAuto ? CSF_AUTO : CSF_OFF; break;
    case CSF_AUTO: b = CSF_OFF;  break;
    default:       b = haveAuto ? CSF_AUTO : CSF_OFF; break;
    }

  return set_bits(settings, pos, 2, b);
}


static void toggle_flag(uint32_t value)
{
  config_flags = (config_flags & ~value) | (~(config_flags & value) & value);
}


static bool apply_host_serial_settings(uint32_t settings)
{
  config_serial_settings = settings;
  host_serial_setup(0, config_host_serial_baud_rate(0), config_host_serial_primary()==0);
  host_serial_setup(1, config_host_serial_baud_rate(1), config_host_serial_primary()==1);
}


static bool apply_host_serial_settings()
{
  char c;
  uint32_t old_config_serial_settings = config_serial_settings;
  apply_host_serial_settings(new_config_serial_settings);

  Serial.println(F("\n[will revert to old settings if no answer within 20 seconds]"));
  Serial.print(F("Keep new host interface settings (y/n)? "));
  uint32_t timeout = millis() + 20000;
  c = 0;
  do { delay(50); c = serial_read(); } while( c!='y' && c!='n' && millis()<timeout );
  Serial.println(c);
  if( c!='y' ) 
    { 
      apply_host_serial_settings(old_config_serial_settings);
      if( c!='n' ) 
        {
          delay(100);
          Serial.println(F("\nSettings were not confirmed withing 20 seconds."));
          delay(2000);

          // flush serial input that may have accumulated while waiting
          while( serial_read()>=0 );
        }
      return false;
    }

  return true;
}


// --------------------------------------------------------------------------------


static void print_flag(int value)
{
  Serial.print((config_flags&value)!=0 ? F("yes") : F("no"));
}



static void print_serial_device_sim(byte dev)
{
  switch( dev )
    {
    case CSM_SIO:   Serial.print(F("SIO")); break;
    case CSM_ACR:   Serial.print(F("ACR")); break;
    case CSM_2SIO1: Serial.print(F("2-SIO-1")); break;
    case CSM_2SIO2: Serial.print(F("2-SIO-2")); break;
    }
}


static void print_serial_interface_host(byte iface)
{
  switch( iface )
    {
    case 0: Serial.print(F("Serial (USB, pin 0/1)")); break;
    case 1: Serial.print(F("Serial1 (pin 18/19)")); break;
    }
}


static void print_serial_flag(uint32_t settings, byte pos, byte bits = 2)
{
  switch( get_bits(settings, pos, bits) )
    {
    case CSF_OFF:  Serial.print(F("off"));        break;
    case CSF_ON:   Serial.print(F("on"));         break;
    case CSF_AUTO: Serial.print(F("autodetect")); break;
    }
}


static void print_serial_device_mapped_to(uint32_t settings)
{
  switch( get_bits(settings, 8, 2) )
    {
#if defined(__SAM3X8E__)
    case 0: Serial.print("Not mapped"); break;
    case 1: Serial.print("Primary serial host interface"); break;
    case 2: Serial.print("Secondary serial host interface"); break;
#else
    case 0: Serial.print(F("Not mapped")); break;
    case 1: Serial.print(F("Serial")); break;
#endif
    }
}


// --------------------------------------------------------------------------------


static bool save_config(byte fileno)
{
  // better to write all data at once (will overwrite instead
  // of deleting/creating the file)
  byte s = sizeof(uint32_t);
  byte data[6*s];
  memcpy(data, &config_flags, s);
  memcpy(data+4, &new_config_serial_settings, s);
  memcpy(data+8, config_serial_device_settings, 4*s);
  return filesys_write_file('C', fileno, (void *) data, 6*s);
}


static bool load_config(byte fileno)
{
  bool ok = false;
  byte fid = filesys_open_read('C', fileno);
  if( fid )
    {
      byte     s = sizeof(uint32_t);
      uint32_t cur_config_flags = config_flags;
      uint32_t cur_config_serial_settings = new_config_serial_settings;
      uint32_t cur_config_serial_device_settings[4];
      memcpy(cur_config_serial_device_settings, config_serial_device_settings, s*4);
      
      if( filesys_read_data(fid, &config_flags, s)==s )
        if( filesys_read_data(fid, &new_config_serial_settings, s)==s )
          if( filesys_read_data(fid, config_serial_device_settings, 4*s)==4*s )
            ok = true;

      if( !ok )
        {
          config_flags = cur_config_flags;
          config_serial_settings = cur_config_serial_settings;
          memcpy(config_serial_device_settings, cur_config_serial_device_settings, s*4);
        }
      
      filesys_close(fid);
#if STANDALONE>0
      config_flags |= CF_SERIAL_INPUT;      
#endif
    }

  return ok;
}


// --------------------------------------------------------------------------------


void config_edit_serial_device(byte dev)
{
  uint32_t settings = config_serial_device_settings[dev];
  while( true )
    {
      Serial.print(F("\033[2J\033[0;0H\n"));
      Serial.print(F("Configure serial device ")); print_serial_device_sim(dev); Serial.println();
      Serial.print(F("\nMap to host (i)nterface    : ")); print_serial_device_mapped_to(settings); Serial.println();
      Serial.print(F("Playback (b)aud rate       : ")); Serial.println(config_baud_rate(get_bits(settings, 0, 4)));
      Serial.print(F("Example playback (N)ULs    : ")); Serial.println(get_bits(settings, 4, 3));
      Serial.print(F("Use (7) bits               : ")); print_serial_flag(settings, 12); Serial.println();
      Serial.print(F("Serial input (u)ppercase   : ")); print_serial_flag(settings, 10); Serial.println();
      Serial.print(F("Translate (B)ackspace to _ : ")); print_serial_flag(settings, 14); Serial.println();
      if( dev==CSM_ACR )
        { Serial.print(F("Enable CLOAD/CSAVE (t)raps : ")); print_serial_flag(settings, 7, 1); Serial.println(); }

      Serial.println(F("\nE(x)it to main menu"));

      Serial.print(F("\n\nCommand: "));
      while( !serial_available() ) delay(50);
      char c = serial_read();
      if( c>31 && c<127 ) Serial.println(c);

      switch( c )
        {
#if defined(__SAM3X8E__)
        case 'i': settings = toggle_bits(settings, 8, 2, 0, 2); break;
#else
        case 'i': settings = toggle_bits(settings, 8, 2, 0, 1); break;
#endif
        case 'b': settings = toggle_bits(settings, 0, 4, BAUD_110, BAUD_19200); break;
        case 'u': settings = toggle_serial_flag(settings, 10); break;
        case 't': settings = toggle_bits(settings, 7, 1); break;
        case '7': settings = toggle_serial_flag(settings, 12); break;
        case 'B': settings = toggle_serial_flag(settings, 14); break;
        case 'N': settings = toggle_bits(settings, 4, 3); break;

        case 27:
        case 'x': 
          config_serial_device_settings[dev] = settings;
          serial_timer_interrupt_setup(dev);
          return;
        }
    }
}


// --------------------------------------------------------------------------------


void config_edit()
{
  new_config_serial_settings = config_serial_settings;

  // flush serial input
  while( serial_read()>=0 );

  while( true )
    {
      char c;
      Serial.print(F("\033[2J\033[0;0H\n"));
      
#if USE_PROFILING>0
      Serial.print(F("Enable pro(f)iling          : ")); print_flag(CF_PROFILE); Serial.println();
#endif
#if USE_THROTTLE>0
      Serial.print(F("Enable (t)hrottling         : ")); print_flag(CF_THROTTLE); Serial.println();
#endif
      Serial.print(F("Enable serial (p)anel       : ")); print_flag(CF_SERIAL_PANEL); Serial.println();
#if STANDALONE==0
      Serial.print(F("Enable serial (i)nput       : ")); print_flag(CF_SERIAL_INPUT); Serial.println();
#endif
      Serial.print(F("Enable serial (d)ebug       : ")); print_flag(CF_SERIAL_DEBUG); Serial.println();
      Serial.print(F("(c)lear memory on powerup   : ")); print_flag(CF_CLEARMEM); Serial.println();


#if !defined(_WIN32)
      Serial.print(F("Host Serial (b)aud rate     : ")); Serial.print(config_host_serial_baud_rate(new_config_serial_settings, 0));
      if( config_host_serial_baud_rate(new_config_serial_settings, 0) != config_host_serial_baud_rate(0) )
        {
          Serial.print(F(" (current: ")); 
          Serial.print(config_host_serial_baud_rate(0));
          Serial.print(')');
        }
      Serial.println();
#endif

#if defined(__SAM3X8E__)
      Serial.print(F("Host Serial1 baud (r)ate    : ")); Serial.print(config_host_serial_baud_rate(new_config_serial_settings, 1));
      if( config_host_serial_baud_rate(new_config_serial_settings, 1) != config_host_serial_baud_rate(1) )
        {
          Serial.print(F(" (current: ")); 
          Serial.print(config_host_serial_baud_rate(1));
          Serial.print(')');
        }
      Serial.println();
      Serial.print(F("(P)rimary host serial       : ")); print_serial_interface_host(get_bits(new_config_serial_settings, 8, 1));
      if( get_bits(new_config_serial_settings, 8, 1) != config_host_serial_primary() )
        {
          Serial.print(F(" (current: ")); 
          print_serial_interface_host(config_host_serial_primary());
          Serial.print(')');
        }
      Serial.println();
#endif

      Serial.println();
      Serial.print(F("(1) Configure SIO           : ")); print_serial_device_mapped_to(config_serial_device_settings[CSM_SIO]); Serial.println();
      Serial.print(F("(2) Configure ACR           : ")); print_serial_device_mapped_to(config_serial_device_settings[CSM_ACR]); Serial.println();
      Serial.print(F("(3) Configure 2SIO-1        : ")); print_serial_device_mapped_to(config_serial_device_settings[CSM_2SIO1]); Serial.println();
      Serial.print(F("(4) Configure 2SIO-2        : ")); print_serial_device_mapped_to(config_serial_device_settings[CSM_2SIO2]); Serial.println();

      Serial.println();
      Serial.println(F("(M)anage Filesystem"));
      Serial.println(F("(A)pply host serial settings"));
      Serial.println(F("(C)lear memory"));
      Serial.println(F("(S)ave configuration"));
      Serial.println(F("(L)oad configuration"));
      Serial.println(F("(R)eset to defaults"));
      Serial.println(F("\nE(x)it"));

      Serial.print(F("\n\nCommand: "));
      while( !serial_available() ) delay(50);
      c = serial_read();
      if( c>31 && c<127 ) Serial.println(c);

      switch( c )
        {
#if USE_PROFILING>0
        case 'f': toggle_flag(CF_PROFILE); break;
#endif
#if USE_THROTTLE>0
        case 't': toggle_flag(CF_THROTTLE); break;
#endif
        case 'p': toggle_flag(CF_SERIAL_PANEL); break;
#if STANDALONE==0
        case 'i': toggle_flag(CF_SERIAL_INPUT); break;
#endif
        case 'd': toggle_flag(CF_SERIAL_DEBUG); break;
        case 'c': toggle_flag(CF_CLEARMEM); break;

        case 'b': toggle_host_serial_baud_rate(0); break;
        case 'r': toggle_host_serial_baud_rate(1); break;
        case 'P': toggle_host_primary_interface(); break;

        case '1': config_edit_serial_device(CSM_SIO); break;
        case '2': config_edit_serial_device(CSM_ACR); break;
        case '3': config_edit_serial_device(CSM_2SIO1); break;
        case '4': config_edit_serial_device(CSM_2SIO2); break;
          
        case 'M': filesys_manage(); break;
        case 'A': apply_host_serial_settings(); break;
        case 'C': 
          {
            mem_clr_ram_limit();
            for(int i=0; i<MEMSIZE; i++) Mem[i] = 0; 
            break;
          }
        case 'S': 
          {
            Serial.print(F("\n\nSave as config # (0=default): "));
            byte i = numsys_read_word();
            Serial.println();
            if( !save_config(i & 0xff) )
              Serial.println(F("Saving failed. Capture/replay in progress?"));
            break;
          }
        case 'L':
          {
            Serial.print(F("\n\nLoad config #: "));
            byte i = numsys_read_word();
            Serial.println();
            if( !load_config(i & 0xff) )
              Serial.println(F("Load failed. File does not exist?"));
            break;
          }
        case 'R': config_defaults(false); break;

        case 27:
        case 'x':
          {
            if( config_serial_settings != new_config_serial_settings )
              {
                Serial.print(F("\nApply new host serial settings (y/n/ESC)? "));
                do { delay(50); c = serial_read(); } while( c!='y' && c!='n' && c!=27 );
                Serial.println(c);
                if( c==27 || (c=='y' && !apply_host_serial_settings()) )
                  continue;
              }

            Serial.print(F("\033[2J"));
            return;
          }
        }
    }
}


void config_defaults(bool apply)
{
  // default settings:
  // - SERIAL_DEBUG, SERIAL_INPUT, SERIAL_PANEL enabled if in STANDALONE mode, otherwise disabled
  // - Profiling disabled
  // - Throttling enabled (on Due)

  config_flags = 0;
#if STANDALONE>0
  config_flags |= CF_SERIAL_DEBUG;
  config_flags |= CF_SERIAL_INPUT;
  config_flags |= CF_SERIAL_PANEL;
#endif
  config_flags |= CF_THROTTLE;

  new_config_serial_settings  = 0;
  new_config_serial_settings |= (BAUD_115200 << 0); // Serial interface (USB): 115200 baud
  new_config_serial_settings |= (BAUD_9600   << 4); // Serial1 interface: 9600 baud
  new_config_serial_settings |= (0           << 8); // USB Serial is primary interface

  uint32_t s = 0;
  s |= (BAUD_1200 <<  0); // serial playback baud rate: 1200
  s |= (4         <<  4); // 4 NUL characters after newline
  s |= (0         <<  8); // not mapped to any host interface
  s |= (2         << 10); // autodetect uppercase inputs
  s |= (2         << 12); // autodetect 7 bit 
  s |= (2         << 14); // autodetect backspace translation

  for(byte dev=0; dev<4; dev++)
    config_serial_device_settings[dev] = s;

  config_serial_device_settings[CSM_SIO]   |= (1 << 8); // map to SIO to primary host interface
  config_serial_device_settings[CSM_2SIO1] |= (1 << 8); // map to 2SIO-1 to primary host interface
  config_serial_device_settings[CSM_ACR]   |= (1 << 7); // enable CLOAD traps

  if( apply ) apply_host_serial_settings(new_config_serial_settings);
}


void config_setup()
{
  if( load_config(0) ) 
    config_serial_settings = new_config_serial_settings;
  else
    config_defaults(true);
}
