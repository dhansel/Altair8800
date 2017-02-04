#include "config.h"
#include "mem.h"
#include "serial.h"
#include "filesys.h"
#include "numsys.h"

uint32_t config_flags;
uint32_t config_serial_settings;


int config_serial_baud()
{
  switch( (config_serial_settings & 0xf0)>>8 )
    {
    case 1:  return 300;
    case 2:  return 1200;
    case 3:  return 9600;
    case 4:  return 19200;
    case 5:  return 57600;
    case 6:  return 115200;
    default: return 9600;
    }
}


static void print_flag(int value)
{
  Serial.println((config_flags&value)!=0 ? F("yes") : F("no"));
}

static void toggle_flag(int value)
{
  config_flags = (config_flags & ~value) | (~(config_flags & value) & value);
}

static void toggle_serial_io()
{
  byte d = (config_serial_map_to() + 1) % 3;
  config_serial_settings = (config_serial_settings & ~0x03) | d;
}

void config_clear_mem()
{
  for(int i=0; i<MEMSIZE; i++) Mem[i] = 0;
}

static bool save_config(byte fileno)
{
  // better to write all data at once (will overwrite instead
  // of deleting/creating the file)
  byte data[8];
  memcpy(data, &config_flags, 4);
  memcpy(data+4, &config_serial_settings, 4);
  return filesys_write_file('C', fileno, (void *) data, 8);
}


static bool load_config(byte fileno)
{
  byte fid = filesys_open_read('C', fileno);
  if( fid )
    {
      filesys_read_data(fid, &config_flags, 4);
      filesys_read_data(fid, &config_serial_settings, 4);
      filesys_close(fid);
#if STANDALONE>0
      config_flags |= CF_SERIAL_INPUT;      
#endif
      return true;
    }
  else
    return false;
}


void config_edit()
{
  while( true )
    {
      Serial.print(F("\033[2J\n"));
#if USE_PROFILING>0
      Serial.print(F("Enable pro(f)iling          : ")); print_flag(CF_PROFILE); 
#endif
#if USE_THROTTLE>0
      Serial.print(F("Enable (t)hrottling         : ")); print_flag(CF_THROTTLE); 
#endif
      Serial.print(F("Enable serial (p)anel       : ")); print_flag(CF_SERIAL_PANEL); 
#if STANDALONE==0
      Serial.print(F("Enable serial (i)nput       : ")); print_flag(CF_SERIAL_INPUT); 
#endif
      Serial.print(F("Enable serial (d)ebug       : ")); print_flag(CF_SERIAL_DEBUG); 
      Serial.print(F("(c)lear memory on powerup   : ")); print_flag(CF_CLEARMEM); 
      Serial.print(F("Map host (s)erial in/out to : "));
      switch( config_serial_map_to() )
        {
        case CSM_2SIO: Serial.println(F("2-SIO")); break;
        case CSM_SIO:  Serial.println(F("SIO")); break;
        case CSM_TAPE: Serial.println(F("TAPE")); break;
        }
      
      Serial.println(F("\n(M)anage Filesystem"));
      Serial.println(F("(C)lear memory"));
      Serial.println(F("(S)ave configuration"));
      Serial.println(F("(L)oad configuration"));
      Serial.println(F("\nE(x)it"));

      Serial.print(F("\n\nCommand: "));
      while( !serial_available() ) delay(50);
      switch( serial_read() )
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
        case 's': toggle_serial_io(); break;

        case 'M': filesys_manage(); break;

        case 'C': config_clear_mem(); break;

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
        case 27:
        case 'x':
          {
            Serial.print(F("\033[2J"));
            return;
          }
        }
    }
}


void config_setup()
{
  if( !load_config(0) )
    {
      config_flags = 0;
#if STANDALONE>0
      config_flags |= CF_SERIAL_DEBUG;
      config_flags |= CF_SERIAL_INPUT;
      //config_flags |= CF_SERIAL_PANEL;
#endif

      config_serial_settings = CSM_2SIO;
    }
}
