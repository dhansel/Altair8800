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
#include "host.h"
#include "serial.h"
#include "filesys.h"
#include "prog_examples.h"
#include "cpucore.h"
#include "prog_ps2.h"
#include "timer.h"
#include "numsys.h"
#include "io.h"

#define SST_RDRF     0x01 // receive register full (character received)
#define SST_TDRE     0x02 // send register empty (ready for next byte)
#define SST_FNF      0x04 // used in simulator to signal file-not-found in CLOAD
#define SST_OVRN     0x20 // data overrun (received character when previous not read)
#define SST_OVRN2    0x40 // used in the simulator to signal that OVRN is upcoming
#define SST_INT      0x80 // interrupt signaled

#define SSC_SIOTP0   0x01 // revision of SIO board, bit 0
#define SSC_SIOTP1   0x02 // revision of SIO board, bit 1
#define SSC_INTTX    0x20 // transmit interrupt enabled
#define SSC_REALTIME 0x40 // force real-(simulation-)time operation (always use baud rate)
#define SSC_INTRX    0x80 // receive  interrupt enabled

#define b2s numsys_byte2string

byte     acr_cload_fid     = 0;
uint32_t acr_cload_timeout = 0;


#if USE_SECOND_2SIO>0
#define NUM_SERIAL_DEVICES 6
static const uint32_t serial_device_interrupts[6] = {INT_SIO, INT_ACR, INT_2SIO1, INT_2SIO2, INT_2SIO3, INT_2SIO4};
#else
#define NUM_SERIAL_DEVICES 4
static const uint32_t serial_device_interrupts[4] = {INT_SIO, INT_ACR, INT_2SIO1, INT_2SIO2};
#endif

volatile byte serial_ctrl[NUM_SERIAL_DEVICES], serial_data[NUM_SERIAL_DEVICES];
volatile byte serial_status[NUM_SERIAL_DEVICES], serial_status_dev[NUM_SERIAL_DEVICES];
byte serial_fid[NUM_SERIAL_DEVICES];
static byte last_active_primary_device = CSM_SIO;

static void serial_replay(byte dev);
static void acr_read_next_byte();
static void serial_timer_interrupt_check_enable(byte dev = 0xff);

void serial_sio_register_io();
void serial_acr_register_io();
void serial_2sio_register_io(byte dev);


static const __FlashStringHelper *devname(byte dev)
{
  switch( dev )
    {
    case CSM_SIO:   return F("88-SIO");
    case CSM_ACR:   return F("ACR");
    case CSM_2SIO1: return F("88-2SIO-1"); 
    case CSM_2SIO2: return F("88-2SIO-2");
#if USE_SECOND_2SIO>0
    case CSM_2SIO3: return F("2nd 88-2SIO-1");
    case CSM_2SIO4: return F("2nd 88-2SIO-2");
#endif
    }
  
  return F("[invalid device]");
}


byte serial_last_active_primary_device()
{
  return last_active_primary_device;
}


void set_serial_status(byte dev, byte status)
{
  // check whether the interrupt line should be enabled
  // either receive interrups are enabled and receive register is full
  // or transmit interrupts are enabled and transmit register is empty
  bool intr = ((serial_ctrl[dev] & SSC_INTRX) && (status & SST_RDRF) || 
               (serial_ctrl[dev] & SSC_INTTX) && (status & SST_TDRE));

  // set the INT status bit accordingly
  if( intr ) status |=  SST_INT;
  else       status &= ~SST_INT;

  // set the status
  serial_status[dev] = status;

  // enable or disable the ALTAIR interrupt status
  altair_interrupt(serial_device_interrupts[dev], intr);

  // set the device status register
  if( dev>=CSM_2SIO1 && dev<=CSM_2SIO4 )
    serial_status_dev[dev] = status & ~SST_OVRN2;
  else if( (serial_ctrl[dev]&(SSC_SIOTP0|SSC_SIOTP1))==1 || dev==CSM_ACR )
    {
      // 88-SIO rev 1
      byte data = 0x00;
      if( status & SST_OVRN )    data |= 0x10;
      if( !(status & SST_RDRF) ) data |= 0x01;
      if( !(status & SST_TDRE) ) data |= 0x80;
      serial_status_dev[dev] = data;
    }
  else if( (serial_ctrl[dev]&(SSC_SIOTP0|SSC_SIOTP1))==0 )
    {
      // 88-SIO rev 0
      byte data = 0x00;
      if( (status & SST_RDRF) ) data |= 0x20;
      if( (status & SST_TDRE) ) data |= 0x02;
      serial_status_dev[dev] = data;
    }
  else
    {
      // console attached to Cromeco FDC
      byte data = 0x00;
      if( status & SST_OVRN )   data |= 0x02;
      if( (status & SST_RDRF) ) data |= 0x40;
      if( (status & SST_TDRE) ) data |= 0x80;
      serial_status_dev[dev] = data;
    }

  // check if timer interrupts need to be enabled or disabled
  serial_timer_interrupt_check_enable(dev);
}


void serial_update_hlda_led()
{
  bool on = false;
  for(byte dev=0; !on && dev<NUM_SERIAL_DEVICES; dev++) 
    if( serial_fid[dev] ) on = true;

  if( on )
    host_set_status_led_HLDA();
  else
    host_clr_status_led_HLDA();
}


void serial_replay_start(byte dev, bool example, byte filenum)
{
  serial_acr_check_cload_timeout();

  if( example )
    {
      // load example
      if( prog_examples_read_start(filenum) )
        {
          serial_fid[dev] = 0xff;
          DBG_FILEOPS4(3, F("loading example #"), b2s(filenum), F(" via "), devname(dev));
        }
      else
        DBG_FILEOPS3(2, F("example #"), b2s(filenum), F(" does not exist"));
    }
  else
    {
      // load from file
      serial_fid[dev] = filesys_open_read('D', filenum);
      if( serial_fid[dev]>0 )
	DBG_FILEOPS4(3, F("loading data file #"), b2s(filenum), F(" via "), devname(dev));
      else
	DBG_FILEOPS3(2, F("data file #"), b2s(filenum), F(" not found"));
    }

  // either start interrupt timer or prepare first byte for replay
  if( serial_ctrl[dev] & (SSC_INTRX|SSC_REALTIME) )
    serial_timer_interrupt_check_enable(dev);
  else
    serial_replay(dev);
  
  serial_update_hlda_led();
}


void serial_capture_start(byte dev, byte filenum)
{
  serial_acr_check_cload_timeout();

  serial_fid[dev] = filesys_open_write('D', filenum);
  if( serial_fid[dev] )
    DBG_FILEOPS4(3, F("capturing from "), devname(dev), F(" to data file #"), b2s(filenum));
  else
    DBG_FILEOPS(1, F("unable to start capturing (storage full?)"));
  
  serial_update_hlda_led();
}
    

bool serial_replay_running(byte dev)
{
  return serial_fid[dev]>=0xfe || serial_fid[dev]>0 && filesys_is_read(serial_fid[dev]);
}


bool serial_capture_running(byte dev)
{
  return serial_fid[dev]<0xfe && serial_fid[dev]>0 && filesys_is_write(serial_fid[dev]);
}


void serial_stop(byte dev)
{
  if( serial_fid[dev]==0xfe )
    {
      DBG_FILEOPS(3, F("ejecting PS2 tape"));
      serial_fid[dev] = 0;
    }
  else if( serial_fid[dev]==0xff )
    {
      DBG_FILEOPS(3, F("stopping example load"));
      serial_fid[dev] = 0;
   }
  else if( serial_fid[dev]>0 )
    {
      if( filesys_is_read(serial_fid[dev]) )
	DBG_FILEOPS(3, F("stopping data replay"));
      else if( filesys_is_write(serial_fid[dev]) )
	DBG_FILEOPS(3, F("ending capture"));

      filesys_close(serial_fid[dev]);
      serial_fid[dev] = 0;
     }

  serial_update_hlda_led();
}


void serial_close_files()
{
  for(byte dev=0; dev<NUM_SERIAL_DEVICES; dev++)
    if( serial_fid[dev]>0 ) serial_stop(dev);

  if( acr_cload_fid>0 ) filesys_close(acr_cload_fid);
  acr_cload_fid = 0;

  serial_timer_interrupt_check_enable();
  serial_update_hlda_led();
}


// -----------------------------------------------------------------------------------------------------------------------


void serial_register_ports()
{
  serial_sio_register_io();
  serial_acr_register_io();
  for(byte dev=CSM_2SIO1; dev<NUM_SERIAL_DEVICES && dev<=CSM_2SIO4; dev++) 
    serial_2sio_register_io(dev); 
}


void serial_set_config(byte dev)
{
  if( dev==0xff )
    {
      serial_register_ports();
      for(dev=0; dev<NUM_SERIAL_DEVICES; dev++) serial_set_config(dev);
    }
  else
    {
      if( config_serial_realtime(dev) )
        serial_ctrl[dev] |= SSC_REALTIME;
      else
        serial_ctrl[dev] &= ~SSC_REALTIME;

      if( dev==CSM_SIO ) serial_ctrl[dev] = (serial_ctrl[dev] & ~3) | (config_serial_siorev() & 3);

      switch( dev )
        {
        case CSM_SIO   : serial_sio_register_io(); break;
        case CSM_ACR   : serial_acr_register_io(); break;
        case CSM_2SIO1 : 
        case CSM_2SIO2 : 
        case CSM_2SIO3 : 
        case CSM_2SIO4 : serial_2sio_register_io(dev); break;
        }
    }
}


void serial_reset(byte dev)
{
  if( dev==0xff )
    {
      for(dev=0; dev<NUM_SERIAL_DEVICES; dev++) serial_reset(dev);
    }
  else
    {
      serial_ctrl[dev] = 0;
      serial_set_config(dev);
      set_serial_status(dev, SST_TDRE);
    }
}


// called when serial data for a device is received
void serial_receive_data(byte dev, byte b)
{
  byte status = serial_status[dev];

  // store received data
  if( status & SST_RDRF )
    {
      // overrun occurred
      // according to 2SIO1 (and 6850 UART) documentation, the overrun is not
      // signaled until the previous (properly received) byte has been read.
      // so we set a flag here to signal that as soon as the current byte is
      // read we have to signal the overrun condition.
      status |= SST_OVRN2;
    }
  else
    {
      serial_data[dev] = b;
      status |= SST_RDRF;
    }
  
  // set status register (also triggers interrupt if necessary)
  set_serial_status(dev, status);
}


// called by the host if serial data received
void serial_receive_host_data(byte host_interface, byte b)
{
  static unsigned long prevESC = 0;
  if( b==27 && config_serial_input_enabled() && !host_read_status_led_WAIT() && host_interface==config_host_serial_primary() )
    {
      if( millis()-prevESC>50 && millis()-prevESC<250 )
        {
          // if we have serial input enabled then hitting 
          // the ESC key twice works as STOP
          altair_interrupt(INT_SW_STOP);
        }
      else
        {
          for(byte dev=0; dev<NUM_SERIAL_DEVICES; dev++)
            if( config_serial_map_sim_to_host(dev)==host_interface )
              serial_receive_data(dev, b);
        }

      prevESC = millis();
    }
  else
    {
      prevESC = 0;
      for(byte dev=0; dev<NUM_SERIAL_DEVICES; dev++)
        if( config_serial_map_sim_to_host(dev)==host_interface )
          serial_receive_data(dev, b);
    }
}


static void serial_replay(byte dev)
{
  byte fid = serial_fid[dev];

  if( fid>0 && (fid==0xff || filesys_is_read(fid)) )
    {
      byte data;
      if( fid==0xff )
        {
          // 0xff is special fid for example replay
          if( prog_examples_read_next(dev, &data) )
            serial_receive_data(dev, data);
          else
            {
              DBG_FILEOPS(4, F("done loading"));
              serial_fid[dev] = 0;
              serial_update_hlda_led();
            }
        }
      else
        {
          // play back captured data
          if( filesys_read_char(fid, &data) )
            {
              DBG_FILEOPS2(5, F("replay data: "), int(data));
              serial_receive_data(dev, data);
            }
          else
            {
              DBG_FILEOPS(4, F("no more data for replay"));
              filesys_close(fid);
              serial_fid[dev] = 0;
              serial_update_hlda_led();
            }
        }
    }
}

bool serial_available()
{
  // this is the simulator reading serial inputs
  host_check_interrupts();

  // read devices from high to low. When written in serial_host_receive_data
  // we apply changes from low to high. Going in the different direction
  // ensures we are not seeing inputs twice if two devices are mapped to
  // the primary host interface.
  for(byte dev=3; dev<0xff; dev--)
    if( config_serial_map_sim_to_host(dev)==config_host_serial_primary() )
      return (serial_status[dev] & SST_RDRF)!=0;

  return false;
}


int serial_read()
{
  // this is the simulator reading serial inputs
  int res = -1;

  host_check_interrupts();

  // read devices from high to low. When written in serial_host_receive_data
  // we apply changes from low to high. Going in the different direction
  // ensures we are not seeing inputs twice if two devices are mapped to
  // the primary host interface.
  for(byte dev=3; dev<0xff; dev--)
    if( config_serial_map_sim_to_host(dev)==config_host_serial_primary() )
      {
        if( (serial_status[dev] & SST_RDRF) )
          {
            res = serial_data[dev];
            for(dev=dev; dev<0xff; dev--) 
              if( config_serial_map_sim_to_host(dev)==config_host_serial_primary() )
                set_serial_status(dev, 0);
          }

        return res;
      }

  return res;
}


static byte serial_read(byte dev)
{
  // this is the simulated code reading serial data
  host_check_interrupts();

  if( config_serial_map_sim_to_host(dev)==config_host_serial_primary() )
    last_active_primary_device = dev;

  byte status = serial_status[dev] & ~(SST_RDRF | SST_OVRN);
  if( status & SST_OVRN2 ) 
    { 
      // there was an overrun condition and the last properly received byte 
      // has just been read => signal OVRN now
      status |= SST_OVRN;
      status &= ~SST_OVRN2;
      //DBG_FILEOPS(1, F("OVRN"));
    }

  set_serial_status(dev, status);

  // note that we're reading whatever is there, even
  // if nothing new has arrived
  return serial_data[dev];
}


void serial_write(byte dev, byte data)
{
  byte host_interface = config_serial_map_sim_to_host(dev);

  if( host_interface!=0xff )
    {
      host_serial_write(host_interface, data);

      // if backspace is translated then force destructive backspace
      // by sending BACKSPACE-SPACE-BACKSPACE instead of just BACKSPACE
      if( data==8 && config_serial_backspace(dev, regPC)!=CSFB_NONE )
        { host_serial_write(host_interface, 32); host_serial_write(host_interface, 8); }

      if( host_interface==config_host_serial_primary() )
        last_active_primary_device = dev;
    }

  if( !host_read_status_led_WAIT() && serial_fid[dev]>0 && filesys_is_write(serial_fid[dev]) )
    {
      if( !filesys_write_char(serial_fid[dev], data) )
        {
          DBG_FILEOPS(1, F("capture storage exhausted"));
          //filesys_close(capture_fid);
          //capture_fid  = 0;
          //serial_update_hlda_led();
        }
      else
        DBG_FILEOPS2(5, F("writing captured data: "), b2s(data));
    }
}


void serial_timer_interrupt_check_enable(byte dev)
{
  if( dev<0xff )
    {
      bool enable  = ((serial_ctrl[dev] & (SSC_INTRX|SSC_REALTIME)) && serial_fid[dev]>0 && (serial_fid[dev]==0xff || filesys_is_read(serial_fid[dev]))
                      ||
                      (serial_ctrl[dev] & (SSC_INTTX|SSC_REALTIME)) && !(serial_status[dev] & SST_TDRE));
      
      bool enabled = timer_running(dev);

      if( !enabled && enable )
        timer_start(dev);
      else if( enabled && !enable )
        timer_stop(dev);
    }
  else
    for(dev=0; dev<NUM_SERIAL_DEVICES; dev++)
      serial_timer_interrupt_check_enable(dev);
}


static void serial_timer_interrupt(byte dev)
{
  // write out data scheduled to write (if necessary)
  if( serial_ctrl[dev] & (SSC_INTTX|SSC_REALTIME) )
    {
      // update status register (send buffer is empty now)
      serial_status[dev] |= SST_TDRE;
    }

  // read serial playback data
  serial_replay(dev);

  // this schedules additional interrupts as necessary
  set_serial_status(dev, serial_status[dev]);
}


static void serial_timer_interrupt_SIO()   { serial_timer_interrupt(CSM_SIO); }
static void serial_timer_interrupt_ACR()   { serial_timer_interrupt(CSM_ACR); }
static void serial_timer_interrupt_2SIO1() { serial_timer_interrupt(CSM_2SIO1); }
static void serial_timer_interrupt_2SIO2() { serial_timer_interrupt(CSM_2SIO2); }
#if USE_SECOND_2SIO>0
static void serial_timer_interrupt_2SIO3() { serial_timer_interrupt(CSM_2SIO3); }
static void serial_timer_interrupt_2SIO4() { serial_timer_interrupt(CSM_2SIO4); }
#endif


void serial_timer_interrupt_setup(byte dev)
{
  if( dev<0xff )
    {
      // calculate the number of microseconds it would take to receive
      // one byte given a specific baud rate (assuming 8 bit plus 1 start bit and 1 stop bit):
      // (10 * 1000000) / baud_rate
      timer_stop(dev);
      uint32_t us_per_byte = 10000000lu / config_serial_playback_baud_rate(dev);
      switch( dev )
        {
        case CSM_SIO:   timer_setup(dev, us_per_byte, serial_timer_interrupt_SIO);   break;
        case CSM_ACR:   timer_setup(dev, us_per_byte, serial_timer_interrupt_ACR);   break;
        case CSM_2SIO1: timer_setup(dev, us_per_byte, serial_timer_interrupt_2SIO1); break;
        case CSM_2SIO2: timer_setup(dev, us_per_byte, serial_timer_interrupt_2SIO2); break;
#if USE_SECOND_2SIO>0
        case CSM_2SIO3: timer_setup(dev, us_per_byte, serial_timer_interrupt_2SIO3); break;
        case CSM_2SIO4: timer_setup(dev, us_per_byte, serial_timer_interrupt_2SIO4); break;
#endif
        }

      if( config_serial_realtime(dev) )
        serial_ctrl[dev] |=  SSC_REALTIME;
      else
        serial_ctrl[dev] &= ~SSC_REALTIME;

      serial_timer_interrupt_check_enable(dev);
    }
  else
    for(dev=0; dev<NUM_SERIAL_DEVICES; dev++)
      serial_timer_interrupt_setup(dev);
}


static byte serial_map_characters_in(byte dev, byte data)
{
  if( config_serial_ucase(dev, regPC) )
    {
      // only use upper-case letters
      if( data>96 && data<123 ) data -= 32;
    }

  if( data==8 )
    {
      switch( config_serial_backspace(dev, regPC) )
        {
        case CSFB_UNDERSCORE: 
          if( data==8 ) data = '_';
          break;
          
        case CSFB_DELETE:
          if( data==8 ) data = 127;
          break;
        }
    }

  return data;
}


static byte serial_map_characters_out(byte dev, byte data)
{
  if( config_serial_7bit(dev, regPC) )
    {
      // only use lower 7 bits
      data &= 0x7f;
    }

  byte b = config_serial_backspace(dev, regPC);
  if( (data=='_' && b==CSFB_UNDERSCORE) || (data==127 && b==CSFB_DELETE) )
    data = 8;

  return data;
}

// -----------------------------------------------------------------------------------------------------------------------

// map between port number (16..23, 0x10..0x17) and CSM_2SIO1..CSM_2SIO4
#define PORT2DEV(port) ((port/2)+CSM_2SIO1-8)
#define DEV2PORT(dev)  ((dev+8-CSM_2SIO1)*2)

byte serial_2sio_in_ctrl(byte port)
{
  // read control register of 88-2SIO device
  byte dev  = PORT2DEV(port);
  byte data = serial_status_dev[dev];
  byte fid  = serial_fid[dev];

  // if transmit interrupts are enabled then the TDRE flag is handled
  // by the timer. If not, we need to determine the flag setting here
  // (the RDRF flag is handled separately in serial_receive_data)
  if( !(serial_ctrl[dev] & (SSC_INTTX|SSC_REALTIME)) )
    {
      if( fid>0 && fid<0xff && filesys_is_write(fid) )
        {
          if( !filesys_eof(fid) )
            data |= SST_TDRE;
          else
            data &= ~SST_TDRE;
        }
      else if( host_serial_available_for_write(config_serial_map_sim_to_host(dev)) )
        data |= SST_TDRE;
      else
        data &= ~SST_TDRE;
    }

  return data;
}


byte serial_2sio_in_data(byte port)
{
  byte data = 0;
  byte dev  = PORT2DEV(port);

  // get character
  data = serial_read(dev);
  
  // map character (for BASIC etc)
  data = serial_map_characters_in(dev, data);

  // if interrupts are not enabled, prepare the next byte for replay now
  if( !(serial_ctrl[dev] & (SSC_INTRX|SSC_REALTIME)) )
    serial_replay(dev);

  return data;
}


void serial_2sio_out_ctrl(byte port, byte data)
{
  byte dev  = PORT2DEV(port);

  // write to control register of 88-2SIO1 device
  if( !(serial_ctrl[dev] & SSC_INTRX) && (data & 0x80) )
    {
      DBG_FILEOPS2(4, F("ENABLING receive interrupts on 2SIO-"), dev==CSM_2SIO1 ? '1' : '2');
      serial_ctrl[dev] |= SSC_INTRX;
    }
  else if( (serial_ctrl[dev] & SSC_INTRX) && !(data & 0x80) )
    {
      DBG_FILEOPS2(4, F("disabling receive interrupts on 2SIO-"), dev==CSM_2SIO1 ? '1' : '2');
      serial_ctrl[dev] &= ~SSC_INTRX;
    }

  if( !(serial_ctrl[dev] & SSC_INTTX) && (data & 0x60)==0x20 )
    {
      DBG_FILEOPS2(4, F("ENABLING transmit interrupts on 2SIO-"), dev==CSM_2SIO1 ? '1' : '2');
      serial_ctrl[dev] |= SSC_INTTX;
    }
  else if( (serial_ctrl[dev] & SSC_INTTX) && (data & 0x60)!=0x20 )
    {
      DBG_FILEOPS2(4, F("disabling transmit interrupts on 2SIO-"), dev==CSM_2SIO1 ? '1' : '2');
      serial_ctrl[dev] &= ~SSC_INTTX;
    }

  if( (data & 0x03)==0x03 )
    {
      // master reset
      serial_reset(dev);
    }

  // check serial status (interrupts)
  set_serial_status(dev, serial_status[dev]);
}


void serial_2sio_out_data(byte port, byte data)
{
  byte dev = PORT2DEV(port);

  // map character (for BASIC etc)
  data = serial_map_characters_out(dev, data);

  // output character
  serial_write(dev, data);

  if( serial_ctrl[dev] & (SSC_INTTX|SSC_REALTIME) )
    {
      // transmit interrupts are enabled
      // update status register (send buffer is NOT empty now)
      // this also schedules a timer to set the TDRE flag again
      set_serial_status(dev, serial_status[dev] & ~SST_TDRE);
    }
}


void serial_2sio_register_io(byte dev)
{
  byte port   = DEV2PORT(dev);
  bool mapped = config_serial_map_sim_to_host(dev)!=0xFF;
  io_register_port_inp(port,   mapped ? serial_2sio_in_ctrl  : NULL);
  io_register_port_inp(port+1, mapped ? serial_2sio_in_data  : NULL);
  io_register_port_out(port,   mapped ? serial_2sio_out_ctrl : NULL);
  io_register_port_out(port+1, mapped ? serial_2sio_out_data : NULL);
}


// ------------------------------------------------------------------------------------------------------------


byte serial_sio_in_ctrl(byte port)
{
  byte data = serial_status_dev[CSM_SIO];

  // if transmit interrupts are enabled then the TDRE flag is handled
  // by the timer. If not, we need to determine the flag setting here
  // (the RDRF flag is handled separately in serial_receive_data)
  if( !(serial_ctrl[CSM_SIO] & (SSC_INTTX|SSC_REALTIME)) )
    {
      byte fid = serial_fid[CSM_SIO];

      bool can_send = true;
      if( fid>0 && fid<0xff && filesys_is_write(fid) )
        { if( filesys_eof(fid) ) can_send = false; }
      else
        { if( !host_serial_available_for_write(config_serial_map_sim_to_host(CSM_SIO)) ) can_send = false; }

      switch( serial_ctrl[CSM_SIO] & (SSC_SIOTP0|SSC_SIOTP1) )
        {
        case 0: // 88SIO rev0
          data = data & ~0x02;
          if( can_send ) data |= 0x02;
          break;

        case 1: // 88SIO rev1
          data = data & ~0x80;
          if( !can_send ) data |= 0x80;
          break;

        case 2: // Cromemco
          data = data & ~0x80;
          if( can_send ) data |= 0x80;
          break;
        }
    }

  return data;
}


byte serial_sio_in_data(byte port)
{
  byte data = 0;

  // get character
  data = serial_read(CSM_SIO);

  // map character (for BASIC etc)
  data = serial_map_characters_in(CSM_SIO, data);
      
  // if interrupts are not enabled, prepare the next byte for replay now
  if( !(serial_ctrl[CSM_SIO] & (SSC_INTRX|SSC_REALTIME)) )
    serial_replay(CSM_SIO);

  return data;
}


void serial_sio_out_ctrl(byte port, byte data)
{
  // port 0 of Cromemco serial card is baud rate (ignore)
  if( (serial_ctrl[CSM_SIO] & (SSC_SIOTP0|SSC_SIOTP1))==2 )
    return;

  // write to control register of 88-SIO
  if( !(serial_ctrl[CSM_SIO] & SSC_INTRX) && (data & 0x01) )
    {
      DBG_FILEOPS(4, F("ENABLING receive interrupts on SIO"));
      serial_ctrl[CSM_SIO] |= SSC_INTRX;
    }
  else if( (serial_ctrl[CSM_SIO] & SSC_INTRX) && !(data & 0x01) )
    {
      DBG_FILEOPS(4, F("disabling receive interrupts on SIO"));
      serial_ctrl[CSM_SIO] &= ~SSC_INTRX;
    }

  if( !(serial_ctrl[CSM_SIO] & SSC_INTTX) && (data & 0x02) )
    {
      DBG_FILEOPS(4, F("ENABLING transmit interrupts on SIO"));
      serial_ctrl[CSM_SIO] |= SSC_INTTX;
    }
  else if( (serial_ctrl[CSM_SIO] & SSC_INTTX) && !(data & 0x02) )
    {
      DBG_FILEOPS(4, F("disabling transmit interrupts on SIO"));
      serial_ctrl[CSM_SIO] &= ~SSC_INTTX;
    }

  // check whether we need to enable timer interrupts
  serial_timer_interrupt_check_enable(CSM_SIO);

  // clear interrupt flag in status
  set_serial_status(CSM_SIO, serial_status[CSM_SIO] & ~SST_INT);
}


void serial_sio_out_data(byte port, byte data)
{
  // map character (for BASIC etc)
  data = serial_map_characters_out(CSM_SIO, data);

  // output character
  serial_write(CSM_SIO, data);

  if( serial_ctrl[CSM_SIO] & (SSC_INTTX|SSC_REALTIME) )
    {
      // transmit interrupts are enabled
      // update status register (send buffer is NOT empty now)
      // this also schedules a timer to set the TDRE flag again
      set_serial_status(CSM_SIO, serial_status[CSM_SIO] & ~SST_TDRE);
    }
}


void serial_sio_register_io()
{
  bool mapped = config_serial_map_sim_to_host(CSM_SIO)!=0xFF;
  io_register_port_inp(0, mapped ? serial_sio_in_ctrl  : NULL);
  io_register_port_inp(1, mapped ? serial_sio_in_data  : NULL);
  io_register_port_out(0, mapped ? serial_sio_out_ctrl : NULL);
  io_register_port_out(1, mapped ? serial_sio_out_data : NULL);
}


// ------------------------------------------------------------------------------------------------------------


bool serial_acr_mount_ps2()
{
  if( serial_fid[CSM_ACR] && serial_fid[CSM_ACR]!=0xfe )
    {
      DBG_FILEOPS(2, F("cannot mount PS2 tape (other operation in progress)"));
      return false;
    }
  else 
    {
      // (re-) mount the PS2 tape
      prog_ps2_read_start();
      DBG_FILEOPS(3, F("mounting PS2 tape"));
      serial_fid[CSM_ACR] = 0xfe;
      acr_read_next_byte();
      serial_update_hlda_led();
      return true;
    }
}


bool serial_acr_check_cload_timeout()
{
  // timeout is 0.1 (simulated) seconds, i.e. 200000 cycles (at 2MHz)
  if( acr_cload_timeout>0 && (timer_get_cycles()-acr_cload_timeout)>200000 )
    {
      // if the last write or read from BASIC was more than 0.1 seconds ago
      // then this is a new read/write operation => close the previous file
      if( acr_cload_fid>0 )
        {
          filesys_close(acr_cload_fid);
          acr_cload_fid = 0;
          DBG_FILEOPS(4, F("closing tape file due to timeout"));
        }

      set_serial_status(CSM_ACR, 0);
      acr_cload_timeout = 0;
      return true;
    }

  return false;
}  

// ALTAIR Extended BASIC loading from tape via CLOAD
static void acr_read_next_cload_byte()
{
  static byte tape_fname = 0;
  bool go = true;
  byte data;

  // check for timeout from previous operation
  serial_acr_check_cload_timeout();
  
  // if we were writing before, close the file now
  if( acr_cload_fid>0 && !filesys_is_read(acr_cload_fid) )
    {
      filesys_close(acr_cload_fid);
      acr_cload_fid = 0;
    }

  // no file is open: either we closed it due to timeout or
  // there was a FILE NOT FOUND error earlier. In either case,
  // we need to start searching from the first file again.
  if( acr_cload_fid==0 )
    tape_fname = 0;

  while( go )
    {
      if( acr_cload_fid>0 )
        {
          if( filesys_read_char(acr_cload_fid, &data) )
            {
              serial_receive_data(CSM_ACR, data);
              go = false;
            }
          else
            {
              filesys_close(acr_cload_fid);
              acr_cload_fid = 0;
            }
        }

      if( go )
        {
          while( acr_cload_fid==0 && tape_fname<96 )
            {
              acr_cload_fid = filesys_open_read('B', 32+tape_fname);
              if( acr_cload_fid ) DBG_FILEOPS2(4, F("reading BASIC CSAVE file: "), char(32+tape_fname));
              tape_fname++;
            }
              
          if( acr_cload_fid==0 )
            {
              serial_status[CSM_ACR] |= SST_FNF;
              break;
            }
        }
    }

  acr_cload_timeout = timer_get_cycles();
}

// This is ALTAIR Extended BASIC saving to ACR via CSAVE
static void acr_write_next_csave_byte(byte data)
{
  static byte leadchar = 0, leadcount = 0, endcount = 0;

  // if we were reading before, close the file now
  if( acr_cload_fid>0 && !filesys_is_write(acr_cload_fid) )
    {
      filesys_close(acr_cload_fid);
      acr_cload_fid = 0;
    }
  
  if( acr_cload_fid==0 )
    {
      if( leadcount==0 && data==0xd3 )
        leadcount = 1;
      else if( data == 0xd3 ) 
        leadcount++;
      else 
        {
          if( leadcount>3 )
            {
              acr_cload_fid = filesys_open_write('B', data);
              if( acr_cload_fid ) DBG_FILEOPS2(4, F("writing BASIC CSAVE file: "), char(data));
              for(byte i=0; i<leadcount; i++) filesys_write_char(acr_cload_fid, 0xd3);
              filesys_write_char(acr_cload_fid, data);
            }
          leadcount = 0;
        }
    }
  else
    {
      filesys_write_char(acr_cload_fid, data);
      
      if( data == 0x00 )
        endcount++;
      else
        endcount = 0;
          
      if( endcount==10 )
        {
          filesys_close(acr_cload_fid);
          acr_cload_fid = 0;
          endcount = 0;
        }
    }

  acr_cload_timeout = timer_get_cycles();
}


static void acr_read_next_byte()
{
  byte fid = serial_fid[CSM_ACR];
  if( fid>0 )
    {
      if( fid==0xfe )
        {
          // reading Programming System II data from tape (endless loop)
          byte data;
          prog_ps2_read_next(&data);
          serial_receive_data(CSM_ACR, data);
        }
      else if( !(serial_ctrl[CSM_ACR] & (SSC_INTRX|SSC_REALTIME)) )
        serial_replay(CSM_ACR);
    }
  else if( regPC==0xE2A0 && config_serial_trap_CLOAD() )
    acr_read_next_cload_byte();
}


byte serial_acr_in_ctrl(byte port)
{
  byte data = serial_status_dev[CSM_ACR];

  // if transmit interrupts are enabled then the TDRE flag is handled
  // by the timer. If not, we need to determine the flag setting here
  // (the RDRF flag is handled separately in serial_receive_data)
  if( !(serial_ctrl[CSM_ACR] & (SSC_INTTX|SSC_REALTIME)) )
    {
      byte fid = serial_fid[CSM_ACR];
      
      data &= ~0x80;
      if( fid>0 && fid<0xff && filesys_is_write(fid) )
        { if( filesys_eof(fid) ) data |= 0x80; }
      else
        { if( !host_serial_available_for_write(config_serial_map_sim_to_host(CSM_ACR)) ) data |= 0x80; }
    }

  // do the following only if no regular file is currently open on the ACR
  // that way we can override the automated mechanism with manual control
  if( serial_fid[CSM_ACR]==0 && (regPC==0xE299 || regPC==0xE2A7) && config_serial_trap_CLOAD() )
    {
      // This is ALTAIR Extended BASIC loading from or saving to ACR
      // our BASIC CSAVE/CLOAD tape emulation is a continuous loop so we always
      // have data available (i.e. bit 0 NOT set) and can always write (i.e. bit 7 NOT set)
      data = 0x00;
      
      serial_acr_check_cload_timeout();
      if( serial_status[CSM_ACR] & SST_FNF )
        {
          // ALTAIR BASIC would wait forever and require the user to
          // reset the computer if a file is not found. We catch that
          // condition here for convenience. We're setting the PC to 
          // 0xC0A0 because it will be increased by one at the end
          // of the IN instruction and C0A1 is a good reset entry point.
          Serial.println(F("FILE NOT FOUND"));
          regPC = 0xC0A0;
          serial_status[CSM_ACR] &= ~SST_FNF;
        }
      else if( acr_cload_fid==0 && regPC==0xE299 )
        {
          // loading from ACR and CLOAD file is not open => read first byte
          acr_read_next_cload_byte();
        }
    }

  return data;
}



byte serial_acr_in_data(byte port)
{
  byte data = 0;

  // get character
  data = serial_read(CSM_ACR);

  // if interrupts are not enabled, prepare the next byte for replay now
  if( !(serial_ctrl[CSM_ACR] & (SSC_INTRX|SSC_REALTIME)) )
    acr_read_next_byte();

  DBG_FILEOPS2(5, F("ACR reading data: "), int(data));

  return data;
}


void serial_acr_out_ctrl(byte port, byte data)
{
  // write to control register of acr interface
  if( !(serial_ctrl[CSM_ACR] & SSC_INTRX) && (data & 0x01) )
    {
      DBG_FILEOPS(4, F("ENABLING interrupts on ACR"));
      serial_ctrl[CSM_ACR] |= SSC_INTRX;
    }
  else if( (serial_ctrl[CSM_ACR] & SSC_INTRX) && !(data & 0x01) )
    {
      DBG_FILEOPS(4, F("disabling interrupts on ACR"));
      serial_ctrl[CSM_ACR] &= ~SSC_INTRX;
    }

  if( !(serial_ctrl[CSM_ACR] & SSC_INTTX) && (data & 0x02) )
    {
      DBG_FILEOPS(4, F("ENABLING transmit interrupts on ACR"));
      serial_ctrl[CSM_ACR] |= SSC_INTTX;
    }
  else if( (serial_ctrl[CSM_ACR] & SSC_INTTX) && !(data & 0x02) )
    {
      DBG_FILEOPS(4, F("disabling transmit interrupts on ACR"));
      serial_ctrl[CSM_ACR] &= ~SSC_INTTX;
    }

  // check whether we need to enable timer interrupts
  serial_timer_interrupt_check_enable(CSM_ACR);

  // clear interrupt flag in status
  set_serial_status(CSM_ACR, serial_status[CSM_ACR] & ~SST_INT);
}


void serial_acr_out_data(byte port, byte data)
{
  //printf("ACR port data write at %04x: %02x (%c)\n", regPC, data, data>=32 ? data : '.');

  // check for timeout from previous operation
  serial_acr_check_cload_timeout();

  if( regPC == 0xE2AF && config_serial_trap_CLOAD() && 
      (serial_fid[CSM_ACR]==0 || !filesys_is_write(serial_fid[CSM_ACR])) )
    acr_write_next_csave_byte(data);
  else
    serial_write(CSM_ACR, data);

  DBG_FILEOPS2(5, F("ACR writing captured data: "), int(data));

  if( serial_ctrl[CSM_ACR] & SSC_INTTX )
    {
      // transmit interrupts are enabled
      // update status register (send buffer is NOT empty now)
      // this also schedules a timer to set the TDRE flag again
      set_serial_status(CSM_ACR, serial_status[CSM_ACR] & ~SST_TDRE);
    }
}


void serial_acr_register_io()
{
  bool mapped = config_serial_map_sim_to_host(CSM_ACR)!=0xFF;
  io_register_port_inp(6, mapped ? serial_acr_in_ctrl  : NULL);
  io_register_port_inp(7, mapped ? serial_acr_in_data  : NULL);
  io_register_port_out(6, mapped ? serial_acr_out_ctrl : NULL);
  io_register_port_out(7, mapped ? serial_acr_out_data : NULL);
}


void serial_setup()
{
  serial_timer_interrupt_setup();
  serial_reset();
}
