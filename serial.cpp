#include "Altair8800.h"
#include "config.h"
#include "host.h"
#include "serial.h"
#include "filesys.h"
#include "prog_examples.h"
#include "cpucore.h"
#include "prog_ps2.h"

#define SST_RDRF    0x01 // receive register full (character received)
#define SST_TDRE    0x02 // send register empty (ready for next byte)
#define SST_FNF     0x04 // used in simulator to signal file-not-found in CLOAD
#define SST_OVRN    0x20 // data overrun (received character when previous not read)
#define SST_OVRN2   0x40 // used in the simulator to signal that OVRN is upcoming
#define SST_INT     0x80 // interrupt signaled


byte     acr_cload_fid     = 0;
uint32_t acr_cload_timeout = 0;

volatile byte serial_ctrl[4];
volatile byte serial_fid[4];
volatile byte serial_status[4], serial_status_dev[4];
volatile byte serial_data[4];
volatile byte last_active_primary_device = 0xff;

static void serial_replay(byte dev);
static void acr_read_next_byte();


void set_serial_status(byte dev, byte status)
{
  serial_status[dev] = status;

  if( dev==CSM_2SIO1 || dev==CSM_2SIO2 )
    serial_status_dev[dev] = status & ~SST_OVRN2;
  else
    {
      serial_status_dev[dev] = 0x81;
      if( status & SST_OVRN ) serial_status_dev[dev] |=  0x10;
      if( status & SST_RDRF ) serial_status_dev[dev] &= ~0x01;
      if( status & SST_TDRE ) serial_status_dev[dev] &= ~0x80;
    }
}


void serial_update_hlda_led()
{
  bool on = false;
  for(byte dev=0; !on && dev<3; dev++) 
    if( serial_fid[dev] ) on = true;

  if( on )
    host_set_status_led_HLDA();
  else
    host_clr_status_led_HLDA();
}


static byte get_device(byte switches)
{
  byte dev;
  if ( (switches & 0x80)==0 ) 
    dev = last_active_primary_device;
  else
    dev = (switches & 0x60) >> 5;

  return dev;
}


void serial_replay_start(byte switches, byte filenum)
{
  serial_acr_check_cload_timeout();

  byte dev = get_device(switches);

  if( serial_fid[dev]==0xfe )
    {
      DBG_FILEOPS(3, "ejecting PS2 tape");
      serial_fid[dev] = 0;
    }
  else if( serial_fid[dev]==0xff )
    {
      DBG_FILEOPS(3, "stopping BASIC example load");
      serial_fid[dev] = 0;
    }
  else if( serial_fid[dev]>0 )
    {
      if( filesys_is_read(serial_fid[dev]) )
        {
          DBG_FILEOPS(3, "stopping data replay");
          filesys_close(serial_fid[dev]);
          serial_fid[dev] = 0;
        }
      else
        DBG_FILEOPS(1, "unable to replay data (capture operation in progress)");
    }
  else
    {
      switch( dev )
        {
        case CSM_SIO:   DBG_FILEOPS(3, "replaying to 88-SIO");    break;
        case CSM_2SIO1: DBG_FILEOPS(3, "replaying to 88-2SIO-1"); break;
        case CSM_2SIO2: DBG_FILEOPS(3, "replaying to 88-2SIO-2"); break;
        case CSM_ACR:   DBG_FILEOPS(3, "replaying to ACR");       break;
        default:        DBG_FILEOPS(1, "invalid replay device");  break;
        }

      if( switches & 0x01 )
        {
          // load from file
          serial_fid[dev] = filesys_open_read('D', filenum);
          if( serial_fid[dev]>0 )
            DBG_FILEOPS2(3, "replaying data, file ", int(filenum));
          else
            DBG_FILEOPS2(2, "unable to replay data (file not found), file ", int(filenum));
        }
      else
        {
          // load example
          if( prog_examples_read_start(filenum) )
            {
              serial_fid[dev] = 0xff;
              DBG_FILEOPS2(3, "loading example ", int(filenum));
            }
          else
            DBG_FILEOPS2(2, "example does not exist: ", int(filenum));
        }
    }

  // either start interrupt timer or prepare first byte for replay
  if( serial_ctrl[dev] & SST_INT )
    serial_timer_interrupt_check_enable(dev);
  else
    serial_replay(dev);

  serial_update_hlda_led();
}


void serial_capture_start(byte switches, byte filenum)
{
  serial_acr_check_cload_timeout();

  byte dev = get_device(switches);

  if( serial_fid[dev]>0 )
    {
      // end capture (0xff is special fid for BASIC example replay)
      if( serial_fid[dev] < 0xff && filesys_is_write(serial_fid[dev]) )
        {
          DBG_FILEOPS(3, "ending capture");
          filesys_close(serial_fid[dev]);
          serial_fid[dev] = 0;
        }
      else
        DBG_FILEOPS(1, "cannot start capture (replay operation in progress)");
    }
  else
    {
      // start capture
      serial_fid[dev] = filesys_open_write('D', filenum);
      if( serial_fid[dev] )
        DBG_FILEOPS2(3, "starting data capture, file ", filenum);
      else
        DBG_FILEOPS(1, "unable to start capturing (storage full?)");
    }

  serial_update_hlda_led();
}


void serial_close_files()
{
  for(byte dev=0; dev<3; dev++)
    {
      if( serial_fid[dev]>0 && serial_fid[dev]<0xff ) 
        {
          filesys_close(serial_fid[dev]);
          serial_fid[dev] = 0;
        }

      if( acr_cload_fid>0 ) filesys_close(acr_cload_fid);
      acr_cload_fid = 0;
    }

  serial_timer_interrupt_check_enable();
  serial_update_hlda_led();
}


// -----------------------------------------------------------------------------------------------------------------------


void serial_reset()
{
  for(byte dev=0; dev<3; dev++)
    {
      serial_ctrl[dev]   = 0;
      set_serial_status(dev, 0);
    }

  serial_timer_interrupt_check_enable();
}


// called when serial data for a device is received
static void serial_receive_data(byte dev, byte b)
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
  
  // trigger interrupt if interrupts are enabled
  if( host_read_status_led_INTE() && (serial_ctrl[dev] & SST_INT) /*&& !(serial_status[dev] & SST_INT)*/ )
    {
      status |= SST_INT;
      set_serial_status(dev, status);
      altair_interrupt(INT_SERIAL);
    }
  else
    set_serial_status(dev, status);
}


// called by the host if serial data received
void serial_receive_host_data(byte host_interface, byte b)
{
  if( b==27 && config_serial_input_enabled() && !host_read_status_led_WAIT() )
    {
      // if we have serial input enabled then the the ESC key works as STOP
      altair_interrupt(INT_SW_STOP);
    }
  else
    {
      for(byte dev=0; dev<4; dev++)
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
              DBG_FILEOPS(4, "done loading");
              serial_fid[dev] = 0;
              serial_update_hlda_led();
              serial_timer_interrupt_check_enable();
            }
        }
      else
        {
          // play back captured data
          if( filesys_read_char(fid, &data) )
            {
              DBG_FILEOPS2(5, "replay data: ", int(data));
              serial_receive_data(dev, data);
            }
          else
            {
              DBG_FILEOPS(4, "no more data for replay");
              filesys_close(fid);
              serial_fid[dev] = 0;
              serial_update_hlda_led();
              serial_timer_interrupt_check_enable();
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
            for(dev=dev; dev<0xff; dev--) set_serial_status(dev, 0);
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

  byte status = serial_status[dev];
  status &= ~(SST_INT | SST_RDRF | SST_OVRN);
  if( status & SST_OVRN2 ) 
    { 
      // there was an overrun condition and the last properly received byte 
      // has just been read => signal OVRN now
      status |= SST_OVRN;
      status &= ~SST_OVRN2;
      //DBG_FILEOPS(1, "OVRN");
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
      if( host_interface==config_host_serial_primary() )
        last_active_primary_device = dev;
    }

  if( !host_read_status_led_WAIT() && serial_fid[dev]>0 && filesys_is_write(serial_fid[dev]) )
    {
      if( !filesys_write_char(serial_fid[dev], data) )
        {
          DBG_FILEOPS(1, "capture storage exhausted");
          //filesys_close(capture_fid);
          //capture_fid  = 0;
          //serial_update_hlda_led();
        }
      else
        DBG_FILEOPS2(5, "writing captured data: ", int(data));
    }
}


bool serial_timer_interrupt_enabled(byte dev)
{
  return host_interrupt_timer_running(dev);
}


void serial_timer_interrupt_disable(byte dev)
{
  if( dev<0xff )
    host_interrupt_timer_stop(dev);
  else
    for(dev=0; dev<4; dev++)
      host_interrupt_timer_stop(dev);
}


void serial_timer_interrupt_check_enable(byte dev)
{
  if( dev<0xff )
    {
      bool enable  = (serial_ctrl[dev] & SST_INT) && serial_fid[dev]>0 && (serial_fid[dev]==0xff || filesys_is_read(serial_fid[dev]));
      bool enabled = host_interrupt_timer_running(0);
      if( !enabled && enable )
        host_interrupt_timer_start(dev);
      else if( enabled && !enable )
        host_interrupt_timer_stop(dev);
    }
  else
    for(dev=0; dev<4; dev++)
      serial_timer_interrupt_check_enable(dev);
}


static void serial_timer_interrupt_SIO()   { serial_replay(CSM_SIO); }
static void serial_timer_interrupt_ACR()   { serial_replay(CSM_ACR); }
static void serial_timer_interrupt_2SIO1() { serial_replay(CSM_2SIO1); }
static void serial_timer_interrupt_2SIO2() { serial_replay(CSM_2SIO2); }


void serial_timer_interrupt_setup(byte dev)
{
  if( dev<0xff )
    {
      // calculate the number of microseconds it would take to receive
      // one byte given a specific baud rate (assuming 8 bit plus 1 stop bit and 1 parity bit):
      // (10 * 1000000) / baud_rate
      bool enabled = serial_timer_interrupt_enabled(dev);
      serial_timer_interrupt_disable(dev);
      uint32_t us_per_byte = 10000000lu / config_serial_playback_baud_rate(dev);
      switch( dev )
        {
        case CSM_SIO:   host_interrupt_timer_setup(dev, us_per_byte, serial_timer_interrupt_SIO);   break;
        case CSM_ACR:   host_interrupt_timer_setup(dev, us_per_byte, serial_timer_interrupt_ACR);   break;
        case CSM_2SIO1: host_interrupt_timer_setup(dev, us_per_byte, serial_timer_interrupt_2SIO1); break;
        case CSM_2SIO2: host_interrupt_timer_setup(dev, us_per_byte, serial_timer_interrupt_2SIO2); break;
        }

      if( enabled ) serial_timer_interrupt_check_enable(dev);
    }
  else
    for(dev=0; dev<4; dev++)
      serial_timer_interrupt_setup(dev);
}


static byte serial_map_characters_in(byte dev, byte data)
{
  // ALTAIR 4k BASIC I/O routine has "IN" instruction at 0x0389
  // MITS Programming System II has "IN" instruction at 0x0725 and 0x07E4
  // ALTAIR EXTENDED BASIC I/O routine has "IN" instruction at 0x009F

  if( config_serial_ucase(dev)==CSF_ON ||
      (config_serial_ucase(dev)==CSF_AUTO && (regPC == 0x038A || regPC == 0x0726 || regPC == 0x08e5)) )
    {
      // only use upper-case letters
      if( data>96 && data<123 ) data -= 32;
    }

  if( config_serial_backspace(dev)==CSF_ON ||
      (config_serial_backspace(dev)==CSF_AUTO && (regPC == 0x038A || regPC == 0x0726 || regPC == 0x08e5 || regPC == 0x00A0)) )
    {
      // translate backspace to underscore
      if( data==8 || data==127 ) data = '_';
    }

  return data;
}


static byte serial_map_characters_out(byte dev, byte data)
{
  // ALTAIR 4k BASIC I/O routine has "OUT" instruction at 0x037F
  // MITS Programming System II has OUT instruction at 0x071B
  // ALTAIR EXTENDED BASIC I/O routine has "OUT" instruction at 0x00AB

  if( config_serial_7bit(dev)==CSF_ON ||
      (config_serial_7bit(dev)==CSF_AUTO && (regPC == 0x0380 || regPC == 0x071C || regPC == 0x00AC)) )
    {
      // only use lower 7 bits
      data &= 0x7f;
    }

  if( config_serial_backspace(dev)==CSF_ON ||
      (config_serial_backspace(dev)==CSF_AUTO && (regPC == 0x0380 || regPC == 0x071C || regPC == 0x00AC)) )
    {
      // translate underscore to backspace
      if( data=='_' ) data = 127;
    }


  return data;
}

// -----------------------------------------------------------------------------------------------------------------------


byte serial_2sio1_in_ctrl()
{
  // read control register of first serial device of 88-2SIO-1
  byte data = serial_status_dev[CSM_2SIO1];
  byte fid  = serial_fid[CSM_2SIO1];

  if( fid>0 && fid<0xff && filesys_is_write(fid) )
    {
      if( !filesys_eof(fid) )
        data |= 0x02;
    }
  else if( host_serial_available_for_write(config_serial_map_sim_to_host(CSM_2SIO1)) )
    data |= 0x02;

  return data;
}


byte serial_2sio1_in_data()
{
  byte data = 0;

  if( serial_status[CSM_2SIO1] & SST_RDRF )
    {
      // get character
      data = serial_read(CSM_2SIO1);

      // map character (for BASIC etc)
      data = serial_map_characters_in(CSM_2SIO1, data);

      // if interrupts are not enabled, prepare the next byte for replay now
      if( !(serial_ctrl[CSM_2SIO1] & SST_INT) )
        serial_replay(CSM_2SIO1);
    }

  return data;
}


void serial_2sio1_out_ctrl(byte data)
{
  // write to control register of first serial device of 88-2SIO1

  if( !(serial_ctrl[CSM_2SIO1] & SST_INT) && (data & 0x80) )
    DBG_FILEOPS(4, "ENABLING interrupts on 2SIO1");
  else if( (serial_ctrl[CSM_2SIO1] & SST_INT) && !(data & 0x80) )
    DBG_FILEOPS(4, "disabling interrupts on 2SIO1");

  // write to control register of 88-2SIO1
  if( (data & 0x03)==0x03 )
    {
      // master reset
      serial_ctrl[CSM_2SIO1]   = 0;
      set_serial_status(CSM_2SIO1, 0);
    }
  else
    {
      serial_ctrl[CSM_2SIO1] = data;
      serial_timer_interrupt_check_enable();
      
      // clear interrupt flag in status
      set_serial_status(CSM_2SIO1, serial_status[CSM_2SIO1] & ~SST_INT);
    }
}


void serial_2sio1_out_data(byte data)
{
  // map character (for BASIC etc)
  data = serial_map_characters_out(CSM_2SIO1, data);

  // output character
  serial_write(CSM_2SIO1, data);
}


// ------------------------------------------------------------------------------------------------------------


byte serial_2sio2_in_ctrl()
{
  // read control register of second serial device of 88-2SIO
  byte data = serial_status_dev[CSM_2SIO2];
  byte fid  = serial_fid[CSM_2SIO2];

  if( fid>0 && fid<0xff && filesys_is_write(fid) )
    {
      if( !filesys_eof(fid) )
        data |= 0x02;
    }
  else if( host_serial_available_for_write(config_serial_map_sim_to_host(CSM_2SIO2)) )
    data |= 0x02;

  return data;
}


byte serial_2sio2_in_data()
{
  byte data = 0;

  if( serial_status[CSM_2SIO2] & SST_RDRF )
    {
      // get character
      data = serial_read(CSM_2SIO2);

      // map character (for BASIC etc)
      data = serial_map_characters_in(CSM_2SIO2, data);

      // if interrupts are not enabled, prepare the next byte for replay now
      if( !(serial_ctrl[CSM_2SIO2] & SST_INT) )
        serial_replay(CSM_2SIO2);
    }

  return data;
}


void serial_2sio2_out_ctrl(byte data)
{
  // write to control register of second serial device of 88-2SIO

  if( !(serial_ctrl[CSM_2SIO2] & SST_INT) && (data & 0x80) )
    DBG_FILEOPS(4, "ENABLING interrupts on 2SIO-2");
  else if( (serial_ctrl[CSM_2SIO2] & SST_INT) && !(data & 0x80) )
    DBG_FILEOPS(4, "disabling interrupts on 2SIO-2");

  // write to control register of 88-2SIO2
  if( (data & 0x03)==0x03 )
    {
      // master reset
      serial_ctrl[CSM_2SIO2]   = 0;
      set_serial_status(CSM_2SIO2, 0);
    }
  else
    {
      serial_ctrl[CSM_2SIO2] = data;
      serial_timer_interrupt_check_enable();
      
      // clear interrupt flag in status
      set_serial_status(CSM_2SIO2, serial_status[CSM_2SIO2] & ~SST_INT);
    }
}


void serial_2sio2_out_data(byte data)
{
  // map character (for BASIC etc)
  data = serial_map_characters_out(CSM_2SIO2, data);

  // output character
  serial_write(CSM_2SIO2, data);
}


// ------------------------------------------------------------------------------------------------------------


byte serial_sio_in_ctrl()
{
  byte data = serial_status_dev[CSM_SIO];
  byte fid  = serial_fid[CSM_SIO];

  if( fid>0 && fid<0xff && filesys_is_write(fid) )
    {
      if( !filesys_eof(fid) )
        data &= ~0x80;
    }
  else if( host_serial_available_for_write(config_serial_map_sim_to_host(CSM_SIO)) )
    data &= ~0x80;

  return data;
}


byte serial_sio_in_data()
{
  byte data = 0;

  if( serial_status[CSM_SIO] & SST_RDRF )
    {
      // get character
      data = serial_read(CSM_SIO);

      // map character (for BASIC etc)
      data = serial_map_characters_in(CSM_SIO, data);
      
      // if interrupts are not enabled, prepare the next byte for replay now
      if( !(serial_ctrl[CSM_SIO] & SST_INT) )
        serial_replay(CSM_SIO);
    }

  return data;
}


void serial_sio_out_ctrl(byte data)
{
  // write to control register of 88-SIO

  if( !(serial_ctrl[CSM_SIO] & SST_INT) && (data & 0x01) )
    {
      DBG_FILEOPS(4, "ENABLING interrupts on SIO");
      serial_ctrl[CSM_SIO] |= 0x80;
    }
  else if( (serial_ctrl[CSM_SIO] & SST_INT) && !(data & 0x01) )
    {
      DBG_FILEOPS(4, "disabling interrupts on SIO");
      serial_ctrl[CSM_SIO] &= ~0x80;
    }

  // check whether we need to enable timer interrupts
  serial_timer_interrupt_check_enable();

  // clear interrupt flag in status
  set_serial_status(CSM_SIO, serial_status[CSM_SIO] & ~SST_INT);
}


void serial_sio_out_data(byte data)
{
  // map character (for BASIC etc)
  data = serial_map_characters_out(CSM_SIO, data);

  // output character
  serial_write(CSM_SIO, data);
}


// ------------------------------------------------------------------------------------------------------------


bool serial_acr_mount_ps2()
{
  if( serial_fid[CSM_ACR] && serial_fid[CSM_ACR]!=0xff )
    {
      DBG_FILEOPS(2, "cannot mount PS2 tape (other operation in progress)");
      return false;
    }
  else 
    {
      // (re-) mount the PS2 tape
      prog_ps2_read_start();
      DBG_FILEOPS(3, "mounting PS2 tape");
      serial_fid[CSM_ACR] = 0xfe;
      acr_read_next_byte();
      serial_update_hlda_led();
      return true;
    }
}


bool serial_acr_check_cload_timeout()
{
  if( acr_cload_timeout>0 && millis() > acr_cload_timeout )
    {
      // if the last write or read from BASIC was more than .1 seconds ago
      // then this is a new read/write operation => close the previous file
      if( acr_cload_fid>0 )
        {
          filesys_close(acr_cload_fid);
          acr_cload_fid = 0;
          DBG_FILEOPS(4, "closing tape file due to timeout");
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
              if( acr_cload_fid ) DBG_FILEOPS2(4, "reading BASIC CSAVE file: ", char(32+tape_fname));
              tape_fname++;
            }
              
          if( acr_cload_fid==0 )
            {
              serial_status[CSM_ACR] |= SST_FNF;
              break;
            }
        }
    }

  acr_cload_timeout = millis() + 100;
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
              if( acr_cload_fid ) DBG_FILEOPS2(4, "writing BASIC CSAVE file: ", char(data));
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

  acr_cload_timeout = millis() + 100;
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
      else if( !(serial_ctrl[CSM_ACR] & SST_INT) )
        serial_replay(CSM_ACR);
    }
  else if( regPC==0xE2A0 && config_serial_trap_CLOAD() )
    acr_read_next_cload_byte();
}


byte serial_acr_in_ctrl()
{
  byte data = serial_status_dev[CSM_ACR];
  byte fid  = serial_fid[CSM_ACR];

  if( fid>0 && fid<0xff && filesys_is_write(fid) )
    {
      if( !filesys_eof(fid) )
        data &= ~0x80;
    }
  else if( host_serial_available_for_write(config_serial_map_sim_to_host(CSM_ACR)) )
    data &= ~0x80;

  if( (regPC==0xE299 || regPC==0xE2A7) && config_serial_trap_CLOAD() )
    {
      // This is ALTAIR Extended BASIC loading from or saving to ACR
      // our BASIC CSAVE/CLOAD tape emulation is a continuous loop so we always
      // have data available (i.e. bit 0 NOT set) 
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
      else if( !(serial_status[CSM_ACR]&SST_RDRF) )
        acr_read_next_cload_byte();
    }

  return data;
}



byte serial_acr_in_data()
{
  byte data = 0;

  if( serial_status[CSM_ACR] & SST_RDRF )
    {
      // get character
      data = serial_read(CSM_ACR);

      // if interrupts are not enabled, prepare the next byte for replay now
      if( !(serial_ctrl[CSM_ACR] & SST_INT) )
        acr_read_next_byte();
    }

  return data;
}


void serial_acr_out_ctrl(byte data)
{
  // write to control register of acr interface
  if( !(serial_ctrl[CSM_ACR] & 0x80) && (data & 0x01) )
    {
      DBG_FILEOPS(4, "ENABLING interrupts on ACR");
      serial_ctrl[CSM_ACR] |= 0x80;
    }
  else if( (serial_ctrl[CSM_ACR] & 0x80) && !(data & 0x01) )
    {
      DBG_FILEOPS(4, "disabling interrupts on ACR");
      serial_ctrl[CSM_ACR] &= ~0x80;
    }

  // check whether we need to enable timer interrupts
  serial_timer_interrupt_check_enable();

  // clear interrupt flag in status
  set_serial_status(CSM_ACR, serial_status[CSM_ACR] & ~SST_INT);
}


void serial_acr_out_data(byte data)
{
  //printf("ACR port data write at %04x: %02x (%c)\n", regPC, data, data>=32 ? data : '.');

  // check for timeout from previous operation
  serial_acr_check_cload_timeout();

  if( regPC == 0xE2AF && config_serial_trap_CLOAD() && 
      (serial_fid[CSM_ACR]==0 || !filesys_is_write(serial_fid[CSM_ACR])) )
    acr_write_next_csave_byte(data);
  else
    serial_write(CSM_ACR, data);

  DBG_FILEOPS2(5, "ACR writing captured data: ", int(data));
}


void serial_setup()
{
  serial_timer_interrupt_setup();
}
