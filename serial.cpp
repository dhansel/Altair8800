#include "Altair8800.h"
#include "config.h"
#include "host.h"
#include "serial.h"
#include "filesys.h"
#include "prog_examples.h"
#include "cpucore.h"
#include "prog_ps2.h"

#define ST_2SIO_RDRF    0x01 /* receive register full (character received) */
#define ST_2SIO_TDRE    0x02 /* send register empty (ready for next byte) */
#define ST_2SIO_OVRN    0x20 /* data overrun (received character when previous not read) */
#define ST_2SIO_OVRN2   0x40 /* used in the simulator to signal that OVRN is upcoming */
#define ST_2SIO_INT     0x80 /* interrupt signaled */

#define CAPTURE_SIO     1
#define CAPTURE_2SIO1   2
#define CAPTURE_TAPE    3


byte capture_device = 0, capture_fid = 0, tape_basic_fid = 0;
uint32_t tape_basic_last_op = 0;

byte serial_2SIO_ctrl = 0;
volatile byte serial_2SIO_status = 0;
volatile byte serial_2SIO_data  = 0;



void serial_update_hlda_led()
{
  if( capture_fid>0 )
    host_set_status_led_HLDA();
  else
    host_clr_status_led_HLDA();
}


void serial_replay_start(byte device, byte filenum)
{
  serial_tape_check_timeout();

  if( capture_fid==0xff )
    {
      if( capture_device == CAPTURE_TAPE )
        DBG_FILEOPS(3, "ejecting PS2 tape");
      else
        DBG_FILEOPS(3, "stopping BASIC example load");

      capture_fid = 0;
    }
  else if( capture_fid>0 )
    {
      if( filesys_is_read(capture_fid) )
        {
          DBG_FILEOPS(3, "stopping data replay");
          filesys_close(capture_fid);
          capture_fid = 0;
        }
      else
        DBG_FILEOPS(1, "unable to replay data (capture operation in progress)");
    }
  else if( capture_fid>0 )
    DBG_FILEOPS(1, "unable to load example (other capture/replay operation in progress)");
  else
    {
      capture_device = 0;
      if( device == 0x08 )
        { capture_device = CAPTURE_2SIO1; DBG_FILEOPS(3, "replaying to 88-2SIO"); }
      else if( device == 0x04 )
        { capture_device = CAPTURE_TAPE; DBG_FILEOPS(3, "replaying to TAPE"); }
      if( device == 0x02 )
        { capture_device = CAPTURE_SIO; DBG_FILEOPS(3, "replaying to 88-SIO"); }
          
      if( capture_device>0 )
        {
          capture_fid = filesys_open_read('S', filenum);
          if( capture_fid>0 )
            DBG_FILEOPS2(3, "replaying captured data, file ", int(filenum));
          else
            DBG_FILEOPS2(2, "unable to replay captured data (file not found), file ", int(filenum));
        }
      else if( device==0 )
        {
          if( prog_examples_read_start(filenum) )
            {
              capture_fid    = 0xff;
              capture_device = CAPTURE_2SIO1;
              DBG_FILEOPS2(3, "loading example ", int(filenum));
            }
          else
            DBG_FILEOPS2(2, "example does not exist: ", int(filenum));
        }
      else
        DBG_FILEOPS(2, "invalid replay device");
    }

  serial_update_hlda_led();
}


void serial_capture_start(byte device, byte filenum)
{
  serial_tape_check_timeout();

  if( capture_fid>0 )
    {
      // end capture (0xff is special fid for BASIC example playback)
      if( capture_fid < 0xff && filesys_is_write(capture_fid) )
        {
          filesys_close(capture_fid);
          capture_fid = 0;
          DBG_FILEOPS(3, "ending capture");
        }
      else
        DBG_FILEOPS(1, "cannot start capture (replay operation in progress)");
    }
  else
    {
      capture_device = 0;
      if( device == 0x08 )
        { capture_device = CAPTURE_2SIO1; DBG_FILEOPS(3, "capturing from 88-2SIO"); }
      else if( device == 0x04 )
        { capture_device = CAPTURE_TAPE; DBG_FILEOPS(3, "capturing from TAPE"); }
      if( device == 0x02 )
        { capture_device = CAPTURE_SIO; DBG_FILEOPS(3, "capturing from 88-SIO"); }
      
      if( capture_device!=0 )
        {
          // start capture
          capture_fid = filesys_open_write('S', filenum);
          if( capture_fid )
            DBG_FILEOPS2(3, "starting data capture, file ", filenum);
          else
            DBG_FILEOPS(1, "unable to start capturing (storage full?)");
        }
      else
        DBG_FILEOPS(2, "invalid capture device");
    }
      
  serial_update_hlda_led();
}


void serial_close_files()
{
  if( capture_fid>0 && capture_fid<0xff ) filesys_close(capture_fid);
  if( tape_basic_fid>0 ) filesys_close(tape_basic_fid);
  capture_fid = 0;
  tape_basic_fid = 0;
  serial_update_hlda_led();
}


// -----------------------------------------------------------------------------------------------------------------------


void serial_reset()
{
  serial_2SIO_ctrl   = 0;
  serial_2SIO_status = 0;
}


// called by the host if serial data received
void serial_receive_data(byte b)
{
  if( b==27 && altair_use_serial_input && !host_read_status_led_WAIT() )
    {
      // if we have serial input enabled then the the ESC key works as STOP
      altair_interrupt(INT_SW_STOP);
    }
  else
    {
      // store received data (the 2SIO board can only buffer one byte)
      if( serial_2SIO_status & ST_2SIO_RDRF )
        {
          // overrun occurred
          // according to 2SIO (and 6850 UART) documentation, the overrun is not
          // signaled until the previous (properly received) byte has been read.
          // so we set a flag here to signal that as soon as the current byte is
          // read we have to signal the overrun condition.
          serial_2SIO_status |= ST_2SIO_OVRN2;
        }
      else
        {
          serial_2SIO_data = b;
          serial_2SIO_status |= ST_2SIO_RDRF;
        }

      // trigger interrupt if interrupts are enabled
      if( host_read_status_led_INTE() && (serial_2SIO_ctrl & 0x80) && (serial_2SIO_status & ST_2SIO_INT)==0 )
        {
          serial_2SIO_status |= ST_2SIO_INT;
          altair_interrupt(INT_SERIAL_2SIO);
        }
    }
}


bool serial_available()
{
  host_check_interrupts();
  return (serial_2SIO_status & ST_2SIO_RDRF)!=0;
}


int serial_read()
{
  if( serial_available() )
    {
      serial_2SIO_status &= ~(ST_2SIO_INT | ST_2SIO_RDRF | ST_2SIO_OVRN);
      if( serial_2SIO_status & ST_2SIO_OVRN2 ) 
        { 
          // there was an overrun condition and the last properly received byte 
          // has just been read => signal OVRN now
          serial_2SIO_status |= ST_2SIO_OVRN;
          serial_2SIO_status &= ST_2SIO_OVRN2;
        }
      return serial_2SIO_data;
    }
  else
    return -1;
}


void serial_timer_interrupt()
{
  Serial.println("SERIAL-TIMER");
}


// -----------------------------------------------------------------------------------------------------------------------


byte serial_2sio_in_ctrl()
{
  // read control register of first serial device of 88-2SIO
  byte data = 0;

  if( capture_device==CAPTURE_2SIO1 && capture_fid>0 && (capture_fid==0xff || filesys_is_read(capture_fid)) )
    {
      if( capture_fid==0xff )
        {
          // we're playing back a basic example, those are always 0-terminated
          // and capture_fid==0xff would not hold if we had
          // reached the end
          data |= ST_2SIO_RDRF;
        }
      else
        {
          if( !filesys_eof(capture_fid) )
            data |= ST_2SIO_RDRF;
        }
    }
  else 
    data |= serial_2SIO_status & (ST_2SIO_RDRF | ST_2SIO_OVRN | ST_2SIO_INT);

  if( capture_device==CAPTURE_2SIO1 && capture_fid>0 && capture_fid<0xff && filesys_is_write(capture_fid) )
    {
      if( !filesys_eof(capture_fid) )
        data |= ST_2SIO_TDRE;
    }
  else if( Serial.availableForWrite() )
    data |= ST_2SIO_TDRE;

  return data;
}


byte serial_2sio_in_data()
{
  byte data = 0;

  // read data from first serial device of 88-2SIO
  if( capture_device==CAPTURE_2SIO1 && capture_fid>0 && (capture_fid==0xff || filesys_is_read(capture_fid)) )
    {
      if( capture_fid==0xff )
        {
          // special fid for BASIC example replay
          if( !prog_examples_read_next(&data) )
            {
              capture_fid = 0;
              DBG_FILEOPS(4, "done loading");
              serial_update_hlda_led();
            }
        }
      else
        {
          if( !filesys_read_char(capture_fid, &data) )
            {
              DBG_FILEOPS(4, "no more data for replay");
              data = 0;
            }
          else
            DBG_FILEOPS2(5, "88-2SIO reading captured data: ", int(data));
        }
    }
  else
    {
      if( serial_2SIO_status & ST_2SIO_RDRF )
        data = serial_read();

      if( regPC == 0x038A || regPC == 0x0726 )
        {
          // ALTAIR BASIC I/O routine has "IN" instruction at 0x0389
          // MITS Programming System II has "IN" instruction at 0x0725

          // only use upper-case letters
          if( data>96 && data<123 ) data -= 32;
                
          // translate backspace to underscore
          if( data==8 || data==127 ) data = '_';
        }
      else if( regPC == 0x00A0 )
        {
          // ALTAIR EXTENDED BASIC I/O routine has "IN" instruction at 0x009F

          // translate backspace to underscore
          if( data==8 || data==127 ) data = '_';
        }
    }

  fflush(stdout);
  return data;
}


void serial_2sio_out_ctrl(byte data)
{
  // write to control register of first serial device of 88-2SIO

  if( !(serial_2SIO_ctrl & 0x80) && (data & 0x80) )
    DBG_FILEOPS(4, "ENABLING interrupts on 2SIO");
  else if( (serial_2SIO_ctrl & 0x80) && !(data & 0x80) )
    DBG_FILEOPS(4, "disabling interrupts on 2SIO");

  // write to control register of 88-2SIO
  if( (data & 0x03)==0x03 )
    {
      // master reset
      serial_2SIO_ctrl   = 0;
      serial_2SIO_status = 0;
    }
  else
    {
      serial_2SIO_ctrl   = data;
      serial_2SIO_status &= ~ST_2SIO_INT;
    }
}


void serial_2sio_out_data(byte data)
{
  // write data to first serial device of 88-2SIO
  if( regPC == 0x0380 || regPC == 0x071C )
    {
      // ALTAIR BASIC I/O routine has "OUT" instruction at 0x037F
      // MITS Programming System II has OUT instruction at 0x071B

      // only use lower 7 bits
      data &= 0x7f;
            
      // translate '_' to backspace
      if( data=='_' ) data = 127;
    }
  else if( regPC == 0x00AC )
    {
      // ALTAIR EXTENDED BASIC I/O routine has "OUT" instruction at 0x00AB

      // translate '_' to backspace
      if( data=='_' ) data = 127;
    }

  Serial.write(data);
  if( !host_read_status_led_WAIT() && capture_fid>0 && (capture_device==CAPTURE_2SIO1) && filesys_is_write(capture_fid) )
    {
      if( !filesys_write_char(capture_fid, data) )
        {
          DBG_FILEOPS(1, "capture storage exhausted");
          //filesys_close(capture_fid);
          //capture_fid  = 0;
          //serial_update_hlda_led();
        }
      else
        DBG_FILEOPS2(5, "88-2SIO writing captured data: ", int(data));
    }
}


// ------------------------------------------------------------------------------------------------------------


byte serial_sio_in_ctrl()
{
  byte data = 0;

  // read control register of 88-SIO
  if( capture_device==CAPTURE_SIO && capture_fid>0 && capture_fid<0xff )
    {
      if( !filesys_is_write(capture_fid) || filesys_eof(capture_fid) )
        data |= 0x80;

      if( !filesys_is_read(capture_fid) || filesys_eof(capture_fid) )
        data |= 0x01;
    }
  else
    data |= 0x81;

  return data;
}


byte serial_sio_in_data()
{
  byte data = 0;

  // read data from 88-SIO
  if( capture_device==CAPTURE_SIO && capture_fid>0 && capture_fid<0xff && filesys_is_read(capture_fid) )
    {
      if( !filesys_read_char(capture_fid, &data) )
        {
          DBG_FILEOPS(4, "no more data for replay");
          data = 0;
        }
      else
        DBG_FILEOPS2(5, "88-SIO reading captured data: ", int(data));
    }

  return data;
}


void serial_sio_out_ctrl(byte data)
{
  // write to control register of 88-SIO
}


void serial_sio_out_data(byte data)
{
  // write data to 88-SIO
  if( capture_fid>0 && (capture_device==CAPTURE_SIO) && filesys_is_write(capture_fid) )
    {
      if( !filesys_write_char(capture_fid, data) )
        {
          DBG_FILEOPS(1, "capture storage exhausted");
          //filesys_close(capture_fid);
          //capture_fid  = 0;
          //serial_update_hlda_led();
        }
      else
        DBG_FILEOPS2(5, "88-SIO writing captured data: ", int(data));
    }
}


// ------------------------------------------------------------------------------------------------------------


bool serial_tape_mount_ps2()
{
  if( capture_fid>0 && (capture_device!=CAPTURE_TAPE && capture_fid!=0xff) )
    {
      DBG_FILEOPS(2, "cannot mount PS2 tape (other operation in progress)");
      return false;
    }
  else 
    {
      // (re-) mount the PS2 tape
      prog_ps2_read_start();
      DBG_FILEOPS(3, "mounting PS2 tape");
      capture_device = CAPTURE_TAPE;
      capture_fid    = 0xff;
      serial_update_hlda_led();
      return true;
    }
}


bool serial_tape_check_timeout()
{
  if( tape_basic_fid>0 && millis() > tape_basic_last_op+100 )
    {
      // if the last write or read from BASIC was more than .1 seconds ago
      // then this is a new read/write operation => close the previous file
      filesys_close(tape_basic_fid);
      tape_basic_fid = 0;
      DBG_FILEOPS(4, "closing tape file due to timeout");
      return true;
    }
  
  return false;
}  


static byte tape_read_next_byte()
{
  static byte tape_fname = 0;
  byte data;
  bool go = true;

  if( capture_fid>0 && capture_device==CAPTURE_TAPE && (capture_fid==0xff || filesys_is_read(capture_fid)) )
    {
      if( capture_fid==0xff )
        {
          /* reading Programming System II data from tape (endless loop) */
          prog_ps2_read_next(&data);
          go = false;
        }
      else if( !filesys_read_char(capture_fid, &data) )
        {
          DBG_FILEOPS(4, "no more captured data");
          //filesys_close(capture_fid);
          //capture_fid = 0;
          //serial_update_hlda_led();
          data = 0;
        }
    }
  else if( regPC==0xE2A0 )
    {
      // This is ALTAIR Extended BASIC loading from tape
      
      // check for timeout from previous operation
      serial_tape_check_timeout();
  
      // if we were writing before, close the file now
      if( tape_basic_fid>0 && !filesys_is_read(tape_basic_fid) )
        {
          filesys_close(tape_basic_fid);
          tape_basic_fid = 0;
        }

      // no file is open: either we closed it due to timeout or
      // there was a FILE NOT FOUND error earlier. In either case,
      // we need to start searching from the first file again.
      if( tape_basic_fid==0 )
        tape_fname = 0;

      while( go )
        {
          /*if( altair_interrupts & INT_SW_STOP )
            break;
            else*/ 

          if( tape_basic_fid>0 )
            {
              if( filesys_read_char(tape_basic_fid, &data) )
                go = false;
              else
                {
                  filesys_close(tape_basic_fid);
                  tape_basic_fid = 0;
                }
            }

          if( go )
            {
              while( tape_basic_fid==0 && tape_fname<96 )
                {
                  tape_basic_fid = filesys_open_read('B', 32+tape_fname);
                  if( tape_basic_fid ) DBG_FILEOPS2(4, "reading BASIC CSAVE file: ", char(32+tape_fname));
                  tape_fname++;
                }
              
              if( tape_basic_fid==0 )
                {
                  // ALTAIR BASIC would wait forever and require the user to
                  // reset the computer if a file is not found. We catch that
                  // condition here for convenience. We're setting the PC to 
                  // 0xC0A0 because it will be increased by one at the the end
                  // of the IN instruction.
                  Serial.println(F("FILE NOT FOUND"));
                  regPC = 0xC0A0;
                  break;
                }
            }
        }

      tape_basic_last_op = millis();
    }

  return data;
}


static void tape_write_next_byte(byte data)
{
  static byte leadchar = 0, leadcount = 0, endcount = 0;

  // check for timeout from previous operation
  serial_tape_check_timeout();
  
  if( capture_fid>0 && capture_device==CAPTURE_TAPE && filesys_is_write(capture_fid) )
    {
      if( !filesys_write_char(capture_fid, data) )
        {
          DBG_FILEOPS(1, "capture storage exhausted");
          //filesys_close(capture_fid);
          //capture_fid  = 0;
          //serial_update_hlda_led();
        }
    }
  else if( regPC == 0xE2AF )
    {
      // This is ALTAIR Extended BASIC saving to tape

      // if we were reading before, close the file now
      if( tape_basic_fid>0 && !filesys_is_write(tape_basic_fid) )
        {
          filesys_close(tape_basic_fid);
          tape_basic_fid = 0;
        }
  
      if( tape_basic_fid==0 )
        {
          if( leadcount==0 && data==0xd3 )
            leadcount = 1;
          else if( data == 0xd3 ) 
            leadcount++;
          else 
            {
              if( leadcount>3 )
                {
                  tape_basic_fid = filesys_open_write('B', data);
                  if( tape_basic_fid ) DBG_FILEOPS2(4, "writing BASIC CSAVE file: ", char(data));
                  for(byte i=0; i<leadcount; i++) filesys_write_char(tape_basic_fid, 0xd3);
                  filesys_write_char(tape_basic_fid, data);
                }
              leadcount = 0;
            }
        }
      else
        {
          filesys_write_char(tape_basic_fid, data);
      
          if( data == 0x00 )
            endcount++;
          else
            endcount = 0;
          
          if( endcount==10 )
            {
              filesys_close(tape_basic_fid);
              tape_basic_fid = 0;
              endcount = 0;
            }
        }

      tape_basic_last_op = millis();
    }
}


byte serial_tape_in_ctrl()
{
  byte data = 0;

  if( capture_device==CAPTURE_TAPE && capture_fid>0 )
    {
      if( capture_fid == 0xff )
        {
          // This is us loading the Programming System II tape
          // => we can always read (continuous loop) but not write
          data |= 0x80;
        }
      else
        {
          if( !filesys_is_write(capture_fid) || filesys_eof(capture_fid) )
            data |= 0x80;
                
          if( !filesys_is_read(capture_fid) || filesys_eof(capture_fid) )
            data |= 0x01;
        }
    }
  else if( regPC==0xE299 || regPC==0xE2A7 )
    {
      // This is ALTAIR Extended BASIC loading from or saving to tape
      // our BASIC CSAVE/CLOAD tape emulation is a continuous loop so we always
      // have data available (i.e. bit 0 NOT set) 
      data = 0x00;
    }
  else
    data = 0x81;

  return data;
}


byte serial_tape_in_data()
{
  byte data = 0;

  data = tape_read_next_byte();
  DBG_FILEOPS2(5, "tape reading captured data: ", int(data));

  return data;
}


void serial_tape_out_ctrl(byte data)
{
  // write to control register of tape interface
}


void serial_tape_out_data(byte data)
{
  //printf("TAPE port data write at %04x: %02x (%c)\n", regPC, data, data>=32 ? data : '.');
  tape_write_next_byte(data);
  DBG_FILEOPS2(5, "88-SIO writing captured data: ", int(data));
}


