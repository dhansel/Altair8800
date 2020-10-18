// -----------------------------------------------------------------------------
// Altair 8800 Simulator 
// Teensy Audio (Tape) interface
// Copyright (C) 2020 Dirk Herrendoerfer
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

// ----------------------------------------------------------------------------------------------
// Audio Tape decoding and input - Simulates a serial port

IntervalTimer tapeTimer;

// Tape signal detect 
static volatile uint16_t ain_last = 0;
static volatile uint16_t ain_current = 0;
static volatile uint32_t last_rise_millis = 0;
static volatile uint16_t rise_wave_period = 0;

static volatile uint8_t low_repeat = 0;
static volatile uint8_t high_repeat = 0;

// Tape-Serial char Buffer
static volatile uint8_t a_serial_buffer[8];
static volatile uint8_t a_serial_floor = 0;
static volatile uint8_t a_serial_ceil = 0;

// Tape-Serial decoder
static volatile uint8_t a_lastlevel = 0;
static volatile uint8_t a_level = 0;
static volatile uint8_t a_char  = 0;
static volatile uint8_t a_state = 0;
static volatile uint32_t a_serial_state_start;// __attribute__ ((alligned(8)));

// Idle detect
static volatile uint16_t a_idle = 20001;

// Stats
static volatile uint32_t a_stat_read = 0;
static volatile uint32_t a_stat_timeout_err = 0;
static volatile uint32_t a_stat_frame_err = 0;

#define BITLEN 3333
#define BITVALID 900
#define TONE_REPEATS 2

#define MINSIGNAL 32

// Tape send (Yes I know this can be done a lot simpler!!!)
// For 1850Hz we use the entire wave, for 2400 we skip #2 and #8 from the waveform
static const int16_t a_waveform[] = {5,10,15,10,5,0,-5,-10,-15,-10,-5};
static volatile int8_t a_waveform_index = 0;
static volatile int8_t a_current_tone = 0;

static volatile int8_t a_send_index = 0;
static volatile int8_t a_send_char = '+';
static volatile uint32_t a_serial_send_start;
static volatile uint8_t a_send_idle = 0; //Idle has three states 2:tone off, 1:lead in, 0: data
static volatile uint32_t a_send_idle_counter = 80000;

static volatile uint8_t a_serial_write_buffer[8];
static volatile uint8_t a_serial_write_floor = 0 ;
static volatile uint8_t a_serial_write_ceil = 0;

static volatile uint8_t a_sending = 0;


// Virtual serial port code
//
static uint8_t a_serial_write_avail()
{
  if (a_serial_write_ceil < a_serial_write_floor)
    return(a_serial_write_floor - a_serial_write_ceil -1);
  else
    return(7 - a_serial_write_ceil + a_serial_write_floor);
}

static uint8_t a_serial_write(uint8_t a_char)
{ 
  if (a_serial_write_avail() == 0)
    return 0;
  a_serial_write_buffer[a_serial_write_ceil++] = a_char;
  a_serial_write_ceil &= 0x7;
  return 1;
}

static void a_serial_store(uint8_t a_char)
{
  a_serial_buffer[a_serial_ceil++] = a_char;
  a_serial_ceil &= 0x7;
}

static uint8_t a_serial_available()
{
  if (a_serial_ceil < a_serial_floor)
    return(8 - a_serial_floor + a_serial_ceil);
  else
    return(a_serial_ceil - a_serial_floor);
}

static uint8_t a_serial_read()
{
  if (a_serial_ceil != a_serial_floor) {
    uint8_t ret;
    ret = a_serial_buffer[a_serial_floor++];
    a_serial_floor &= 0x7;
    return ret;
  }
  return 0;
}

volatile uint8_t det = 0; 

// Tape encoder/decoder timer interrupt routine
//
void tapeUpdate()
{
  uint32_t utime;
  ain_last=ain_current;
//  ain_current = (ain_last + analogRead(A22) )/ 2;
  ain_current = analogRead(A22);
  utime=micros();

  // Idle detection 
  if (!a_sending && ain_current < 512+MINSIGNAL && ain_current > 512-MINSIGNAL) {
    if(a_idle > 20000){
      tapeTimer.update(2000000);
    }
    else
      a_idle++;
  }
  else {
    if(a_idle > 20000){
      tapeTimer.update(50);
    }
    a_idle = 0;
  }

  //detect every rising edge 
  if (ain_current < 512-MINSIGNAL*2)
    det=0;

  if (det == 0 && ain_current > 512) {
    rise_wave_period = utime - last_rise_millis;
    last_rise_millis = utime;
    det=1;
  }
  else {
    goto decode; 
  }

  //Decoder: Frequencies
  //There are 2 tones: 1850Hz and 2400Hz (541ms and 417ms period time)
  if (rise_wave_period >= 350) {
   //Valid Tone
    if (rise_wave_period >= 500 && rise_wave_period < 650){
      //Low Tone
      low_repeat++;
      high_repeat=0;
    } else {
      //High
      high_repeat++;
      low_repeat=0;
    }
  }
 
  if (low_repeat == TONE_REPEATS ) {
     a_level = 1;
  }
  else if (high_repeat == TONE_REPEATS ) {
    a_level = 0;
  }
decode:
  // Hardcoded 300 baud 8N1 decoding
  uint32_t mtime;
  mtime = utime - a_serial_state_start;
  
  //Find startbit
  if (a_state == 0) {
    if (a_lastlevel == 0 && a_level == 1) {
      a_serial_state_start = utime;
      a_state=1;
    }
  }
  else if (a_state > 0 && mtime > 10*BITLEN ) {
    //Timeout/invalid we really should never get here
    if (a_state < 10)
      a_stat_timeout_err++;
    a_state = 0;
    a_char = 0;
  }
  else if (a_state == 10 && mtime < 10*BITLEN ) {
    //Time's up either we've got a char, or we didn't
    a_state = 0;
    a_char = 0;
  }
  else if (a_state == 9 && mtime > 9*BITLEN + BITVALID) {
    //Stop bit (we test for that a couple of times)
    if (a_level == 0) {
      //store achar
      a_serial_store(a_char);
      //Serial.print((char)a_char);
      a_stat_read++;
    }
    else {
      //Stop Bit Error
      a_stat_frame_err++;
    }
    a_state = 10;
  }
  else if (a_state < 9 && mtime > 8*BITLEN + BITVALID) {
    //7 bit
    if (a_level == 0) {
      a_char |= 1<<7;
    }
    a_state = 9;
  }
  else if (a_state < 8 && mtime > 7*BITLEN + BITVALID) {
    //6 bit
    if (a_level == 0) {
      a_char |= 1<<6;
    }
    a_state = 8;
  }
  else if (a_state < 7 && mtime > 6*BITLEN + BITVALID) {
    //5 bit
    if (a_level == 0) {
      a_char |= 1<<5;
    }
    a_state = 7;
  }
  else if (a_state < 6 && mtime > 5*BITLEN + BITVALID) {
    //4 bit
    if (a_level == 0) {
      a_char |= 1<<4;
    }
    a_state = 6;
  }
  else if (a_state < 5 && mtime > 4*BITLEN + BITVALID) {
    //3 bit
    if (a_level == 0) {
      a_char |= 1<<3;
    }
    a_state = 5;
  }
  else if (a_state < 4 && mtime > 3*BITLEN + BITVALID) {
    //2 bit
    if (a_level == 0) {
      a_char |= 1<<2;
    }
    a_state = 4;
  }
  else if (a_state < 3 && mtime > 2*BITLEN + BITVALID) {
    //1 bit
    if (a_level == 0) {
      a_char |= 1<<1;
    }
    a_state = 3;
  }
  else if (a_state < 2 && mtime > 1*BITLEN + BITVALID) {
    //0 bit
    if (a_level == 0) {
      a_char |= 1<<0;
    }
    a_state = 2;
  }

  a_lastlevel = a_level;

  //---------------------------
  //Send Part of the IRQ
  
  if (a_sending) {
    if (a_send_idle == 0) {
      if (micros() > a_serial_send_start + BITLEN) {
        if (a_send_index == 10) {
          //restart (next char)
          a_send_index = 0;
          if (a_serial_write_ceil != a_serial_write_floor) {
            a_send_char = a_serial_write_buffer[a_serial_write_floor++];
            a_serial_write_floor &= 0x7;
          }
          else {
            //no next char - go to idle timeout
            a_send_idle = 2;
          }
        }
        if (a_send_index == 9) {
          //Stop_bit
          a_current_tone = 1;  
        }
        else if (a_send_index == 0) {
          //Start_bit
          a_current_tone = 0;
        }
        else {
          //data
          if (a_send_char & 1<<(a_send_index - 1))
            a_current_tone = 1;
          else
            a_current_tone = 0;
        }
        a_send_index++;
        a_serial_send_start = micros();
      }
    }
    else if (a_send_idle == 1) {
      // doing the lead-in tone
      a_current_tone = 1;
      a_send_idle_counter--;
      if (a_send_idle_counter == 0)
        a_send_idle=0;
    } 
    else if (a_send_idle == 2) {
      // doing the lead-out tone
      a_current_tone = 1;
      a_send_idle_counter++;

      // Go back to sending if there is a char in the buffer
      if (a_serial_write_ceil != a_serial_write_floor) {
        a_send_char = a_serial_write_buffer[a_serial_write_floor++];
        a_serial_write_floor &= 0x7;
        a_send_idle_counter = 0;
        a_send_idle = 0;
      }

      // If we time out, go to full idle mode
      if (a_send_idle_counter == 80000)
        a_send_idle=3;
    }   

    // If we are not full idle then generate tones
    if (a_send_idle != 3) {
      //Do the waveform generation
      if (a_waveform_index >= 11) {
        a_waveform_index = 0;
      }
      else {
        if (a_current_tone == 1) {
          //For the high frequency waveform, we skip 3 elements 
          if(a_waveform_index == 2 || a_waveform_index == 5 || a_waveform_index == 8)
            a_waveform_index++;
        }
      }
    
      analogWrite(A21,2048+(a_waveform[a_waveform_index++] * 150));
    }
    else {
      //Done with sending leave sending mode
      a_sending = 0;
      a_idle = 0;
      analogWrite(A21,2048);
    }
  }
  else {
    //Not sending, if there is a new char in the buffer, get it, start sending
    if (a_serial_write_ceil != a_serial_write_floor) {
      a_send_char = a_serial_write_buffer[a_serial_write_floor++];
      a_serial_write_floor &= 0x7;
      a_send_index = 0;
      a_sending = 1;
      a_send_idle = 1;
      a_send_idle_counter = 80000;
      tapeTimer.update(50);
    }
  }
}

