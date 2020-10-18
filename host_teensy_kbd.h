/*
 * This file is part of uLibx.
 *
 * Copyright (C) 2018,2020  D.Herrendoerfer
 *
 *   uLibx is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   uLibx is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with uLibx.  If not, see <http://www.gnu.org/licenses/>.
 *
*/

/* Note: This is just for the Altair8800Again shield 
 *       the pins for clk and data are routed through a driver  */

static volatile uint8_t uKbd_buffer[16]; 
static volatile uint8_t uKbd_ptr_floor = 0;
static volatile uint8_t uKbd_ptr_ceil = 0;

static volatile uint8_t uKbd_ignoreirq = 0;

static uint8_t uKbd_clkpin_read = 39; 
static uint8_t uKbd_clkpin_oe = 38; 
static uint8_t uKbd_clkpin_write = 37; 

static uint8_t uKbd_datapin_read = 36; 
static uint8_t uKbd_datapin_oe = 35; 
static uint8_t uKbd_datapin_write = 34; 

static volatile uint8_t uKbd_parity_err = 0; 
static volatile uint8_t uKbd_stopbit_err = 0; 

// Altair_8800_again specific code
static void uKBD_set_pin_data_output(uint8_t mode)
{
  if (mode) {
    //Output:
    digitalWriteFast(uKbd_datapin_write, HIGH);
    digitalWriteFast(uKbd_datapin_oe,LOW);
    return;
  }
  //Input:
  digitalWriteFast(uKbd_datapin_oe,HIGH);
}
static void uKBD_set_pin_clk_output(uint8_t mode)
{
  if (mode) {
    //Output:
    digitalWriteFast(uKbd_clkpin_write, HIGH);
    digitalWriteFast(uKbd_clkpin_oe,LOW);
    return;
  }
  //Input:
  digitalWriteFast(uKbd_clkpin_oe,HIGH);
}

void uKBD_irq()
{
  static uint8_t uKbd_seq = 0;
  static uint8_t uKbd_parity = 0;
  static uint8_t uKbd_scancode = 0;
  static uint8_t uKbd_devdata = 0;
  static uint8_t uKbd_lastdata = 1;
  
  static uint32_t uKbd_tm;
  static uint32_t uKbd_tm_prev = 0;

  if (uKbd_ignoreirq)
    return;
    
  uKbd_tm=millis();

  uKbd_devdata = digitalReadFast(uKbd_datapin_read);
    
  // Timeout catch
  if ( uKbd_tm - uKbd_tm_prev > 100) {
    uKbd_seq = 0;
    uKbd_scancode = 0;
    uKbd_parity = 0;
  }

  switch(uKbd_seq) {
  case 0: // Start bit
    if (uKbd_lastdata && uKbd_devdata==0)
      uKbd_seq++;
    break;  
  case 1: // Bit0
  case 2: // Bit1
  case 3: // Bit2
  case 4: // Bit3
  case 5: // Bit4
  case 6: // Bit5
  case 7: // Bit6
  case 8: // Bit7
    if (uKbd_devdata) {
      uKbd_parity++;
      uKbd_scancode |= (1 << (uKbd_seq-1));
    }
    uKbd_seq++;
    break;
  case 9: // Parity (Odd)
    if (uKbd_devdata) 
      uKbd_parity++;
    if (uKbd_parity & 0x01 == 0) {
      // Parity error

      uKBD_set_pin_clk_output(1);
      digitalWriteFast(uKbd_clkpin_write, LOW);
      delayMicroseconds(50);
      uKBD_set_pin_clk_output(0);
      
      uKbd_seq = 0;
      uKbd_scancode = 0;
      uKbd_parity = 0;
      uKbd_parity_err++;
    }
    else {
      uKbd_seq++;
    }
    break;
  case 10: // Stop bit
    if (uKbd_devdata) {
      uKbd_buffer[uKbd_ptr_ceil++] = uKbd_scancode;
      uKbd_ptr_ceil = uKbd_ptr_ceil & 0xF; 
    }
    else {
      uKbd_stopbit_err++;  
    }
    uKbd_seq = 0;
    uKbd_scancode = 0;
    uKbd_parity = 0;
    break;
  }

  uKbd_lastdata = uKbd_devdata;
  uKbd_tm_prev = uKbd_tm;
  return;  
}

inline uint8_t wait_while_timeout(uint8_t port, boolean state, uint16_t timeout)
{
  uint32_t end = millis() + timeout;
  while(digitalReadFast(port) == state)
  {
    if (millis() > end)
      return(1);
  }
  return 0;
}

inline void wait_while(uint8_t port, boolean state)
{
  while(digitalReadFast(port) == state);
}

static uint8_t uKbd_send_byte(uint8_t data)
{
  uint8_t parity=0;  

  uKbd_ignoreirq = 1;

  uKBD_set_pin_data_output(1);
  uKBD_set_pin_clk_output(1);

  //Start
  digitalWrite(uKbd_clkpin_write, HIGH);
  delayMicroseconds(100);
  digitalWrite(uKbd_datapin_write, LOW);

  uKBD_set_pin_clk_output(0);

  if (wait_while_timeout(uKbd_clkpin_read, HIGH, 100))
    return(1);
  
  //Data
  for (int i=0; i<8; i++) {
    if (data & (1<<i)) {
      digitalWrite(uKbd_datapin_write, HIGH);
      parity++;
    }
    else {
      digitalWrite(uKbd_datapin_write, LOW);
    }
    // Ack from kbd
    wait_while(uKbd_clkpin_read, LOW);
    wait_while(uKbd_clkpin_read, HIGH);
  }

  //Parity (Odd)
  if (parity & 0x00 != 0) {
    digitalWrite(uKbd_datapin_write, HIGH);
  }
  else {
    digitalWrite(uKbd_datapin_write, LOW);
  }
  // Ack from kbd
  wait_while(uKbd_clkpin_read,LOW);

  //Final Ack check
  uKBD_set_pin_data_output(0);
  wait_while(uKbd_datapin_read,HIGH);
  wait_while(uKbd_clkpin_read,HIGH);
  //Wait for release
  wait_while(uKbd_datapin_read,LOW);
  wait_while(uKbd_clkpin_read,LOW);  

  uKbd_ignoreirq = 0;

  return(0);
}

uint8_t uKbd_available()
{
  if (uKbd_ptr_ceil < uKbd_ptr_floor)
    return(16 - uKbd_ptr_floor + uKbd_ptr_ceil);
  else
    return(uKbd_ptr_ceil - uKbd_ptr_floor);
}

uint8_t uKbd_read()
{
  uint8_t ret;

  if (uKbd_ptr_ceil != uKbd_ptr_floor) {
    ret = uKbd_buffer[uKbd_ptr_floor++];
    uKbd_ptr_floor = uKbd_ptr_floor & 0xF;
    return ret;
  }
  
  while (uKbd_ptr_ceil == uKbd_ptr_floor);
  ret = uKbd_buffer[uKbd_ptr_floor++];
  uKbd_ptr_floor = uKbd_ptr_floor & 0xF;
  return ret;
}

int uKbd_start()
{
  uint8_t ret;
  
  pinMode(uKbd_clkpin_read, INPUT);
  pinMode(uKbd_datapin_read, INPUT);
  pinMode(uKbd_clkpin_write, OUTPUT);
  pinMode(uKbd_datapin_write, OUTPUT);
  pinMode(uKbd_clkpin_oe, OUTPUT);
  pinMode(uKbd_datapin_oe, OUTPUT);

  uKBD_set_pin_data_output(0);
  uKBD_set_pin_clk_output(0);

  attachInterrupt(digitalPinToInterrupt(uKbd_clkpin_read), uKBD_irq, FALLING);
  
  // The Keyboard needs at least 1500ms to start up from power up;
  while(millis()<1750);

  // Empty the input buffer 
  while(uKbd_available()){
    ret=uKbd_read();
    delay(250);
  }
  // send reset
  if(uKbd_send_byte(0xFF))
    return(1); // Keyboard does not talk to us :-(

  //Some keyboards also need this
  uKbd_send_byte(0xFA);

  return (0);
}

static uint8_t uKbd_send_command(uint8_t cmd, uint8_t data)
{ 
  uint8_t ret;
  uKbd_send_byte(cmd);

  ret=uKbd_read();
  if (ret == 0xFA)
    uKbd_send_byte(data);
  
  ret=uKbd_read();
  if (ret == 0xFA)
    return 0;

  return 1;
}

uint8_t uKbd_lights(uint8_t set)
{
   return uKbd_send_command(0xED, (set & 0x07));
}


