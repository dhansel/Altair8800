/**
 ** soft_uart library
 ** Copyright (C) 2015
 **
 **   Antonio C. Domínguez Brito <adominguez@iusiani.ulpgc.es>
 **     División de Robótica y Oceanografía Computacional <www.roc.siani.es>
 **     and Departamento de Informática y Sistemas <www.dis.ulpgc.es>
 **     Universidad de Las Palmas de Gran  Canaria (ULPGC) <www.ulpgc.es>
 **  
 ** This file is part of the soft_uart library.
 ** The soft_uart library is free software: you can redistribute it and/or modify
 ** it under  the  terms of  the GNU  General  Public  License  as  published  by
 ** the  Free Software Foundation, either  version  3  of  the  License,  or  any
 ** later version.
 ** 
 ** The  soft_uart library is distributed in the hope that  it  will  be  useful,
 ** but   WITHOUT   ANY WARRANTY;   without   even   the  implied   warranty   of
 ** MERCHANTABILITY or FITNESS FOR A PARTICULAR  PURPOSE.  See  the  GNU  General
 ** Public License for more details.
 ** 
 ** You should have received a copy  (COPYING file) of  the  GNU  General  Public
 ** License along with the soft_uart library.
 ** If not, see: <http://www.gnu.org/licenses/>.
 **/
/*
 * File: soft_uart.h 
 * Description:  This  is  an  implementation  of  a  software  UART  (Universal
 * Asynchronous Receiver  Transmitter)  library  for  the  Arduino  Due's  Atmel
 * ATSAM3X8E micro-controller.
 * Date: June 22nd, 2015
 * Author: Antonio C. Dominguez-Brito <adominguez@iusiani.ulpgc.es>
 * ROC-SIANI - Universidad de Las Palmas de Gran Canaria - Spain
 */

/**
 ** Modifications by David Hansel on 11/2/17:
 ** - Added "set_rx_handler" function which sets a call-back handler to be called
 **   after a byte of data has been received. Note that the call-back will be
 **   invoked from within an interrupt handler so it should be coded to finish
 **   quickly.
 ** - Added "begin" function that takes the same "config" argument as other
 **   Serial.begin functions to specify number of bits, parity and stop bits.
 ** - In function config (lines 244-277) changed NUM_DIGITAL_PINS to 74
 **   to allow using the (otherwise unused) pins connected to the RX and TX 
 **   lights between the two USB connectors as a serial interface
 **   (those are digital pins 72 and 73 but NUM_DIGITAL_PINS is defined as 72)
 **/

#ifndef SOFT_UART_H
#define SOFT_UART_H

#include "Arduino.h"

#include <cstdint>
#include <type_traits>

#include "fifo.h"

#define serial_tc_declaration(id,rx_length,tx_length) \
void TC##id##_Handler(void) \
{ \
  uint32_t status=TC_GetStatus( \
    arduino_due::soft_uart::tc_timer_table[ \
      static_cast<uint32_t>( \
        arduino_due::soft_uart::timer_ids::TIMER_TC##id \
      ) \
    ].tc_p, \
    arduino_due::soft_uart::tc_timer_table[ \
      static_cast<uint32_t>( \
        arduino_due::soft_uart::timer_ids::TIMER_TC##id \
      ) \
    ].channel \
  ); \
  \
  arduino_due::soft_uart::uart< \
    arduino_due::soft_uart::timer_ids::TIMER_TC##id, \
    rx_length, \
    tx_length \
  >::tc_interrupt(status); \
} \
\
typedef arduino_due::soft_uart::serial< \
  arduino_due::soft_uart::timer_ids::TIMER_TC##id, \
  rx_length, \
  tx_length \
> serial_tc##id##_t; \
\
serial_tc##id##_t serial_tc##id;

#define serial_tc0_declaration(rx_length,tx_length) \
serial_tc_declaration(0,rx_length,tx_length)

#define serial_tc1_declaration(rx_length,tx_length) \
serial_tc_declaration(1,rx_length,tx_length)

#define serial_tc2_declaration(rx_length,tx_length) \
serial_tc_declaration(2,rx_length,tx_length)

#define serial_tc3_declaration(rx_length,tx_length) \
serial_tc_declaration(3,rx_length,tx_length)

#define serial_tc4_declaration(rx_length,tx_length) \
serial_tc_declaration(4,rx_length,tx_length)

#define serial_tc5_declaration(rx_length,tx_length) \
serial_tc_declaration(5,rx_length,tx_length)

#define serial_tc6_declaration(rx_length,tx_length) \
serial_tc_declaration(6,rx_length,tx_length)

#define serial_tc7_declaration(rx_length,tx_length) \
serial_tc_declaration(7,rx_length,tx_length)

#define serial_tc8_declaration(rx_length,tx_length) \
serial_tc_declaration(8,rx_length,tx_length)

namespace arduino_due
{

  namespace soft_uart
  {
    enum class timer_ids: uint32_t
    {
      TIMER_TC0=0,
      TIMER_TC1=1,
      TIMER_TC2=2,
      TIMER_TC3=3,
      TIMER_TC4=4,
      TIMER_TC5=5,
      TIMER_TC6=6,
      TIMER_TC7=7,
      TIMER_TC8=8,
      TIMER_IDS
    };

    enum class mode_codes: int32_t
    {
      INVALID_MODE=-1,
      FULL_DUPLEX=0,
      RX_MODE=1,
      TX_MODE=2
    };

    enum default_pins: uint32_t
    {
      DEFAULT_RX_PIN=2,
      DEFAULT_TX_PIN=3
    };

    enum bit_rates: uint32_t
    {
      MIN_BIT_RATE=75,
      MAX_BIT_RATE=115200,
      DEFAULT_BIT_RATE=9600
    };

    enum class return_codes: int32_t
    {
      EVERYTHING_OK=0,
      BAD_BIT_RATE_ERROR=-1,
      BAD_RX_PIN=-2,
      BAD_TX_PIN=-3,
      BAD_HALF_DUPLEX_PIN=-4
    };

    enum class data_bit_codes: uint32_t
    {
      FIVE_BITS=5,
      SIX_BITS=6,
      SEVEN_BITS=7,
      EIGHT_BITS=8,
      NINE_BITS=9,
    };

    enum class parity_codes: uint32_t
    {
      NO_PARITY=0,
      EVEN_PARITY=1,
      ODD_PARITY=2
    };

    enum class stop_bit_codes: uint32_t
    {
      ONE_STOP_BIT=1,
      TWO_STOP_BITS=2
    };

    enum class rx_status_codes: uint32_t
    {
      LISTENING,
      RECEIVING
    };

    enum rx_data_status_codes: uint32_t
    {
      NO_DATA_AVAILABLE=0,
      DATA_AVAILABLE=1,
      DATA_LOST=2,
      BAD_START_BIT=4,
      BAD_PARITY=8,
      BAD_STOP_BIT=16,
    };

    enum class tx_status_codes: uint32_t
    {
      IDLE,
      SENDING
    };

    struct tc_timer_data
    { 
      Tc* tc_p; 
      uint32_t channel;
      IRQn_Type irq;
    };

    extern tc_timer_data 
      tc_timer_table[static_cast<uint32_t>(timer_ids::TIMER_IDS)];
    
    template<
      timer_ids TIMER,
      size_t RX_BUFFER_LENGTH,
      size_t TX_BUFFER_LENGTH
    > class uart 
    {
      static_assert(TIMER<timer_ids::TIMER_IDS,"[ERROR] Bad TC id provided to instantiate template uart");

      public:

        uart() { _mode_=mode_codes::INVALID_MODE; }

        ~uart() { end(); }
  
        uart(const uart&) = delete;
        uart(uart&&) = delete;
        uart& operator=(const uart&) = delete;
        uart& operator=(uart&&) = delete;

        return_codes config(
          uint32_t rx_pin = default_pins::DEFAULT_RX_PIN,
          uint32_t tx_pin = default_pins::DEFAULT_TX_PIN,
          uint32_t bit_rate = bit_rates::DEFAULT_BIT_RATE,
          data_bit_codes the_data_bits = data_bit_codes::EIGHT_BITS,
          parity_codes the_parity = parity_codes::EVEN_PARITY,
          stop_bit_codes the_stop_bits = stop_bit_codes::ONE_STOP_BIT
        ) 
        {
          _mode_=mode_codes::INVALID_MODE;
          
          if(rx_pin>=74)
            return return_codes::BAD_RX_PIN;

          if(tx_pin>=74)
            return return_codes::BAD_TX_PIN;

          return_codes ret_code=
            _ctx_.config(
              rx_pin,
              tx_pin,
              bit_rate,
              the_data_bits,
              the_parity,
              the_stop_bits
            ); 

          if(ret_code!=return_codes::EVERYTHING_OK) return ret_code;
                  
          // cofigure tx pin
          pinMode(tx_pin,OUTPUT);
          digitalWrite(tx_pin,HIGH);

          // configure & attatch interrupt on rx pin
          pinMode(rx_pin,INPUT_PULLUP);
          attachInterrupt(rx_pin,uart::rx_interrupt,CHANGE);

          _ctx_.enable_rx_interrupts(); 
          _mode_=mode_codes::FULL_DUPLEX;

          return return_codes::EVERYTHING_OK;
        }
  
        return_codes half_duplex_config(
          uint32_t rx_tx_pin = default_pins::DEFAULT_RX_PIN,
          uint32_t bit_rate = bit_rates::DEFAULT_BIT_RATE,
          data_bit_codes the_data_bits = data_bit_codes::EIGHT_BITS,
          parity_codes the_parity = parity_codes::EVEN_PARITY,
          stop_bit_codes the_stop_bits = stop_bit_codes::ONE_STOP_BIT,
          bool in_rx_mode = true
        ) 
        {
          _mode_=mode_codes::INVALID_MODE;

          if(rx_tx_pin>=NUM_DIGITAL_PINS)
            return return_codes::BAD_HALF_DUPLEX_PIN;

          return_codes ret_code=
            _ctx_.config(
              rx_tx_pin,
              rx_tx_pin,
              bit_rate,
              the_data_bits,
              the_parity,
              the_stop_bits
            ); 

          if(ret_code!=return_codes::EVERYTHING_OK) return ret_code;

          if(in_rx_mode)
          {
            // configure & attatch interrupt on rx pin
            pinMode(rx_tx_pin,INPUT_PULLUP);
            attachInterrupt(rx_tx_pin,uart::rx_interrupt,CHANGE);
            _ctx_.enable_rx_interrupts(); 
            _mode_=mode_codes::RX_MODE;
          }
          else
          {
            // cofigure tx pin
            pinMode(rx_tx_pin,OUTPUT);
            digitalWrite(rx_tx_pin,HIGH);
            _mode_=mode_codes::TX_MODE;
          }

          return return_codes::EVERYTHING_OK;
        }

        mode_codes get_mode() { return _mode_; }

        bool set_rx_mode()
        {
          if( 
            (_mode_==mode_codes::INVALID_MODE) ||
            (_mode_==mode_codes::FULL_DUPLEX)
          ) return false;

          if(_mode_==mode_codes::RX_MODE) return true; 
          flush();

          pinMode(_ctx_.rx_pin,INPUT_PULLUP);
          attachInterrupt(_ctx_.rx_pin,uart::rx_interrupt,CHANGE);
          
          _mode_=mode_codes::RX_MODE;
          _ctx_.enable_rx_interrupts(); 
          return true;
        }

        bool set_tx_mode()
        {
          if( 
            (_mode_==mode_codes::INVALID_MODE) ||
            (_mode_==mode_codes::FULL_DUPLEX)
          ) return false;

          if(_mode_==mode_codes::TX_MODE) return true;

          while(true)
          {
            _ctx_.disable_rx_interrupts();
            if(_ctx_.rx_status==rx_status_codes::LISTENING) break;
            _ctx_.enable_rx_interrupts();
          }

          detachInterrupt(_ctx_.rx_pin);
          pinMode(_ctx_.tx_pin,OUTPUT);
          digitalWrite(_ctx_.tx_pin,HIGH);

          _mode_=mode_codes::TX_MODE;
          return true;
        }

        void end() { _ctx_.end(); }

        int available() { return _ctx_.available(); }

        static void tc_interrupt(uint32_t the_status)
        { _ctx_.tc_interrupt(the_status); }

        static void rx_interrupt() { _ctx_.rx_interrupt(); }

        void set_rx_handler(void (*h)(void)) { _ctx_.rx_handler = h; }

        timer_ids get_timer() { return TIMER; }

        size_t get_rx_buffer_length() { return RX_BUFFER_LENGTH; }
        size_t get_tx_buffer_length() { return TX_BUFFER_LENGTH; }
  
        uint32_t get_bit_rate() { return _ctx_.bit_rate; }
        double get_bit_time() { return _ctx_.bit_time; }  
        double get_frame_time() { return _ctx_.frame_time; }  

        uint32_t get_rx_data(uint32_t& data) 
        { 
          return (
            (
              (_mode_==mode_codes::FULL_DUPLEX) ||
              (_mode_==mode_codes::RX_MODE)
            )? 
              _ctx_.get_rx_data(data): 
              rx_data_status_codes::NO_DATA_AVAILABLE
          );
        }

        bool data_available(uint32_t status)
        { return _ctx_.data_available(status); }

        bool data_lost(uint32_t status)
        { return _ctx_.data_lost(status); }

        bool bad_status(uint32_t status) 
        { return _ctx_.bad_status(status); }
        
        bool bad_start_bit(uint32_t status) 
        { return _ctx_.bad_start_bit(status); }
        
        bool bad_parity(uint32_t status) 
        { return _ctx_.bad_parity(status); }

        bool bad_stop_bit(uint32_t status) 
        { return _ctx_.bad_stop_bit(status); }

        // is TX buffer full?
        bool is_tx_full() 
        { 
          return (
            (
              (_mode_==mode_codes::FULL_DUPLEX) ||
              (_mode_==mode_codes::TX_MODE)
            )? _ctx_.is_tx_full(): false
          ); 
        }

        // is TX buffer full?
        bool available_for_write() 
        { 
          return (
            (
              (_mode_==mode_codes::FULL_DUPLEX) ||
              (_mode_==mode_codes::TX_MODE)
            )? _ctx_.available_for_write(): 0 
          ); 
        }

        // NOTE: data is 5, 6, 7, 8 or 9 bits length
        bool set_tx_data(uint32_t data) 
        { 
          return (
            (
              (_mode_==mode_codes::FULL_DUPLEX) ||
              (_mode_==mode_codes::TX_MODE)
            )? _ctx_.set_tx_data(data): false
          ); 
        }

        tx_status_codes get_tx_status() { return _ctx_.get_tx_status(); }

        void flush() { _ctx_.flush(); }

        void flush_rx() { _ctx_.flush_rx(); }

      private:

        struct _uart_ctx_
        {

          return_codes config(
            uint32_t the_rx_pin,
            uint32_t the_tx_pin,
            uint32_t the_bit_rate,
            data_bit_codes the_data_bits,
            parity_codes the_parity,
            stop_bit_codes the_stop_bits
          );

          void end()
          {
            flush();

            disable_tc_interrupts(); disable_rx_interrupts();

            stop_tc_interrupts(); detachInterrupt(rx_pin);

            pmc_disable_periph_clk(uint32_t(timer_p->irq));
          }

          void tc_interrupt(uint32_t the_status);
          void rx_interrupt();

          uint32_t get_rx_data(uint32_t& data);

          int available()
          {
            register int any;
            disable_tc_interrupts();
            any=rx_buffer.items();
            enable_tc_interrupts();

            return any;
          }

          bool data_available(uint32_t status)
          { 
            return (
              status&(
                rx_data_status_codes::DATA_AVAILABLE|
                rx_data_status_codes::DATA_LOST
              )
            ); 
          }

          bool data_lost(uint32_t status)
          { return (status&rx_data_status_codes::DATA_LOST); }

          bool bad_status(uint32_t status)
          {
            return (
              status&(
                rx_data_status_codes::BAD_START_BIT|
                rx_data_status_codes::BAD_PARITY|
                rx_data_status_codes::BAD_STOP_BIT
              )
            );
          }

          bool bad_start_bit(uint32_t status)
          { return (status&rx_data_status_codes::BAD_START_BIT); }
          
          bool bad_parity(uint32_t status)
          { return (status&rx_data_status_codes::BAD_PARITY); }
          
          bool bad_stop_bit(uint32_t status)
          { return (status&rx_data_status_codes::BAD_STOP_BIT); }

          // is TX buffer full?
          bool is_tx_full() 
          { 
            disable_tc_interrupts();
            auto full=tx_buffer.is_full();
            enable_tc_interrupts();
            return full;
          }

          int available_for_write() 
          { 
            disable_tc_interrupts();
            auto items=tx_buffer.available();
            enable_tc_interrupts();
            return items;
          }

          // NOTE: only the 5, 6, 7, 8  or 9 lowest significant bits
          // of data are send
          bool set_tx_data(uint32_t data);

          void flush()
          {
            // wait until sending everything
            while(tx_status!=tx_status_codes::IDLE) 
            { /*nothing */ }
          }

          void flush_rx()
          {
            disable_tc_interrupts();
            rx_buffer.reset();
            enable_tc_interrupts();
          }

          tx_status_codes get_tx_status() { return tx_status; }

          uint32_t get_even_parity(uint32_t data,uint32_t bits)
          {
            uint32_t odd_parity=data&1;
            for(uint32_t bit=1; bit<bits; bit++)
              odd_parity=odd_parity^((data>>bit)&1);
            return odd_parity;
          }

          void enable_rx_interrupts() { NVIC_EnableIRQ(rx_irq); }
          void disable_rx_interrupts() { NVIC_DisableIRQ(rx_irq); }

          void config_tc_interrupt() { NVIC_SetPriority(timer_p->irq,0); }

          void enable_tc_interrupts() { NVIC_EnableIRQ(timer_p->irq); }
          void disable_tc_interrupts() { NVIC_DisableIRQ(timer_p->irq); }

          void start_tc_interrupts()
          {
            NVIC_ClearPendingIRQ(timer_p->irq);
            NVIC_EnableIRQ(timer_p->irq);
            TC_Start(timer_p->tc_p,timer_p->channel);
          }

          void stop_tc_interrupts()
          {
            NVIC_DisableIRQ(timer_p->irq);
            TC_Stop(timer_p->tc_p,timer_p->channel);
          }

          void enable_tc_ra_interrupt()
          {
            timer_p->tc_p->TC_CHANNEL[timer_p->channel].TC_IER=
              TC_IER_CPAS;
          }

          bool is_enabled_ra_interrupt()
          {
            return (
              timer_p->tc_p->TC_CHANNEL[timer_p->channel].TC_IMR &
              TC_IMR_CPAS
            );
          }

          void disable_tc_ra_interrupt()
          {
            timer_p->tc_p->TC_CHANNEL[timer_p->channel].TC_IDR=
              TC_IDR_CPAS;
          }

          void enable_tc_rc_interrupt()
          {
            timer_p->tc_p->TC_CHANNEL[timer_p->channel].TC_IER=
              TC_IER_CPCS;
          }

          bool is_enabled_rc_interrupt()
          {
            return (
              timer_p->tc_p->TC_CHANNEL[timer_p->channel].TC_IMR &
              TC_IMR_CPCS
            );
          }

          void disable_tc_rc_interrupt()
          {
            timer_p->tc_p->TC_CHANNEL[timer_p->channel].TC_IDR=
              TC_IDR_CPCS;
          }

          void get_incoming_bit()
          { rx_data |= (rx_bit<<rx_bit_counter); }

          void update_rx_data_buffer()
          {
            rx_data_status=(
              (rx_buffer.push(static_cast<uint32_t>(rx_data)))?
                rx_data_status_codes::DATA_AVAILABLE:
                rx_data_status_codes::DATA_LOST
            );

            if( rx_handler!=NULL ) rx_handler();
          }

          void set_outgoing_bit()
          {
            if((tx_data>>tx_bit_counter) & 1) 
              PIO_Set(tx_pio_p,tx_mask);
            else PIO_Clear(tx_pio_p,tx_mask);
          }

          tc_timer_data* timer_p;
          uint32_t rx_pin;
          Pio* rx_pio_p;
          uint32_t rx_mask;
          IRQn_Type rx_irq;

          uint32_t tx_pin;
          Pio* tx_pio_p;
          uint32_t tx_mask;

          double tc_tick;
          double bit_time;
          double frame_time;
          uint32_t bit_ticks;
          uint32_t bit_1st_half;

          // serial protocol
          uint32_t bit_rate;
          data_bit_codes data_bits;
          parity_codes parity;
          stop_bit_codes stop_bits;
          uint32_t rx_frame_bits;
          uint32_t tx_frame_bits;
          uint32_t parity_bit_pos;
          uint32_t first_stop_bit_pos;
          uint32_t data_mask;
          void (*rx_handler)(void);

          // rx data
          circular_fifo<uint32_t,RX_BUFFER_LENGTH> rx_buffer;
          volatile uint32_t rx_data;
          volatile uint32_t rx_bit_counter;
          volatile uint32_t rx_bit;
          volatile rx_status_codes rx_status;
          volatile uint32_t rx_data_status;
          volatile bool rx_at_end_quarter;

          // tx data
          fifo<uint32_t,TX_BUFFER_LENGTH> tx_buffer;
          volatile uint32_t tx_data;
          volatile uint32_t tx_bit_counter;
          volatile tx_status_codes tx_status;
        };
  
        static _uart_ctx_ _ctx_;

        mode_codes _mode_;
    };

    template<
      timer_ids TIMER,
      size_t RX_BUFFER_LENGTH,
      size_t TX_BUFFER_LENGTH
    > class serial: public HardwareSerial 
    {
      public:

        typedef uart<TIMER,RX_BUFFER_LENGTH,TX_BUFFER_LENGTH> raw_uart;

        serial() 
        { 
          _peek_data_valid_=false; 
          _last_data_status_=rx_data_status_codes::NO_DATA_AVAILABLE;
        }

        serial(const serial&) = delete;
        serial(serial&&) = delete;
        serial& operator=(const serial&) = delete;
        serial& operator=(serial&&) = delete;
        
        void begin(unsigned long baud_rate)
        {
          _tc_uart_.config(
            default_pins::DEFAULT_RX_PIN,
            default_pins::DEFAULT_TX_PIN,
            static_cast<uint32_t>(baud_rate),
            data_bit_codes::EIGHT_BITS,
            parity_codes::NO_PARITY,
            stop_bit_codes::ONE_STOP_BIT
          );
        }
	void begin(uint32_t rx_pin, uint32_t tx_pin, unsigned long baud_rate, unsigned long config)
	{
	  data_bit_codes bits = data_bit_codes::EIGHT_BITS;
	  parity_codes parity = parity_codes::NO_PARITY;
	  stop_bit_codes stop = stop_bit_codes::ONE_STOP_BIT;
          
          switch( config )
            {
            case SERIAL_5N1: bits = data_bit_codes::FIVE_BITS;  parity=parity_codes::NO_PARITY;   stop = stop_bit_codes::ONE_STOP_BIT;  break;
            case SERIAL_6N1: bits = data_bit_codes::SIX_BITS ;  parity=parity_codes::NO_PARITY;   stop = stop_bit_codes::ONE_STOP_BIT;  break;
            case SERIAL_7N1: bits = data_bit_codes::SEVEN_BITS; parity=parity_codes::NO_PARITY;   stop = stop_bit_codes::ONE_STOP_BIT;  break;
            case SERIAL_8N1: bits = data_bit_codes::EIGHT_BITS; parity=parity_codes::NO_PARITY;   stop = stop_bit_codes::ONE_STOP_BIT;  break;
            case SERIAL_5N2: bits = data_bit_codes::FIVE_BITS;  parity=parity_codes::NO_PARITY;   stop = stop_bit_codes::TWO_STOP_BITS; break;
            case SERIAL_6N2: bits = data_bit_codes::SIX_BITS ;  parity=parity_codes::NO_PARITY;   stop = stop_bit_codes::TWO_STOP_BITS; break;
            case SERIAL_7N2: bits = data_bit_codes::SEVEN_BITS; parity=parity_codes::NO_PARITY;   stop = stop_bit_codes::TWO_STOP_BITS; break;
            case SERIAL_8N2: bits = data_bit_codes::EIGHT_BITS; parity=parity_codes::NO_PARITY;   stop = stop_bit_codes::TWO_STOP_BITS; break;
            case SERIAL_5E1: bits = data_bit_codes::FIVE_BITS;  parity=parity_codes::EVEN_PARITY; stop = stop_bit_codes::ONE_STOP_BIT;  break;
            case SERIAL_6E1: bits = data_bit_codes::SIX_BITS ;  parity=parity_codes::EVEN_PARITY; stop = stop_bit_codes::ONE_STOP_BIT;  break;
            case SERIAL_7E1: bits = data_bit_codes::SEVEN_BITS; parity=parity_codes::EVEN_PARITY; stop = stop_bit_codes::ONE_STOP_BIT;  break;
            case SERIAL_8E1: bits = data_bit_codes::EIGHT_BITS; parity=parity_codes::EVEN_PARITY; stop = stop_bit_codes::ONE_STOP_BIT;  break;
            case SERIAL_5E2: bits = data_bit_codes::FIVE_BITS;  parity=parity_codes::EVEN_PARITY; stop = stop_bit_codes::TWO_STOP_BITS; break;
            case SERIAL_6E2: bits = data_bit_codes::SIX_BITS ;  parity=parity_codes::EVEN_PARITY; stop = stop_bit_codes::TWO_STOP_BITS; break;
            case SERIAL_7E2: bits = data_bit_codes::SEVEN_BITS; parity=parity_codes::EVEN_PARITY; stop = stop_bit_codes::TWO_STOP_BITS; break;
            case SERIAL_8E2: bits = data_bit_codes::EIGHT_BITS; parity=parity_codes::EVEN_PARITY; stop = stop_bit_codes::TWO_STOP_BITS; break;
            case SERIAL_5O1: bits = data_bit_codes::FIVE_BITS;  parity=parity_codes::ODD_PARITY;  stop = stop_bit_codes::ONE_STOP_BIT;  break;
            case SERIAL_6O1: bits = data_bit_codes::SIX_BITS ;  parity=parity_codes::ODD_PARITY;  stop = stop_bit_codes::ONE_STOP_BIT;  break;
            case SERIAL_7O1: bits = data_bit_codes::SEVEN_BITS; parity=parity_codes::ODD_PARITY;  stop = stop_bit_codes::ONE_STOP_BIT;  break;
            case SERIAL_8O1: bits = data_bit_codes::EIGHT_BITS; parity=parity_codes::ODD_PARITY;  stop = stop_bit_codes::ONE_STOP_BIT;  break;
            case SERIAL_5O2: bits = data_bit_codes::FIVE_BITS;  parity=parity_codes::ODD_PARITY;  stop = stop_bit_codes::TWO_STOP_BITS; break;
            case SERIAL_6O2: bits = data_bit_codes::SIX_BITS ;  parity=parity_codes::ODD_PARITY;  stop = stop_bit_codes::TWO_STOP_BITS; break;
            case SERIAL_7O2: bits = data_bit_codes::SEVEN_BITS; parity=parity_codes::ODD_PARITY;  stop = stop_bit_codes::TWO_STOP_BITS; break;
            case SERIAL_8O2: bits = data_bit_codes::EIGHT_BITS; parity=parity_codes::ODD_PARITY;  stop = stop_bit_codes::TWO_STOP_BITS; break;
            }

	  _tc_uart_.config(rx_pin, tx_pin, static_cast<uint32_t>(baud_rate), bits, parity, stop);
	}


        void begin(
          uint32_t rx_pin = default_pins::DEFAULT_RX_PIN,
          uint32_t tx_pin = default_pins::DEFAULT_TX_PIN,
          uint32_t bit_rate = bit_rates::DEFAULT_BIT_RATE,
          data_bit_codes the_data_bits = data_bit_codes::EIGHT_BITS,
          parity_codes the_parity = parity_codes::NO_PARITY,
          stop_bit_codes the_stop_bits = stop_bit_codes::ONE_STOP_BIT
        )
        {
          _tc_uart_.config(
            rx_pin,
            tx_pin,
            bit_rate,
            the_data_bits,
            the_parity,
            the_stop_bits
          );
        }

        // NOTE: on function half_duplex_begin() the last function 
        // argument specifies the operation mode: true (RX_MODE, 
        // reception mode, the default) or false (TX_MODE, trans-
        // mission mode)
        return_codes half_duplex_begin(
          uint32_t rx_tx_pin = default_pins::DEFAULT_RX_PIN,
          uint32_t bit_rate = bit_rates::DEFAULT_BIT_RATE,
          data_bit_codes the_data_bits = data_bit_codes::EIGHT_BITS,
          parity_codes the_parity = parity_codes::NO_PARITY,
          stop_bit_codes the_stop_bits = stop_bit_codes::ONE_STOP_BIT,
          bool in_rx_mode = true
        )
        {
          return _tc_uart_.half_duplex_config(
            rx_tx_pin,
            bit_rate,
            the_data_bits,
            the_parity,
            the_stop_bits,
            in_rx_mode
          );
        }

        void end() { _tc_uart_.end(); }

        int available(void) { return _tc_uart_.available(); }

        int availableForWrite(void)
        { return available_for_write(); }

        int available_for_write(void) 
        { return _tc_uart_.available_for_write(); }

        int peek(void) 
        {
          if(_peek_data_valid_) return _last_data_;

          _last_data_status_=_tc_uart_.get_rx_data(_last_data_);

          if(
            !_tc_uart_.data_available(_last_data_status_) ||
            _tc_uart_.bad_status(_last_data_status_)
          ) return -1;

          _peek_data_valid_=true;

          return _last_data_;
        }

        int read(void) 
        {
          if(_peek_data_valid_) 
          { _peek_data_valid_=false; return _last_data_; }

          _last_data_status_=_tc_uart_.get_rx_data(_last_data_);

          if(
            !_tc_uart_.data_available(_last_data_status_) ||
            _tc_uart_.bad_status(_last_data_status_)
          ) return -1;

          return _last_data_; 
        }
        void set_rx_handler(void (*h)(void)) { _tc_uart_.set_rx_handler(h); }


        bool data_lost() { return _tc_uart_.data_lost(_last_data_status_); }

        bool bad_status() { return _tc_uart_.bad_status(_last_data_status_); }
        
        bool bad_start_bit() { return _tc_uart_.bad_start_bit(_last_data_status_); }
        
        bool bad_parity() { return _tc_uart_.bad_parity(_last_data_status_); }
        bool bad_stop_bit() { return _tc_uart_.bad_stop_bit(_last_data_status_); }
        
        void flush(void) { _tc_uart_.flush(); } 
        
        void flushRX(void) { _tc_uart_.flush_rx(); }
        
        size_t write(uint8_t data) 
        {
          while(!available_for_write()) { /* nothing */ }

          return (
              (_tc_uart_.set_tx_data(static_cast<uint32_t>(data)))? 
                1: 0 
          ); 
        } 

        size_t write(uint32_t data) 
        {
          while(!available_for_write()) { /* nothing */ }

          return (
              (_tc_uart_.set_tx_data(data))? 
                1: 0 
          ); 
        }

        mode_codes get_mode() { return _tc_uart_.get_mode(); }

        bool set_rx_mode() { return _tc_uart_.set_rx_mode(); }
        bool set_tx_mode() { return _tc_uart_.set_tx_mode(); }

        uint32_t get_last_data() { return _last_data_; }
        uint32_t get_last_data_status() { return _last_data_status_; }
        double get_bit_time() { return _tc_uart_.get_bit_time(); }
        double get_frame_time() { return _tc_uart_.get_frame_time(); }
        timer_ids get_timer() { return _tc_uart_.get_timer(); }

        using Print::write; // pull in write(str) and write(buf, size) from Print
        operator bool() { return true; } 

      private:

        //uart<TIMER,RX_BUFFER_LENGTH,TX_BUFFER_LENGTH> _tc_uart_;
        raw_uart _tc_uart_;

        uint32_t _last_data_;
        uint32_t _last_data_status_;
        bool _peek_data_valid_;
    };

    template<
      timer_ids TIMER,
      size_t RX_BUFFER_LENGTH,
      size_t TX_BUFFER_LENGTH
    > typename uart<
      TIMER,
      RX_BUFFER_LENGTH,
      TX_BUFFER_LENGTH
    >::_uart_ctx_ 
      uart<TIMER,RX_BUFFER_LENGTH,TX_BUFFER_LENGTH>::_ctx_;

    template<
      timer_ids TIMER,
      size_t RX_BUFFER_LENGTH,
      size_t TX_BUFFER_LENGTH
    > return_codes uart<
      TIMER,
      RX_BUFFER_LENGTH,
      TX_BUFFER_LENGTH
    >::_uart_ctx_::config(
      uint32_t the_rx_pin,
      uint32_t the_tx_pin,
      uint32_t the_bit_rate,
      data_bit_codes the_data_bits,
      parity_codes the_parity,
      stop_bit_codes the_stop_bits
    )
    {
      if(
        (the_bit_rate<bit_rates::MIN_BIT_RATE) || 
        (the_bit_rate>bit_rates::MAX_BIT_RATE)
      ) return return_codes::BAD_BIT_RATE_ERROR;
      rx_handler = NULL;


      timer_p=&(tc_timer_table[static_cast<uint32_t>(TIMER)]);
      bit_rate=the_bit_rate;
      
      // NOTE: we will be using the fastest clock for TC ticks
      // just using a prescaler of 2
      tc_tick=double(1)/double((VARIANT_MCK)>>1);
      bit_time=double(1)/double(bit_rate);
  
      bit_ticks=static_cast<uint32_t>(bit_time/tc_tick);
      bit_1st_half=(bit_ticks>>1);

      data_bits=the_data_bits; parity=the_parity; stop_bits=the_stop_bits;

      // reception frame length in bits
      rx_frame_bits=1 + // the start bit
        static_cast<uint32_t>(data_bits) + // the data bits
        ((parity!=parity_codes::NO_PARITY)? 1: 0) + // the parity?
        1; // for reception we ONLY consider one stop bit

      // transmission frame length in bits
      tx_frame_bits=rx_frame_bits;
      if(stop_bits==stop_bit_codes::TWO_STOP_BITS) 
      {
        // for transmission we DO consider also the second stop bit
        tx_frame_bits++; 
      }

      frame_time=tx_frame_bits*bit_time;

      parity_bit_pos=1 + // the start bit
        static_cast<uint32_t>(data_bits); // the data bits

      first_stop_bit_pos=parity_bit_pos +
        ((parity!=parity_codes::NO_PARITY)? 1: 0); // the parity?

      data_mask=(1<<static_cast<uint32_t>(data_bits))-1;

      rx_pin=the_rx_pin;
      rx_pio_p=g_APinDescription[rx_pin].pPort;
      rx_mask=g_APinDescription[rx_pin].ulPin;
      rx_status=rx_status_codes::LISTENING;
      rx_data_status=rx_data_status_codes::NO_DATA_AVAILABLE;
      rx_buffer.reset();
      rx_at_end_quarter=false;

      rx_irq=(
        (rx_pio_p==PIOA)? 
          PIOA_IRQn:
          (
            (rx_pio_p==PIOB)? 
              PIOB_IRQn: ((rx_pio_p==PIOC)? PIOC_IRQn: PIOD_IRQn)
          )
      );

      tx_pin=the_tx_pin;
      tx_pio_p=g_APinDescription[tx_pin].pPort;
      tx_mask=g_APinDescription[tx_pin].ulPin;
      tx_status=tx_status_codes::IDLE;
      tx_buffer.reset();

      // PMC settings
      pmc_set_writeprotect(0);
      pmc_enable_periph_clk(uint32_t(timer_p->irq));
  
      // timing setings
      TC_Configure(
        timer_p->tc_p,
        timer_p->channel,
        TC_CMR_TCCLKS_TIMER_CLOCK1 |
        TC_CMR_WAVE |
        TC_CMR_WAVSEL_UP_RC
      );
      
      TC_SetRA(timer_p->tc_p,timer_p->channel,bit_1st_half);
      disable_tc_ra_interrupt();

      TC_SetRC(timer_p->tc_p,timer_p->channel,bit_ticks);
      disable_tc_rc_interrupt();

      config_tc_interrupt();
      stop_tc_interrupts();

      return return_codes::EVERYTHING_OK;
    }

    template<
      timer_ids TIMER,
      size_t RX_BUFFER_LENGTH,
      size_t TX_BUFFER_LENGTH
    > void uart<
      TIMER,
      RX_BUFFER_LENGTH,
      TX_BUFFER_LENGTH
    >::_uart_ctx_::tc_interrupt(
      uint32_t the_status
    )
    {
      // RA compare interrupt
      if((the_status & TC_SR_CPAS) && is_enabled_ra_interrupt())
      {
        if(rx_status==rx_status_codes::RECEIVING)
        { 
          if(!rx_at_end_quarter)
          {
            get_incoming_bit();
            if((rx_bit_counter++)==rx_frame_bits-1)  
            {
              if(stop_bits==stop_bit_codes::TWO_STOP_BITS)
                get_incoming_bit();

              disable_tc_ra_interrupt();

              if(tx_status==tx_status_codes::IDLE)
                stop_tc_interrupts();

              update_rx_data_buffer();

              rx_status=rx_status_codes::LISTENING;
            }
          }
          else rx_at_end_quarter=false;
        }
      }
     
      // RC compare interrupt
      if((the_status & TC_SR_CPCS) && is_enabled_rc_interrupt())
      {
        // rx code
        if(
            !is_enabled_ra_interrupt() && 
            (rx_status==rx_status_codes::RECEIVING)
        )
        {
          get_incoming_bit();
          if((rx_bit_counter++)==rx_frame_bits-1)
          {
            if(stop_bits==stop_bit_codes::TWO_STOP_BITS)
              get_incoming_bit();

            if(tx_status==tx_status_codes::IDLE)
            {
              disable_tc_rc_interrupt();
              stop_tc_interrupts(); 
            }

            update_rx_data_buffer();

            rx_status=rx_status_codes::LISTENING;
          }
        }

        // tx code
        if(tx_status==tx_status_codes::SENDING)
        {
          set_outgoing_bit();
          if((tx_bit_counter++)==tx_frame_bits-1) 
          {
            uint32_t data_to_send;
            if(tx_buffer.pop(data_to_send)) 
            { tx_data=data_to_send; tx_bit_counter=0; }
            else
            {
              if(rx_status==rx_status_codes::LISTENING)
              { disable_tc_rc_interrupt(); stop_tc_interrupts(); }
              else
              {
                if(is_enabled_ra_interrupt())
                  disable_tc_rc_interrupt();
              }

              tx_status=tx_status_codes::IDLE;
            }
          }
        }
      }
    }
   
    template<
      timer_ids TIMER,
      size_t RX_BUFFER_LENGTH,
      size_t TX_BUFFER_LENGTH
    > void uart<
      TIMER,
      RX_BUFFER_LENGTH,
      TX_BUFFER_LENGTH
    >::_uart_ctx_::rx_interrupt()
    {
      register uint32_t sampled_bit=
        PIO_Get(rx_pio_p,PIO_INPUT,rx_mask);

      switch(rx_status)
      {
        case rx_status_codes::LISTENING:
          if(!sampled_bit)
          {
            rx_status=rx_status_codes::RECEIVING;
            rx_data=rx_bit_counter=rx_bit=0;
            
            if(tx_status==tx_status_codes::IDLE)
            { 
              enable_tc_ra_interrupt(); 
              start_tc_interrupts(); 
            } 
            else 
            {
              register uint32_t timer_value=
                TC_ReadCV(timer_p->tc_p,timer_p->channel);

              if(timer_value<=(bit_1st_half>>1)) 
                enable_tc_ra_interrupt();
              else
              {
                if(timer_value>bit_1st_half+(bit_1st_half>>1))
                {
                  enable_tc_ra_interrupt();
                  rx_at_end_quarter=true;
                }
              }
            }
          } 
          break;
          
        case rx_status_codes::RECEIVING:
          rx_bit=sampled_bit;
          break;
      }
    }

    template<
      timer_ids TIMER,
      size_t RX_BUFFER_LENGTH,
      size_t TX_BUFFER_LENGTH
    > uint32_t uart<
      TIMER,
      RX_BUFFER_LENGTH,
      TX_BUFFER_LENGTH
    >::_uart_ctx_::get_rx_data(
      uint32_t& data
    )
    {
      register uint32_t status;
      register bool not_empty;
      uint32_t data_received;

      disable_tc_interrupts();
      
      if(!(not_empty=rx_buffer.pop(data_received)))
        rx_data_status=rx_data_status_codes::NO_DATA_AVAILABLE;
      status=rx_data_status;

      enable_tc_interrupts();

      if(!not_empty) return status;
      
      // checking start bit
      status=(
        (data_received & 1)?
          status|rx_data_status_codes::BAD_START_BIT:
          status&(~rx_data_status_codes::BAD_START_BIT)
      );

      // cheking parity
      if(parity!=parity_codes::NO_PARITY)
      {
        register uint32_t data_parity=get_even_parity(
          (data_received>>1),
          static_cast<uint32_t>(data_bits)
        );

        if(parity==parity_codes::ODD_PARITY) 
          data_parity=data_parity^1;

        register uint32_t received_parity=
          (data_received>>parity_bit_pos) & 1;

        // verifying parity bit
        status=(
          (received_parity^data_parity)?
            status|rx_data_status_codes::BAD_PARITY:
            status&(~rx_data_status_codes::BAD_PARITY)
        );
      }

      // checking stop bit
      // NOTE: we only verify the first stop bit
      status=(
        ((data_received>>first_stop_bit_pos) & 1)?
          status&(~rx_data_status_codes::BAD_STOP_BIT):
          status|rx_data_status_codes::BAD_STOP_BIT
      );

      // NOTE: in case of error, we put the received raw data
      // on the high 16 bits of status
      if(bad_status(status)) status=status|(data_received<<16);

      data=(data_received>>1)&data_mask;

      return status;
    }

    template<
      timer_ids TIMER,
      size_t RX_BUFFER_LENGTH,
      size_t TX_BUFFER_LENGTH
    > bool uart<
      TIMER,
      RX_BUFFER_LENGTH,
      TX_BUFFER_LENGTH
    >::_uart_ctx_::set_tx_data(
      uint32_t data
    )
    {
      // setting the data to send
      uint32_t data_to_send=data&data_mask;
     
      // setting the parity bit
      if(parity!=parity_codes::NO_PARITY)
      {
        register uint32_t data_parity=get_even_parity(
          data_to_send,
          static_cast<uint32_t>(data_bits)
        );

        if(parity==parity_codes::ODD_PARITY) 
          data_parity=data_parity^1;

        data_to_send=
          (data_to_send<<1)|(data_parity<<parity_bit_pos);
      }
      else data_to_send=(data_to_send<<1);

      // setting the stop bits
      if(stop_bits==stop_bit_codes::ONE_STOP_BIT)
        data_to_send=data_to_send|(1<<first_stop_bit_pos);
      else
        data_to_send=data_to_send|(0x3<<first_stop_bit_pos);

      register bool not_full;
  
      disable_tc_interrupts();
      not_full=tx_buffer.push(data_to_send);
      enable_tc_interrupts();

      if(!not_full) return false;
      
      enable_tc_rc_interrupt();

      disable_tc_interrupts();

      disable_rx_interrupts();
      if(
        (rx_status==rx_status_codes::LISTENING)
        && (tx_status==tx_status_codes::IDLE) 
      ) start_tc_interrupts();
      enable_rx_interrupts();

      //disable_tc_interrupts();
      
      if(tx_status==tx_status_codes::IDLE)
      {
        tx_buffer.pop(data_to_send); 
        tx_data=data_to_send; tx_bit_counter=0; 
      }

      tx_status=tx_status_codes::SENDING;
      
      enable_tc_interrupts();

      return true;
    }
  
  }

}

#endif // SOFT_UART_H

