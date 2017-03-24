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

#if defined(_WIN32) || defined(__linux__)

#include <Arduino.h>
#include <time.h>
#include <string>
#include "Altair8800.h"
#include "mem.h"
#include "serial.h"
#include "cpucore.h"
#include "host_pc.h"
#include "profile.h"
#include "timer.h"

#ifdef _WIN32

#define _WINSOCKAPI_
#include <Windows.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#define SignalEvent SetEvent

#else

#include <signal.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/eventfd.h>
#include <unistd.h>
typedef int SOCKET;
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
static unsigned long long int signal_write_buf = 1;
#define SignalEvent(x) write(x, &signal_write_buf, 8)==0

#endif

byte data_leds;
uint16_t status_leds;
uint16_t addr_leds;
byte stop_request;
static FILE *storagefile = NULL;

//#define DEBUG


bool host_read_function_switch(byte i)
{
  return false;
}


bool host_read_function_switch_debounced(byte i)
{
  return false;
}


bool host_read_function_switch_edge(int i)
{
  return false;
}


uint16_t host_read_function_switches_edge()
{
  return 0;
}


void host_reset_function_switch_state()
{
}

// ----------------------------------------------------------------------------------


void host_write_data(const void *data, uint32_t addr, uint32_t len)
{
#ifdef DEBUG
  printf("Writing %i bytes to   0x%04x: ", len, addr);
  for(int i=0; i<len; i++) printf("%02x ", ((byte *) data)[i]);
  printf("\n");
#endif

  if( storagefile )
    {
      fseek(storagefile, addr, SEEK_SET);
      fwrite(data, len, 1, storagefile);
      fflush(storagefile);
    }
}


void host_read_data(void *data, uint32_t addr, uint32_t len)
{
  if( storagefile )
    {
      fseek(storagefile, addr, SEEK_SET);
      uint32_t l = fread(data, 1, len, storagefile);
      if( l<len ) memset(((byte *) data)+l, 0, len-l);
    }

#ifdef DEBUG
  printf("Reading %i bytes from 0x%04x: ", len, addr);
  for(int i=0; i<len; i++) printf("%02x ", ((byte *) data)[i]);
  printf("\n");
#endif
}


void host_move_data(uint32_t to, uint32_t from, uint32_t len)
{
  void *buf = malloc(len);
  host_read_data(buf, from, len);
  host_write_data(buf, to, len);
  free(buf);
}


void host_copy_flash_to_ram(void *dst, const void *src, uint32_t len)
{
  memcpy(dst, src, len);
}


// ----------------------------------------------------------------------------------


static const char *get_full_path(char *fnamebuf, int fnamebuf_size, const char *filename)
{
  snprintf(fnamebuf, 30, "disks/%s", filename);
  return fnamebuf;
}


bool host_file_exists(const char *filename)
{
  char fnamebuf[30];
  FILE *f = fopen(get_full_path(fnamebuf, 30, filename), "rb");
  if( f )
    {
      fclose(f);
      return true;
    }

  return false;
}


int32_t host_get_file_size(const char *filename)
{
  int32_t res = -1;

  char fnamebuf[30];
  FILE *f = fopen(get_full_path(fnamebuf, 30, filename), "rb");
  if( f )
    {
      if( fseek(f, 0, SEEK_END)==0 )
        res = ftell(f);
      fclose(f);
    }
  
  return res;
}


FILE *open_file(const char *filename)
{
  static char fnamebuf[30];
  static FILE *f = NULL;
  if( f==NULL || strcmp(filename, fnamebuf+6)!=0 )
    {
      if( f!=NULL ) fclose(f);
      f = fopen(get_full_path(fnamebuf, 30, filename), "r+b");
      if( !f ) f = fopen(fnamebuf, "w+b");
    }

  return f;
}


uint32_t host_read_file(const char *filename, uint32_t offset, uint32_t len, void *buffer)
{
  uint32_t res = 0;
  FILE *f = open_file(filename);
  if( f )
    {
      if( fseek(f, offset, SEEK_SET)==0 )
        res = fread(buffer, 1, len, f);
    }

  //printf("host_read_file('%s', %04x, %04x, %p)=%04x\n", filename, offset, len, buffer, res);
  return res;
}


uint32_t host_write_file(const char *filename, uint32_t offset, uint32_t len, void *buffer)
{
  uint32_t res = 0;
  FILE *f = open_file(filename);
  if( f )
    {
      if( fseek(f, offset, SEEK_SET)==0 )
        {
          res = fwrite(buffer, 1, len, f);
          fflush(f);
        }
    }

  //printf("host_write_file('%s', %04x, %04x, %p)=%04x\n", filename, offset, len, buffer, res);
  return res;
}


// ----------------------------------------------------------------------------------

static int ctrlC = 0;

void sig_handler(int signum)
{
  ctrlC++;
}


uint32_t host_get_random()
{
  return rand()*65536l | rand();
}


static int      inp_serial0 = -1, inp_serial1 = -1;
static uint32_t cycles_per_char[2];
static SOCKET   iface_socket = INVALID_SOCKET;

static SOCKET set_up_listener(const char* pcAddress, int nPort)
{
  u_long nInterfaceAddr = inet_addr(pcAddress);
  if (nInterfaceAddr != INADDR_NONE) 
    {
      SOCKET sd = socket(AF_INET, SOCK_STREAM, 0);
#ifndef _WIN32
      int i = 1;
      setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, &i, sizeof(i));
#endif
      if (sd != INVALID_SOCKET) 
        {
          sockaddr_in sinInterface;
          sinInterface.sin_family = AF_INET;
          sinInterface.sin_addr.s_addr = nInterfaceAddr;
          sinInterface.sin_port = nPort;
          if (bind(sd, (sockaddr*)&sinInterface, sizeof(sockaddr_in)) != SOCKET_ERROR) 
            {
              listen(sd, 1);
              return sd;
            }
        }
    }

  return INVALID_SOCKET;
}


#ifdef _WIN32

static HANDLE signalEvent;

DWORD WINAPI host_input_thread(void *data)
{
  WSAEVENT eventHandles[3], socket_event;
  SOCKET accept_socket = INVALID_SOCKET;

  // initialize socket for secondary interface
  WSADATA wsaData;
  WSAStartup(MAKEWORD(1,1), &wsaData);
  accept_socket = set_up_listener("127.0.0.1", htons(8800));
  if( accept_socket == INVALID_SOCKET )
    printf("Can not listen on port 8800 => secondary interface not available\n");
  else
    {
      socket_event = WSACreateEvent();
      WSAEventSelect(accept_socket, socket_event, FD_ACCEPT);
    }
  
  // initialize stdin handle
  HANDLE stdIn = GetStdHandle(STD_INPUT_HANDLE);

  while( 1 )
    {
      int n = 0;
      int console_id = -1, socket_id = -1;

      if( inp_serial0<0 )
        { 
          // ready to receive more data on console (primary input)
          console_id = WSA_WAIT_EVENT_0+n; 
          eventHandles[n++] = stdIn; 
        }

      if( iface_socket != INVALID_SOCKET )
        {
          if( inp_serial1<0 )
            {
              // ready to receive more data on socket (secondary input)
              socket_id = WSA_WAIT_EVENT_0+n; 
              eventHandles[n++] = socket_event; 
            }
        }
      else if( accept_socket != INVALID_SOCKET )
        { 
          // ready to accept connection on socket
          socket_id = WSA_WAIT_EVENT_0+n; 
          eventHandles[n++] = socket_event; 
        }

      // adding this allows host_check_interrupts to signal this thread that 
      // an input has been read and we can accept more inputs now (otherwise
      // we may get stuck in WSAWaitForMultipleEvents even though more input
      // is available)
      eventHandles[n++] = signalEvent;

      // wait until we either
      // - get input on console (if we are ready to accept more)
      // - get input on socket (if we are ready to accept more)
      // - a new client is connected (if none is connected right now)
      // - host_check_interrupts has signaled that there was a change in
      //   whether we are ready to accept more data
      DWORD result = WSAWaitForMultipleEvents(n, eventHandles, false, WSA_INFINITE, true);
      if( console_id>=0 && result == WSA_WAIT_EVENT_0 + console_id )
        {
          if( Serial.available() )
            {
              // we received some console input (reading it resets the event)
              inp_serial0 = Serial.read();
            }
          else
            {
              // some sort of other events => clear it from the queue
              INPUT_RECORD r[512];
              DWORD read;
              ReadConsoleInput(stdIn, r, 512, &read );
            }          
        }
      else if( socket_id>=0 && result == WSA_WAIT_EVENT_0+socket_id )
        {
          // activity on socket
          if( iface_socket==INVALID_SOCKET )
            {
              // if we don't have an iface_socket then we're currently
              // waiting for connections
              sockaddr_in sinRemote;
              socklen_t nAddrSize = sizeof(sinRemote);
              iface_socket = accept(accept_socket, (sockaddr*)&sinRemote, &nAddrSize);
              if( iface_socket != INVALID_SOCKET )
                {
                  // success => prepare to receive data
                  WSAResetEvent(socket_event);
                  WSAEventSelect(iface_socket, socket_event, FD_READ | FD_CLOSE);
                 }
            }
          else
            {
              // we have an iface_socket => either input or connection drop
              char c;
              if( recv(iface_socket, &c, 1, 0)==0 )
                {
                  // no input => connection was dropped
                  iface_socket = INVALID_SOCKET;
                  
                  // prepare to accept more connections
                  WSAResetEvent(socket_event);
                  WSAEventSelect(accept_socket, socket_event, FD_ACCEPT);
                }
              else
                {
                  // received input on socket
                  DWORD n;
                  inp_serial1 = (byte) c;

                  // if no more data to read then reset the event
                  if( ioctlsocket(iface_socket, FIONREAD, &n)==0 && n==0 ) WSAResetEvent(socket_event);
                }
            }
        }
    }
}

#else

static int signalEvent;

void *host_input_thread(void *data)
{
  SOCKET accept_socket = INVALID_SOCKET;
  fd_set s_rd, s_wr, s_ex;

  // initialize socket for secondary interface
  accept_socket = set_up_listener("127.0.0.1", htons(8800));
  if( accept_socket == INVALID_SOCKET )
    printf("Can not listen on port 8800 => secondary interface not available\r\n");

  FD_ZERO(&s_wr);
  FD_ZERO(&s_ex);
  while( 1 )
    {
      FD_ZERO(&s_rd);

      int nfds = 0;
      if( inp_serial0<0 )
	{
          // ready to receive more data on console (primary input)
	  FD_SET(fileno(stdin), &s_rd);
	  if( fileno(stdin)>=nfds ) nfds = fileno(stdin)+1;
	}

      if( iface_socket != INVALID_SOCKET )
        {
          if( inp_serial1<0 )
	    {
              // ready to receive more data on socket (secondary input)
	      FD_SET(iface_socket, &s_rd); 
	      if( iface_socket>=nfds ) nfds = iface_socket+1;
	    }
        }
      else if( accept_socket != INVALID_SOCKET )
	{
          // ready to accept connection on socket
	  FD_SET(accept_socket, &s_rd); 
	  if( accept_socket>=nfds ) nfds = accept_socket+1;
	}

      // adding this allows host_check_interrupts to signal this thread that 
      // an input has been read and we can accept more inputs now (otherwise
      // we may get stuck in WSAWaitForMultipleEvents even though more input
      // is available)
      FD_SET(signalEvent, &s_rd);
      if( signalEvent>=nfds ) nfds = signalEvent+1;

      // wait until we either
      // - get input on console (if we are ready to accept more)
      // - get input on socket (if we are ready to accept more)
      // - a new client is connected (if none is connected right now)
      // - host_check_interrupts has signaled that there was a change in
      //   whether we are ready to accept more data
      if( select(nfds, &s_rd, NULL, NULL, NULL) >= 0 )
        {
	  if( FD_ISSET(signalEvent, &s_rd) )
	    {
	      // clear the signal
	      byte buf[8]; 
	      read(signalEvent, buf, 8)==0;
	    }

          if( FD_ISSET(fileno(stdin), &s_rd) )
	    inp_serial0 = Serial.read();
          
          if( iface_socket != INVALID_SOCKET && FD_ISSET(iface_socket, &s_rd) )
            {
              char c;
              if( recv(iface_socket, &c, 1, MSG_NOSIGNAL)==0 )
                {
                  // no input => connection was dropped
                  iface_socket = INVALID_SOCKET;
                }
              else
                {
                  // received input on socket
                  inp_serial1 = (byte) c;
                }
            }
          else if( accept_socket != INVALID_SOCKET && FD_ISSET(accept_socket, &s_rd) )
            {
              // accept a new connection
              sockaddr_in sinRemote;
              socklen_t nAddrSize = sizeof(sinRemote);
              iface_socket = accept(accept_socket, (sockaddr*)&sinRemote, &nAddrSize);
	      if( iface_socket!=INVALID_SOCKET )
		{
		  // make a connected telnet client enter CHAR mode
		  write(iface_socket,"\377\375\042\377\373\001",6)!=0;
		}
            }
        }
    }

  return NULL;
}

#endif


void host_check_interrupts()
{
  static unsigned long prevCtrlC = 0;
  static uint32_t prev_char_cycles0 = 0, prev_char_cycles1 = 0;

  // check input from interface 0 (console)
  if( inp_serial0>=0 || ctrlC>0 )
    if( host_read_status_led_WAIT() || (timer_get_cycles()-prev_char_cycles0) >= cycles_per_char[0] )
      {
	int c = -1;
	
	if( ctrlC>0 )
	  { c = 3; ctrlC--; }
	else if( inp_serial0>=0 )
	  { 
	    c = inp_serial0; 
	    
	    // we have consumed the input => signal input thread to receive more
	    inp_serial0 = -1; 
	    SignalEvent(signalEvent); 
	  }
	
	if( c==3 )
	  {
	    // CTRL-C was pressed.  If we receive two CTRL-C in short order
	    // then we terminate the emulator.
	    if( millis()>prevCtrlC+250 )
	      prevCtrlC = millis();
	    else
	      exit(0);
	  }
	
	if( c>=0 )
	  serial_receive_host_data(0, (byte) c);
	
	prev_char_cycles0 = timer_get_cycles();
      }

  // check input from interface 1 (socket)
  if( inp_serial1>=0 )
    if( (timer_get_cycles()-prev_char_cycles1) >= cycles_per_char[1] )
      {
	serial_receive_host_data(1, (byte) inp_serial1);
	
	// we have consumed the input => signal input thread to receive more
	inp_serial1 = -1;
	SignalEvent(signalEvent); 
	
	prev_char_cycles1 = timer_get_cycles();
      }
}


void host_serial_setup(byte iface, unsigned long baud, bool set_primary_interface)
{
  // assuming 10 bits (start bit + 8 data bits + stop bit) per character
  if( iface<2 ) cycles_per_char[iface]  = (10*2000000)/baud;
}


void host_serial_write(byte iface, byte data)
{
  if( iface==0 )
    Serial.write(data);
  else
    send(iface_socket, (char *) &data, 1, 0 /*MSG_NOSIGNAL*/);
}


bool host_serial_available_for_write(byte iface)
{
  return iface==0 ? Serial.availableForWrite() : (iface_socket != INVALID_SOCKET);
}


void host_setup()
{
  data_leds = 0;
  status_leds = 0;
  addr_leds = 0;
  stop_request = 0;
  
  // open storage data file for mini file system
  storagefile = fopen("AltairStorage.dat", "r+b");
  if( storagefile==NULL ) 
    {
      void *chunk = calloc(1024, 1);
      storagefile = fopen("AltairStorage.dat", "wb");
      if( storagefile!=NULL )
        {
          uint32_t size;
          for( size = 0; (size+1024) < HOST_STORAGESIZE; size+=1024 )
            fwrite(chunk, 1024, 1, storagefile);
          fwrite(chunk, HOST_STORAGESIZE-size, 1, storagefile);
          fclose(storagefile);
        }
      
      storagefile = fopen("AltairStorage.dat", "r+b");
    }

#if defined(_WIN32)
  // send CTRL-C to input instead of processing it (otherwise the
  // emulator would immediately quit if CTRL-C is pressed) and we
  // could not use CTRL-C to stop a running BASIC example.
  // CTRL-C is handled in host_check_interrupts (above) such that 
  // pressing it twice within 250ms will cause the emulator to terminate.
  DWORD mode;
  HANDLE hstdin = GetStdHandle(STD_INPUT_HANDLE);
  GetConsoleMode(hstdin, &mode);
  SetConsoleMode(hstdin, mode & ~ENABLE_PROCESSED_INPUT);

  // create an event that can be sent to awaken the input thread
  signalEvent = CreateEvent(NULL, false, false, NULL);

  // create the input thread
  DWORD id; 
  HANDLE h = CreateThread(0, 0, host_input_thread, NULL, 0, &id);
  CloseHandle(h);
#elif defined(__linux__)
  // handle CTRL-C in sig_handler so only pressing it twice
  // will terminate the simulator (otherwise CTRL-C could not
  // be sent to the emulated program
  signal(SIGINT, sig_handler);

  // create an event that can be sent to awaken the input thread
  signalEvent = eventfd(0, 0);

  // create the input thread
  pthread_t id;
  pthread_create(&id, NULL, host_input_thread, 0);
  pthread_detach(id);
#endif
  
  // initialize random number generator
  srand((unsigned int) time(NULL));
}

#endif
