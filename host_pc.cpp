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

// un-define Serial which was #define'd to SwitchSerialClass in switch_serial.h
// otherwise we get infinite loops when calling Serial.* functions below
#undef Serial


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
      if( !f ) f = fopen(fnamebuf, "rb");
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


uint32_t host_set_file(const char *filename, uint32_t offset, uint32_t len, byte b, bool keep_open)
{
  uint32_t res = 0;
  static FILE *f = NULL;

  // if a filename is given then (re-)open the file
  if( filename!=NULL )
    {
      char fnamebuf[30];
      if( f!=NULL ) fclose(f);
      f = fopen(get_full_path(fnamebuf, 30, filename), "r+b");
      if( !f ) f = fopen(fnamebuf, "w+b");
    }

  // if a position is given then seek to that position
  if( f && offset < 0xffffffff )
    if( fseek(f, offset, SEEK_SET)!=0 )
      { fclose(f); f = NULL; }

  // if the file is open and length is >0 then write to the file
  if( f && len>0 )
    {
      // write data in 256-byte chunks
      byte buf[256];
      memset(buf, b, 256);
      for(uint32_t i=0; i<len; i+=256) 
        res += fwrite(buf, i+256<len ? 256 : len-i, 1, f);
    }

  // if we're not supposed to keep the file open then close it
  if( !keep_open )
    { fclose(f); f=NULL; }

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


static int      inp_serial[HOSTPC_NUM_SOCKET_CONN+1];
static uint32_t cycles_per_char[HOSTPC_NUM_SOCKET_CONN+1];
static SOCKET   iface_socket[HOSTPC_NUM_SOCKET_CONN];

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
  WSAEVENT eventHandles[6], socket_accept_event, socket_read_event[HOSTPC_NUM_SOCKET_CONN];
  SOCKET accept_socket = INVALID_SOCKET;

  // initialize socket for secondary interface
  WSADATA wsaData;
  WSAStartup(MAKEWORD(1,1), &wsaData);
#if HOSTPC_NUM_SOCKET_CONN>0
  accept_socket = set_up_listener("127.0.0.1", htons(8800));
  if( accept_socket == INVALID_SOCKET )
    printf("Can not listen on port 8800 => secondary interface not available\n");
  else
    {
      socket_accept_event = WSACreateEvent();
      WSAEventSelect(accept_socket, socket_accept_event, FD_ACCEPT);
      for(int i=0; i<HOSTPC_NUM_SOCKET_CONN; i++) socket_read_event[i] = WSACreateEvent();
    }
#endif
  
  // initialize stdin handle
  HANDLE stdIn = GetStdHandle(STD_INPUT_HANDLE);

  while( 1 )
    {
      int i, n = 0;

      if( inp_serial[0]<0 )
        { 
          // ready to receive more data on console (primary input)
          eventHandles[n++] = stdIn; 
        }

     if( accept_socket != INVALID_SOCKET )
       eventHandles[n++] = socket_accept_event; 

      for(i=0; i<HOSTPC_NUM_SOCKET_CONN; i++)
        if( iface_socket[i] != INVALID_SOCKET && inp_serial[i+1]<0 )
          {
            // ready to receive more data on this socket
            eventHandles[n++] = socket_read_event[i]; 
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
      if( result >= WSA_WAIT_EVENT_0 && result < WSA_WAIT_EVENT_0+n )
        {
          result -= WSA_WAIT_EVENT_0;
          if( eventHandles[result]==stdIn )
            {
              if( Serial.available() )
                {
                  // we received some console input (reading it resets the event)
                  inp_serial[0] = Serial.read();
                }
              else
                {
                  // some sort of other events => clear it from the queue
                  INPUT_RECORD r[512];
                  DWORD read;
                  ReadConsoleInput(stdIn, r, 512, &read );
                }
            }
          else if( eventHandles[result] == socket_accept_event )
            {
              sockaddr_in sinRemote;
              socklen_t nAddrSize = sizeof(sinRemote);
              for(i=0; i<HOSTPC_NUM_SOCKET_CONN; i++)
                if( iface_socket[i]==INVALID_SOCKET )
                  break;

              if( i<HOSTPC_NUM_SOCKET_CONN )
                {
                  iface_socket[i] = accept(accept_socket, (sockaddr*)&sinRemote, &nAddrSize);
                  if( iface_socket[i]!=INVALID_SOCKET )
                    {
                      const char *s = "[Connected as: ";
                      send(iface_socket[i],s,strlen(s), 0);
                      s = host_serial_port_name(i+1);
                      send(iface_socket[i],s,strlen(s), 0);
                      s = "]\r\n";
                      send(iface_socket[i],s,strlen(s), 0);
                      
                      //printf("Connected client to serial #%i\n", i+1);
                      WSAResetEvent(socket_read_event[i]);
                      WSAEventSelect(iface_socket[i], socket_read_event[i], FD_READ | FD_CLOSE);
                    }
                }
              else
                {
                  SOCKET s = accept(accept_socket, (sockaddr*)&sinRemote, &nAddrSize);
                  const char *msg = "[Too many client connections]";
                  send(s,msg,strlen(msg), 0);
                  shutdown(s, 2);
                }
              
              WSAResetEvent(socket_accept_event);
            }
          else
            {
              for(i=0; i<HOSTPC_NUM_SOCKET_CONN; i++)
                if( eventHandles[result]==socket_read_event[i] )
                  {
                    // either input or connection drop
                    char c;
                    if( recv(iface_socket[i], &c, 1, 0)==0 )
                      {
                        // no input => connection was dropped
                        iface_socket[i] = INVALID_SOCKET;
                        inp_serial[i+1] = -1;
                        //printf("Disconnected serial #%i\n", i+2);
                      }
                    else
                      {
                        // received input on socket
                        DWORD n;
                        inp_serial[i+1] = (byte) c;
                        //printf("Received %i on serial #%i\n", c, i+2);
                        
                        // if no more data to read then reset the event
                        if( ioctlsocket(iface_socket[i], FIONREAD, &n)==0 && n==0 ) WSAResetEvent(socket_read_event[i]);
                      }
                  }
            }
        }
    }

  return 0;
}

#else

static int signalEvent;

void *host_input_thread(void *data)
{
  SOCKET accept_socket = INVALID_SOCKET;
  fd_set s_rd, s_wr, s_ex;
  int i;

  // initialize socket for secondary interface
#if HOSTPC_NUM_SOCKET_CONN>0
  accept_socket = set_up_listener("0.0.0.0", htons(8800));
  if( accept_socket == INVALID_SOCKET )
    printf("Can not listen on port 8800 => secondary interface not available\r\n");
#endif

  FD_ZERO(&s_wr);
  FD_ZERO(&s_ex);
  while( 1 )
    {
      FD_ZERO(&s_rd);

      int nfds = 0;
      if( inp_serial[0]<0 )
	{
          // ready to receive more data on console (primary interface)
	  FD_SET(fileno(stdin), &s_rd);
	  if( fileno(stdin)>=nfds ) nfds = fileno(stdin)+1;
	}

      for(i=0; i<HOSTPC_NUM_SOCKET_CONN; i++)
	if( iface_socket[i] != INVALID_SOCKET && inp_serial[i+1]<0 )
	  {
	    // ready to receive more data on socket
	    FD_SET(iface_socket[i], &s_rd); 
	    if( iface_socket[i]>=nfds ) nfds = iface_socket[i]+1;
	  }

      if( accept_socket != INVALID_SOCKET )
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
	    inp_serial[0] = Serial.read();

	  for(i=0; i<HOSTPC_NUM_SOCKET_CONN; i++)
	    if( iface_socket[i] != INVALID_SOCKET && FD_ISSET(iface_socket[i], &s_rd) )
            {
              char c;
              if( recv(iface_socket[i], &c, 1, MSG_NOSIGNAL)==0 )
                {
                  // no input => connection was dropped
                  iface_socket[i] = INVALID_SOCKET;
                  inp_serial[i+1] = -1;
                }
              else
                {
                  // received input on socket
		  //printf("Received %02X on serial #%i\r\n", (byte) c, i+1);
                  inp_serial[i+1] = (byte) c;
                }
            }

	  if( accept_socket != INVALID_SOCKET && FD_ISSET(accept_socket, &s_rd) )
            {
              sockaddr_in sinRemote;
              socklen_t nAddrSize = sizeof(sinRemote);

	      for(i=0; i<HOSTPC_NUM_SOCKET_CONN; i++)
		if( iface_socket[i]==INVALID_SOCKET )
		  break;

	      if( i<HOSTPC_NUM_SOCKET_CONN )
		{
		  // accept a new connection
		  iface_socket[i] = accept(accept_socket, (sockaddr*)&sinRemote, &nAddrSize);
		  if( iface_socket[i]!=INVALID_SOCKET )
		    {
		      // make a connected telnet client enter CHAR mode
		      //write(iface_socket[i],"\377\375\042\377\373\001",6)==0;
		      const char *s = "[Connected as: ";
		      send(iface_socket[i],s,strlen(s), 0);
		      s = host_serial_port_name(i+1);
		      send(iface_socket[i],s,strlen(s), 0);
		      s = "]\r\n";
		      send(iface_socket[i],s,strlen(s), 0);
		    }
		}
              else
                {
                  SOCKET s = accept(accept_socket, (sockaddr*)&sinRemote, &nAddrSize);
                  const char *msg = "[Too many client connections]";
                  send(s,msg,strlen(msg), 0);
                  shutdown(s, 2);
                }
            }
        }
    }

  return NULL;
}

#endif


static void host_check_ctrlc(char c)
{
  static unsigned long prevCtrlC = 0;

  if( c==3 )
    {
      // CTRL-C was pressed.  If we receive two CTRL-C in short order
      // then we terminate the emulator.
      if( millis()>prevCtrlC+250 )
        prevCtrlC = millis();
      else
        exit(0);
    }
  else
    prevCtrlC = 0;
}


void host_check_interrupts()
{
  static unsigned long prevCtrlC = 0;
  static uint32_t prev_char_cycles[HOSTPC_NUM_SOCKET_CONN+1] = {0};

  // check input from interface 0 (console)
  if( inp_serial[0]>=0 || ctrlC>0 )
    if( host_read_status_led_WAIT() || (timer_get_cycles()-prev_char_cycles[0]) >= cycles_per_char[0] )
      {
	int c = -1;
	
	if( ctrlC>0 )
	  { c = 3; ctrlC--; }
	else if( inp_serial[0]>=0 )
	  { 
	    c = inp_serial[0]; 
	    
	    // we have consumed the input => signal input thread to receive more
	    inp_serial[0] = -1; 
	    SignalEvent(signalEvent); 
	  }

        // double ctrl-c on console quits emulator
        host_check_ctrlc(c);
	
	if( c>=0 )
          serial_receive_host_data(0, (byte) c);
	
	prev_char_cycles[0] = timer_get_cycles();
      }

  // check input from interface 1-HOSTPC_NUM_SOCKET_CONN+1 (sockets)
  for(int i=1; i<HOSTPC_NUM_SOCKET_CONN+1; i++)
    if( inp_serial[i]>=0 )
      if( host_read_status_led_WAIT() || (timer_get_cycles()-prev_char_cycles[i]) >= cycles_per_char[i] )
        {
          // double ctrl-c on primary interface quits emulator
          if( i==SwitchSerial.getSelected() ) host_check_ctrlc(inp_serial[i]);

          serial_receive_host_data(i, (byte) inp_serial[i]);
          
          // we have consumed the input => signal input thread to receive more
          inp_serial[i] = -1;
          SignalEvent(signalEvent); 
          
          prev_char_cycles[i] = timer_get_cycles();
        }
}


// ----------------------------------------------------------------------------------------------------


void host_serial_setup(byte iface, uint32_t baud, uint32_t config, bool set_primary_interface)
{
  // assuming 10 bits (start bit + 8 data bits + stop bit) per character
  if( iface<HOSTPC_NUM_SOCKET_CONN+1 ) cycles_per_char[iface]  = (10*2000000)/baud;

  // switch the primary serial interface (if requested)
  if( set_primary_interface ) SwitchSerial.select(iface); 
}


void host_serial_end(byte i)
{}


bool host_serial_ok(byte i)
{
  return i==0 || (i<HOSTPC_NUM_SOCKET_CONN+1 && iface_socket[i-1]!=INVALID_SOCKET);
}


int host_serial_available(byte i)
{
  return i<HOSTPC_NUM_SOCKET_CONN+1 && inp_serial[i]>=0 ? 1 : 0;
}


int host_serial_peek(byte i)
{
  return i<HOSTPC_NUM_SOCKET_CONN+1 ? inp_serial[i] : -1;
}


int host_serial_read(byte i)
{
  if( i<HOSTPC_NUM_SOCKET_CONN+1 )
    {
      int res = inp_serial[i];
      inp_serial[i] = -1;
      return res;
    }
  else
    return -1;
}


void host_serial_flush(byte i)
{}


int host_serial_available_for_write(byte i)
{
  if( i==0 )
    return Serial.availableForWrite();
  else if( i<HOSTPC_NUM_SOCKET_CONN+1 )
    return iface_socket[i-1] != INVALID_SOCKET;
  else
    return false;
}


size_t host_serial_write(byte i, uint8_t data)
{
  if( i==0 )
    { Serial.write(data); return 1; }
  else if( i<HOSTPC_NUM_SOCKET_CONN+1 && iface_socket[i-1] != INVALID_SOCKET )
    { send(iface_socket[i-1], (char *) &data, 1, 0 /*MSG_NOSIGNAL*/); return 1; }

  return 0;
}


const char *host_serial_port_name(byte i)
{
  switch(i)
    {
    case 0: return "Console";
    case 1: return "1st client on socket 8800";
    case 2: return "2nd client on socket 8800";
    case 3: return "3rd client on socket 8800";
    case 4: return "4th client on socket 8800";
    }

 return "???";
}


bool host_serial_port_baud_limits(byte i, uint32_t *min, uint32_t *max)
{
  if( i<HOSTPC_NUM_SOCKET_CONN+1 )
    {
      *min = 110;
      *max = 115200;
      return true;
    }
  else
    return false;
}


bool host_serial_port_has_configs(byte i)
{
  return false;
}


// ----------------------------------------------------------------------------------------------------

bool host_have_sd_card()
{
  return false;
}


void host_system_info()
{
#if defined(_WIN32)
  SwitchSerial.println("Host is Windows PC");
#else
  SwitchSerial.println("Host is Linux/Unix PC");
#endif
}


// these are defined and initialized in the main() function in Arduino/Arduino.cpp
extern int    g_argc;
extern char **g_argv;

bool host_is_reset()
{
  bool res = false;
  for(int i=0; i<g_argc; i++)
    if( strcmp(g_argv[i], "-r")==0 )
      return true;

  return false;
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

  inp_serial[0] = -1;
  for(int i=0; i<HOSTPC_NUM_SOCKET_CONN; i++)
    {
      iface_socket[i] = INVALID_SOCKET;
      inp_serial[i+1] = -1;
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
