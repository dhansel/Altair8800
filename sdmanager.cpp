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

#include "config.h"
#include <Arduino.h>
#include "sdmanager.h"
#include "XModem.h"
#include "host.h"
#include "numsys.h"
#include "serial.h"


#ifdef HOST_HAS_FILESYS

HOST_FILESYS_FILE_TYPE datafile;


int recvChar(int msDelay) 
{ 
  unsigned long start = millis();
  while( (int) (millis()-start) < msDelay) 
    { 
      if( Serial.available() ) return (uint8_t) Serial.read(); 
    }

  return -1; 
}


void sendData(const char *data, int size)
{
  Serial.write((const uint8_t *) data, size);
}


static void consume_input()
{
  while( true ) 
    { if( Serial.read()<0 ) { delay(15); if( Serial.read()<0 ) { delay(150); if( Serial.read()<0 ) break; } } }
}


bool dataHandlerSend(unsigned long no, char* data, int size)
{
  if( datafile )
    {
      unsigned long offset = (no-1)*128;
      if( host_filesys_file_seek(datafile, offset) && host_filesys_file_pos(datafile)==offset )
        {
          int n = host_filesys_file_read(datafile, size, (void *) data);

          // if there was nothing more to read then signal end-of-file
          if( n==0 ) return false;

          // XMODEM sends blocks of 128 bytes so if we have less than that
          // fill the rest of the buffer with EOF (ASCII 26) characters
          while( n<size ) data[n++]=26;

          return true;
        }
    }

  // either file was not open or we got past the end
  return false;
}


bool dataHandlerReceive(unsigned long no, char* data, int size)
{
  if( datafile )
    {
      unsigned long offset = (no-1)*128;
      if( host_filesys_file_seek(datafile, offset) && host_filesys_file_pos(datafile)==offset )
        return host_filesys_file_write(datafile, size, (void *) data)==size;
    }

  return false;
}


static char *getFilename(const char *prompt)
{
  static char buf[16];
  int l = 0, dot_pos = -1;

  Serial.print(prompt);
  while(1)
    {
      int c = serial_read(); delay(10);
      if( (c>='0' && c<='9') || (c>='a' && c<='z') || (c>='A' && c<='Z') || c=='_' || c=='~' )
        {
          if( c>='a' && c<='z' ) c -= 32;
          if( (dot_pos<0 && l<8) || (dot_pos>=0 && l-dot_pos<4) )
            { buf[l++] = c; Serial.write(c); }
        }
      else if( c=='.' && dot_pos<0 )
        {
          dot_pos = l;
          buf[l++]=c;
          Serial.write(c);
        }
      else if( c==8 || c==127 )
        {
          if( l>0 )
            {
              l--;
              if( buf[l]=='.' ) dot_pos = -1;
              Serial.print(F("\010 \010"));
            }
        }
      else if( c==9 )
        {
          // TAB=auto-complete
          const char *name;
          bool first = true;
          HOST_FILESYS_DIR_TYPE dir = host_filesys_dir_open();

          // go through directory and try to find a match
          while( (name=host_filesys_dir_nextfile(dir)) )
            {
              if( strnicmp(buf, name, l)==0 )
                {
                  if( first )
                    {
                      // first match => take full file name
                      strcpy(buf, name);
                      first = false;
                    }
                  else
                    {
                      // subsequent match => reduce to matching characters
                      for(int i=l; buf[i]; i++)
                        if( name[i] != buf[i] )
                          { buf[i] = 0; break; }
                    }
                }
            }
          host_filesys_dir_close(dir);

          if( first )
            {
              // no match found => send BELL
              Serial.write(7);
            }
          else if( buf[l]==0 )
            {
              // nothing added => print matching files
              HOST_FILESYS_DIR_TYPE dir = host_filesys_dir_open();
              
              int c=4;
              while( (name=host_filesys_dir_nextfile(dir))!=NULL )
                {
                  if( strnicmp(buf, name, l)==0 )
                    {
                      if( c++ == 4 ) { c=0; Serial.println(); }
                      Serial.print(name);
                      for(int i=strlen(name); i<14; i++) Serial.print(' ');
                    }
                }
              host_filesys_dir_close(dir);
              
              Serial.println('\n');
              Serial.print(prompt);
              Serial.print(buf);
            }
          else
            {
              // print added characters, set new length and dot position
              while( buf[l] ) { Serial.write(buf[l]); if( buf[l]=='.' ) dot_pos = l; l++; }
            }
        }
      else if( c==13 )
        {
          Serial.println();
          buf[l]=0;
          return buf;
        }
      else if( c==27 )
        {
          Serial.println();
          return NULL;
        }
    }
}


static bool checkOverwrite(char *fname)
{
  if( host_filesys_file_exists(fname) )
    {
      char c=0;
      Serial.print(F("File ")); Serial.print(fname); Serial.print(F(" exists. Overwrite (y/n)? "));
      while(c!='y' && c!='n' && c!=27) { c = serial_read(); delay(10); }
      Serial.println(c);
      return c=='y';
    }

  return true;
}


static void receiveFile()
{
  Serial.println();
  char *fname = getFilename("File name to receive: ");
  if( fname!=NULL && checkOverwrite(fname) )
    {
      host_filesys_file_remove(fname);    
      datafile = host_filesys_file_open(fname, true);
      if( datafile )
        {
          XModem modem(recvChar, sendData, dataHandlerReceive);
          
          bool isempty = false;
          host_serial_interrupts_pause();
          delay(100);
          Serial.println(F("\nInitiate XMODEM send now."));
          if( modem.receive() )
            Serial.println(F("\r\nSuccess!"));
          else
            {
              Serial.println(F("\r\nERROR!"));
              isempty = (host_filesys_file_size(fname)==0);
            }
          host_serial_interrupts_resume();          
          consume_input();
          host_filesys_file_close(datafile);
          
          // if nothing was received then delete the empty file
          if( isempty ) host_filesys_file_remove(fname);
        }
      else
        { Serial.print(F("Can not write file: ")); Serial.println(fname); }
    }
}


static void sendFile()
{
  Serial.println();
  char *fname = getFilename("File name to send: ");
  if( fname!=NULL )
    {
      datafile = host_filesys_file_open(fname, false);

      if( datafile )
        {
          XModem modem(recvChar, sendData, dataHandlerSend);
          
          host_serial_interrupts_pause();
          delay(100);
          Serial.println(F("\nInitiate XMODEM receive now."));
          if( modem.transmit() )
            Serial.println(F("Success!"));
          else
            Serial.println(F("ERROR!"));
          consume_input();
          host_serial_interrupts_resume();          

          host_filesys_file_close(datafile);
        }
      else
        { Serial.print(F("Can not read file: ")); Serial.println(fname); }
    }
}


static void deleteFile()
{
  Serial.println();
  char *fname = getFilename("File name to delete: ");
  if( fname!=NULL )
    {
      if( host_filesys_file_exists(fname) )
        {
          char c=0;
          Serial.print(F("Really delete file ")); Serial.print(fname); Serial.print(F(" (y/n)? "));
          while(c!='y' && c!='n') { c = serial_read(); delay(10); }
          Serial.println(c);
          if( c=='y' )
            {
              if( !host_filesys_file_remove(fname) )
                { Serial.print(F("Can not delete file:")); Serial.println(fname); }
              else
                Serial.println(F("Ok."));
            }
        }
      else
        { Serial.print(F("File does not exist: ")); Serial.println(fname); }
    }
}


static void copyFile()
{
  Serial.println();
  char *fname = getFilename("File name to copy: ");
  if( fname!=NULL )
    {
      HOST_FILESYS_FILE_TYPE from = host_filesys_file_open(fname, false);
      if( from )
        {
          fname = getFilename("Copy to: ");
          if( fname!=NULL && checkOverwrite(fname) )
            {
              host_filesys_file_remove(fname);    
              HOST_FILESYS_FILE_TYPE to = host_filesys_file_open(fname, true);
              if( to )
                {
                  int n;
                  byte buffer[1024];
                  
                  Serial.print(F("Copying...")); 
                  unsigned long t = millis();
                  while( from && to && (n=host_filesys_file_read(from, 1024, buffer))>0 ) 
                    {
                      if( (millis()-t)>500 ) { Serial.print('.'); t = millis(); }
                      host_filesys_file_write(to, n, buffer);
                    }
                  
                  if( !from || !to )
                    Serial.println(F("ERROR!"));
                  else
                    Serial.println(F("Ok."));
                  host_filesys_file_close(from);
                  host_filesys_file_close(to);
                }
              else
                { Serial.print(F("Can not write file: ")); Serial.println(fname); }
            }
        }
      else
        { Serial.print(F("Can not read file: ")); Serial.println(fname); }
    }
}


static void renameFile()
{
  Serial.println();
  char *fname = getFilename("File to rename: ");
  if( fname!=NULL )
    {
      char *fromname = strdup(fname);
      if( host_filesys_file_exists(fromname) )
        {
          fname = getFilename("Rename to: ");
          if( fname!=NULL && checkOverwrite(fname) )
            {
              host_filesys_file_remove(fname);
              if( host_filesys_file_rename(fromname, fname) )
                Serial.println(F("Ok."));
              else
                Serial.println(F("ERROR!"));
            }
        }
      else
        { Serial.print(F("File does not exist: ")); Serial.println(fromname); }

      free(fromname);
    }
}


bool waitKey(bool &lineByLine, char pc = '\n')
{
  bool wait = false, exit = false;
  if( serial_available() )
    {
      char c = serial_read();
      if( c == ' ' ) 
        lineByLine = true;
      else if( c==27 || c==3 )
        exit = true;
      else
        wait = true;
    }
  
  if( wait || (pc=='\n' && lineByLine) )
    {
      while( !serial_available() ) delay(10);
      char c = serial_read();
      if( c==27 || c==3 )
        exit = true;
      else 
        lineByLine = (c==' '); 
    }

  return exit;
}



static void typeFile()
{
  Serial.println();
  bool lineByLine = false;
  char *fname = getFilename("File name to type out: ");
  if( fname!=NULL )
    {
      HOST_FILESYS_FILE_TYPE f = host_filesys_file_open(fname, false);
      if( f )
        {
          char c;
          while( host_filesys_file_read(f, 1, &c) ) 
            {
              Serial.write(c);
              c = serial_read();
              if( waitKey(lineByLine, c) ) break;
            }
          Serial.println("\033[0m");
        }
      else
        { Serial.print(F("Can not read file: ")); Serial.println(fname); }
    }
}



static void dumpFile()
{
  Serial.println();
  char *fname = getFilename("File name to dump: ");

  if( fname!=NULL )
    {
      HOST_FILESYS_FILE_TYPE f = host_filesys_file_open(fname, false);
      if( f )
        {
          bool lineByLine = false, ESC = false;
          byte buf[16], n, i;
          
          Serial.print(F("Start address: "));
          uint32_t addr = numsys_read_dword(&ESC);
          Serial.println('\n');
          host_filesys_file_seek(f, addr);
          while( !ESC && (n=host_filesys_file_read(f, 16, buf))>0 )
            {
              numsys_print_word(addr>>16); 
              numsys_print_word(addr&0xFFFF); 
              Serial.print(':'); Serial.print(' ');
              for(i=0; i<n; i++)
                {
                  numsys_print_byte(buf[i]);
                  Serial.print(' ');
                  if( i==7 ) Serial.print(' ');
                }
              
              for(i=n; i<16; i++)
                {
                  Serial.print(' '); Serial.print(' '); Serial.print(' '); 
                  if( i==8 ) Serial.print(' ');
                }
              
              Serial.print(' '); Serial.print(' '); 
              for(i=0; i<n; i++)
                {
                  Serial.print(buf[i]>=32 && buf[i]<127 ? char(buf[i]) : '.');
                  if( i==7 ) Serial.print(' ');
                }
              
              addr += n;
              Serial.println();

              if( waitKey(lineByLine) ) break;
            }
          
          host_filesys_file_close(f);
        }
      else
        { Serial.print(F("Can not read file: ")); Serial.println(fname); }
    }
}


static int compare(const char *n1, const char *n2)
{
  const char *d1 = strchr(n1, '.');
  const char *d2 = strchr(n2, '.');

  if( d1==NULL && d2!=NULL )
    return -11;
  else if( d1!=NULL && d2==NULL )
    return 1;
  else if( d1==NULL && d2==NULL )
    return stricmp(n1, n2);
  else
    {
      int r = stricmp(d1, d2);
      if( r==0 ) r = stricmp(n1, n2);
      return r;
    }
}

static int partition(char **arr, int low, int high)
{
  // pivot = Element to be placed at right position
  char *tmp, *pivot = arr[high];  
  int i = (low - 1);
  for(int j = low; j <= high- 1; j++)
    if( compare(arr[j], pivot)<0 )
      {
        i++;
        tmp = arr[i]; arr[i] = arr[j]; arr[j] = tmp;
      }

  tmp = arr[i+1]; arr[i+1] = arr[high]; arr[high] = tmp;
  return i+1;
}


static void quickSort(char **arr, int low, int high)
{
  if (low < high)
    {
      // pi is partitioning index, arr[pi] is now at right place
      int pi = partition(arr, low, high);
      quickSort(arr, low, pi - 1);
      quickSort(arr, pi + 1, high);
    }
}


static void printDirectory()
{
  HOST_FILESYS_DIR_TYPE dir = host_filesys_dir_open();
  bool lineByLine = false;

  int n = 0, j;
  const char *name;
  char **names = NULL;

  while( (name=host_filesys_dir_nextfile(dir))!=NULL )
    {
      names = (char**) realloc(names, (n+1) * sizeof(char *));
      names[n] = strdup(name);
      n++;
    }
  host_filesys_dir_close(dir);

  quickSort(names, 0, n-1);
  for(j=0; j<n; j++)
    {
      char buf[20];
      int i = 0;
      for(char *c = names[j]; *c; c++)
        {
          if( *c=='.' ) while( i<8 ) { Serial.print(' '); i++; }
          Serial.print(*c);
          i++;
        }
      
      int size = host_filesys_file_size(names[j]);

      while( i++<16 ) Serial.print(' ');
      if( size < 10000 )
        sprintf(buf, "%5i", size);
      else
        sprintf(buf, "%4iK", int(size/1024.0+0.5));
      Serial.println(buf);
      if( waitKey(lineByLine) ) break;
    }
  
  for(int i=0; i<n; i++) free(names[i]);
  free(names);
}



void sd_manager() 
{
  Serial.println(F("\033[2J\033[0;0H"));
  Serial.println(F("SD card file manager"));

  bool redraw = true;
  while(1)
    {
      if( redraw )
        {
          Serial.println(F("\n(d)irectory"));
          Serial.println(F("(t)ype out file"));
          Serial.println(F("(D)ump file"));
          Serial.println(F("(r)eceive file via XMODEM"));
          Serial.println(F("(s)end file via XMODEM"));
          Serial.println(F("(c)opy file"));
          Serial.println(F("(R)ename file"));
          Serial.println(F("(e)rase file"));
          Serial.println(F("(h)elp"));
          Serial.println(F("E(x)it"));
          redraw = false;
        }
      
      Serial.print(F("\nCommand: "));
      byte c = 0;
      while( !((c>=32 && c<=126) || c==27) )
        {
          while( !serial_available() ) delay(50);
          c = serial_read();
        }
      if( c>31 && c<127 ) Serial.println((char) c);
      
      switch( c )
        {
        case 'd':
          printDirectory();
          break;

        case 'r':
          receiveFile();
          break;

        case 's':
          sendFile();
          break;

        case 'e':
          deleteFile();
          break;

        case 'c':
          copyFile();
          break;

        case 'R':
          renameFile();
          break;

        case 't':
          typeFile();
          break;

        case 'D':
          dumpFile();
          break;

        case 'x':
        case 27:
          return;

        default:
          {
            redraw = true;
            break;
          }
        }
    }
}

#else

void sd_manager() 
{}

#endif
