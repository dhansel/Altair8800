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
#if (NUM_DRIVES>0 || NUM_HDSK_UNITS>0) && defined(__SAM3X8E__)

#include <Arduino.h>
#include "sdmanager.h"
#include "XModem.h"
#include "host.h"
#include <SD.h>


File datafile;

int recvChar(int msDelay) 
{ 
  unsigned long start = millis();
  while( millis()-start < msDelay) 
    { 
      if( Serial.available() ) return (uint8_t) Serial.read(); 
      delay(1); 
    }

  return -1; 
}


void sendChar(char sym) 
{ 
  Serial.write(sym); 
}


bool dataHandlerSend(unsigned long no, char* data, int size)
{
  if( datafile )
    {
      unsigned long offset = (no-1)*128;
      if( datafile.seek(offset) && datafile.position()==offset )
        {
          int n = datafile.read((uint8_t *) data, size);

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
      if( datafile.seek(offset) && datafile.position()==offset )
        return datafile.write((uint8_t *) data, size) == size;
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
      int c = Serial.read();
      if( (c>='0' && c<='9') || (c>='a' && c<='z') || (c>='A' && c<='Z') || c=='_' )
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
          bool first = true;
          File entry, dir = SD.open("/");
          dir.rewindDirectory();

          // go through directory and try to find a match
          while( entry=dir.openNextFile() )
            {
              if( !entry.isDirectory() && strncmp(buf, entry.name(), l)==0 )
                {
                  if( first )
                    {
                      // first match => take full file name
                      strcpy(buf, entry.name());
                      first = false;
                    }
                  else
                    {
                      // subsequent match => reduce to matching characters
                      for(int i=l; buf[i]; i++)
                        if( entry.name()[i] != buf[i] )
                          { buf[i] = 0; break; }
                    }
                }

              entry.close();
            }
          dir.close();
          host_clr_status_led_HLDA();

          if( first )
            {
              // no match found => send BELL
              Serial.write(7);
            }
          else if( buf[l]==0 )
            {
              // nothing added => print matching files
              File entry, dir = SD.open("/");
              dir.rewindDirectory();
              
              int c=4;
              while( entry=dir.openNextFile() )
                {
                  if( !entry.isDirectory() && strncmp(buf, entry.name(), l)==0 )
                    {
                      if( c++ == 4 ) { c=0; Serial.println(); }
                      Serial.print(entry.name());
                      for(int i=strlen(entry.name()); i<14; i++) Serial.print(' ');
                    }
                  entry.close();
                }
              dir.close();
              host_clr_status_led_HLDA();
              
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


static void consume_input()
{
  while( true ) 
    { if( Serial.read()<0 ) { delay(15); if( Serial.read()<0 ) { delay(150); if( Serial.read()<0 ) break; } } }
}


static bool checkOverwrite(char *fname)
{
  if( SD.exists(fname) )
    {
      char c=0;
      Serial.print(F("File ")); Serial.print(fname); Serial.print(F(" exists. Overwrite (y/n)? "));
      while(c!='y' && c!='n' && c!=27) c = Serial.read();
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
      SD.remove(fname);    
      datafile = SD.open(fname, FILE_WRITE);
      if( datafile )
        {
          XModem modem(recvChar, sendChar, dataHandlerReceive);
          
          bool isempty = false;
          Serial.println(F("\nInitiate XMODEM send now."));
          if( modem.receive() )
            Serial.println(F("\r\nSuccess!"));
          else
            {
              Serial.println(F("\r\nERROR!"));
              isempty = (datafile.size()==0);
            }
          
          datafile.close();
          
          // if nothing was received then delete the empty file
          if( isempty ) SD.remove(fname);
          
          consume_input();
        }
      else
        { Serial.print(F("Can not write file: ")); Serial.println(fname); }
    }

  host_clr_status_led_HLDA();
}


static void sendFile()
{
  Serial.println();
  char *fname = getFilename("File name to send: ");
  if( fname!=NULL )
    {
      datafile = SD.open(fname, FILE_READ);

      if( datafile )
        {
          XModem modem(recvChar, sendChar, dataHandlerSend);
          
          Serial.println(F("\nInitiate XMODEM receive now."));
          if( modem.transmit() )
            Serial.println(F("Success!"));
          else
            Serial.println(F("ERROR!"));
          
          datafile.close();
          consume_input();
        }
      else
        { Serial.print(F("Can not read file: ")); Serial.println(fname); }
    }

  host_clr_status_led_HLDA();
}


static void deleteFile()
{
  Serial.println();
  char *fname = getFilename("File name to delete: ");
  if( fname!=NULL )
    {
      if( SD.exists(fname) )
        {
          char c=0;
          Serial.print(F("Really delete file ")); Serial.print(fname); Serial.print(F(" (y/n)? "));
          while(c!='y' && c!='n') c = Serial.read();
          Serial.println(c);
          if( c=='y' )
            {
              if( !SD.remove(fname) )
                { Serial.print(F("Can not delete file:")); Serial.println(fname); }
              else
                Serial.println(F("Ok."));
            }
        }
      else
        { Serial.print(F("File does not exist: ")); Serial.println(fname); }
    }
  
  host_clr_status_led_HLDA();
}


static void copyFile()
{
  Serial.println();
  char *fname = getFilename("File name to copy: ");
  if( fname!=NULL )
    {
      File from = SD.open(fname, FILE_READ);
      if( from )
        {
          fname = getFilename("Copy to: ");
          if( fname!=NULL && checkOverwrite(fname) )
            {
              SD.remove(fname);    
              File to = SD.open(fname, FILE_WRITE);
              if( to )
                {
                  int n;
                  byte buffer[1024];
                  
                  Serial.print(F("Copying...")); 
                  unsigned long t = millis();
                  while( from && to && (n=from.read(buffer, 1024))>0 ) 
                    {
                      if( (millis()-t)>500 ) { Serial.print('.'); t = millis(); }
                      to.write(buffer, n);
                    }
                  
                  if( !from || !to )
                    Serial.println(F("ERROR!"));
                  else
                    Serial.println(F("Ok."));
                  from.close();
                  to.close();
                }
              else
                { Serial.print(F("Can not write file: ")); Serial.println(fname); }
            }
        }
      else
        { Serial.print(F("Can not read file: ")); Serial.println(fname); }
    }
  
  host_clr_status_led_HLDA();
}


static void typeFile()
{
  Serial.println();
  char *fname = getFilename("File name to type out: ");
  if( fname!=NULL )
    {
      File f = SD.open(fname, FILE_READ);
      if( f )
        {
          char c;
          while( f.read(&c, 1) ) 
            {
              Serial.write(c);
              c = Serial.read();
              if( c==27 || c==3 ) break;
            }
          Serial.println("\033[0m");
        }
      else
        { Serial.print(F("Can not read file: ")); Serial.println(fname); }
    }
  
  host_clr_status_led_HLDA();
}


static void printDirectory()
{
  File entry, dir = SD.open("/");

  Serial.println();
  dir.rewindDirectory();
  while( entry=dir.openNextFile() )
    {
      if( !entry.isDirectory() )
        {
          Serial.print(entry.name());
          int n=16-strlen(entry.name());
          while(n-->0) Serial.print(' ');
          Serial.println(entry.size(), DEC);
        }
      entry.close();
    }

  dir.rewindDirectory();
  dir.close();
  host_clr_status_led_HLDA();
}


void sd_manager() 
{
  host_serial_interrupts_pause();
  Serial.println(F("\033[2J\033[0;0H"));
  Serial.println(F("SD card file manager"));

  bool redraw = true;
  while(1)
    {
      if( redraw )
        {
          Serial.println(F("\n(d)irectory"));
          Serial.println(F("(t)ype out file"));
          Serial.println(F("(r)eceive file via XMODEM"));
          Serial.println(F("(s)end file via XMODEM"));
          Serial.println(F("(c)opy file"));
          Serial.println(F("(e)rase file"));
          Serial.println(F("(h)elp"));
          Serial.println(F("E(x)it"));
          redraw = false;
        }
      
      Serial.print(F("\nCommand: "));
      byte c = 0;
      while( !((c>=32 && c<=126) || c==27) )
        {
          while( !Serial.available() ) delay(50);
          c = Serial.read();
        }
      Serial.println((char) c);
      
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

        case 't':
          typeFile();
          break;

        case 'x':
        case 27:
          {
            host_serial_interrupts_resume();
            return;
          }

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
