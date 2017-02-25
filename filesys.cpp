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

#include "filesys.h"
#include "host.h"
#include "numsys.h"
#include "serial.h"


struct DirEntryStruct
{
  byte     name1, name2;
  uint16_t len;
  uint32_t pos;
};


#define MAX_OPEN_FILES 3
static byte num_open_files = 0;
static uint32_t dir_start = 0;
static struct DirEntryStruct file_data[MAX_OPEN_FILES];

#ifndef HOST_BUFFERSIZE
#define HOST_BUFFERSIZE 0
#endif

#if HOST_BUFFERSIZE>1
static uint32_t write_buffer_len = 0;
static byte     write_buffer[HOST_BUFFERSIZE];
#endif


static byte dir_get_num_entries()
{
  byte n;
  host_read_data(&n, HOST_STORAGESIZE-4, 1);
  return n;
}

static void dir_set_num_entries(byte n)
{
  host_write_data(&n, HOST_STORAGESIZE-4, 1);
  dir_start = HOST_STORAGESIZE-4-(n*sizeof(struct DirEntryStruct));
}

static void dir_write_entry(byte num, struct DirEntryStruct *entry)
{
  uint32_t entrypos = HOST_STORAGESIZE-4-((num+1)*sizeof(struct DirEntryStruct));
  host_write_data(entry, entrypos, sizeof(struct DirEntryStruct));
}

static void dir_read_entry(byte num, struct DirEntryStruct *entry)
{
  uint32_t entrypos = HOST_STORAGESIZE-4-((num+1)*sizeof(struct DirEntryStruct));
  host_read_data(entry, entrypos, sizeof(struct DirEntryStruct));
}


static byte dir_find_file(char nm1, char nm2, struct DirEntryStruct *entry)
{
  //printf("filesys_find_file(%c, %c)\n", nm1, nm2);
  byte num_entries = dir_get_num_entries();
  for(byte i=0; i<num_entries; i++)
    {
      dir_read_entry(i, entry);
      if( entry->name1==nm1 && entry->name2==nm2 )
        return i;
    }
  
  return 0xff;
}


static uint32_t dir_get_filespace_end()
{
  byte numentries = dir_get_num_entries();
  struct DirEntryStruct entry;

  uint32_t filespaceend = 0;
  for(byte i=0; i<numentries; i++)
    {
      //printf("%i\n", i);
      dir_read_entry(i, &entry);
      if( entry.pos+entry.len > filespaceend ) 
        filespaceend = entry.pos+entry.len;
    }

  return filespaceend;
}


static byte alloc_file_id()
{
  for(byte i=0; i<MAX_OPEN_FILES; i++)
    if( file_data[i].name1==0 )
      {
        num_open_files++;
        return i;
      }
  
  return 0;
}


static void free_file_id(byte fid)
{
  if( num_open_files>0 && fid>0 && fid<=MAX_OPEN_FILES && file_data[fid-1].name1 != 0 )
    {
      file_data[fid-1].name1 = 0;
      num_open_files--;
    }
}


static void filesys_delete(byte dirindex)
{
  struct DirEntryStruct entry, entry2;
  byte numentries = dir_get_num_entries();

  if( dirindex < numentries )
    {
      uint32_t filespaceend = dir_get_filespace_end();

      dir_read_entry(dirindex, &entry);
      
      // move all other data up
      host_move_data(entry.pos, entry.pos+entry.len, filespaceend-entry.len);
      
      // update directory entries for all files located past the deleted one
      filespaceend = 0;
      for(byte i=0; i<numentries; i++)
        if( i != dirindex )
          {
            dir_read_entry(i, &entry2);
            if( entry2.pos > entry.pos )
              {
                entry2.pos -= entry.len;
                dir_write_entry(i, &entry2);
              }
            
            if( entry2.pos+entry2.len > filespaceend ) 
              filespaceend = entry2.pos+entry2.len;
          }
      
      // move the directory entries
      host_move_data(HOST_STORAGESIZE-4-((numentries-1)*sizeof(struct DirEntryStruct)),
                     HOST_STORAGESIZE-4-(numentries*sizeof(struct DirEntryStruct)),
                     (numentries-dirindex-1)*sizeof(struct DirEntryStruct));
      
      // set new number of directory entries
      dir_set_num_entries(numentries-1);
    }
}


void filesys_delete(char nm1, char nm2)
{
  struct DirEntryStruct entry;
  filesys_delete(dir_find_file(nm1, nm2, &entry));
}


byte filesys_open_write(char nm1, char nm2)
{
  byte fid = 0;

  //printf("filesys_open_write(%c, %c)\n", nm1, nm2);

  if( num_open_files < MAX_OPEN_FILES )
    {
      // current limitation: can not start writing while another file is open
      for(int i=0; i<MAX_OPEN_FILES; i++)
        if( file_data[i].name1!=0 )
          return false;

      byte numentries = dir_get_num_entries();
      uint32_t filespaceend = dir_get_filespace_end();
      
      struct DirEntryStruct entry, entry2;
      byte dirindex = dir_find_file(nm1, nm2, &entry);
      if( dirindex==0xff )
        {
          // check that there's enough space to create the directory entry
          if( filespaceend + (numentries+1)*sizeof(struct DirEntryStruct) + 4 >= HOST_STORAGESIZE )
            return 0;
          
          // new file => add directory entry
          dirindex = numentries;
          dir_set_num_entries(numentries+1);
        }
      else
        {
          // file exists => move all other data up (i.e. delete file)
          host_move_data(entry.pos, entry.pos+entry.len, filespaceend-entry.len);
          
          // update directory entries for all files located past the deleted one
          filespaceend = 0;
          for(byte i=0; i<numentries; i++)
            if( i != dirindex )
              {
                dir_read_entry(i, &entry2);
                if( entry2.pos > entry.pos )
                  {
                    entry2.pos -= entry.len;
                    dir_write_entry(i, &entry2);
                  }
                
                if( entry2.pos+entry2.len > filespaceend ) 
                  filespaceend = entry2.pos+entry2.len;
              }
        }

      fid = alloc_file_id();

      // update the directory information for this file
      file_data[fid].name1 = nm1;
      file_data[fid].name2 = nm2;
      file_data[fid].pos   = filespaceend;
      file_data[fid].len   = 0;
      dir_write_entry(dirindex, file_data+fid);

      // file mode is 'write' and remember directory index
      file_data[fid].name1 = 'W';
      file_data[fid].name2 = dirindex;
      fid++;

#if HOST_BUFFERSIZE>1
      write_buffer_len = 0;
#endif
    }

  return fid;
}


byte filesys_open_read(char nm1, char nm2)
{
  byte fid = 0, dirindex;

  //printf("filesys_open_read(%c, %c)\n", nm1, nm2);

  struct DirEntryStruct entry;
  if( num_open_files < MAX_OPEN_FILES )
    if( (dirindex=dir_find_file(nm1, nm2, &entry))!=0xff && entry.len>0 )
      {
        fid = alloc_file_id();
        file_data[fid].name1 = 'R';
        file_data[fid].name2 = dirindex;
        file_data[fid].pos   = entry.pos;
        file_data[fid].len   = entry.len;
        fid++;
      }

  //printf("filesys_open_read(%c, %c) = %i\n", nm1, nm2, fid);
  return fid;
}


bool filesys_write_char(byte fid, byte c)
{
  return filesys_write_data(fid, &c, 1);
}


bool filesys_write_data(byte fid, const void *data, uint16_t len)
{
  //printf("filesys_write_data(%i,%i)\n", fid, len);
  struct DirEntryStruct *entry = file_data+(fid-1);

  if( entry->name1!='W' )
    return false;

#if HOST_BUFFERSIZE>1
  if( entry->pos + entry->len + write_buffer_len + len < dir_start )
    {
      if( write_buffer_len + len > HOST_BUFFERSIZE || len > HOST_BUFFERSIZE/2 )
        {
          host_write_data(write_buffer, entry->pos + entry->len, write_buffer_len);
          entry->len += write_buffer_len;
          write_buffer_len = 0;
        }
      
      if( len > HOST_BUFFERSIZE/2 )
        {
          host_write_data(data, entry->pos + entry->len, len);
          entry->len += len;
        }
      else
        {
          memcpy(write_buffer+write_buffer_len, data, len);
          write_buffer_len += len;
        }

      return true;
    }

#else
  if( entry->pos + entry->len + len < dir_start )
    {
      host_write_data(data, entry->pos + entry->len, len);
      entry->len += len;
      return true;
    }
#endif

  return false;
}
 

bool filesys_write_file(char nm1, char nm2, const void *data, uint16_t len)
{
  bool res = false;
  struct DirEntryStruct entry;
  //printf("filesys_write_file(%c, %c)\n", nm1, nm2);
  byte dirindex = dir_find_file(nm1, nm2, &entry);
  if( dirindex!=0xff && entry.len==len )
    {
      // file exists and has same length
      host_write_data(data, entry.pos, entry.len);
      res = true;
    }
  else
    {
      byte fid = filesys_open_write(nm1, nm2);
      if( fid )
        {
          res = filesys_write_data(fid, data, len);
          filesys_close(fid);
        }
    }

  return res;
}


uint16_t filesys_read_file(char nm1, char nm2, void *data, uint16_t len)
{
  uint16_t res = 0;
  byte fid = filesys_open_read(nm1, nm2);
  if( fid )
    {
      res = filesys_read_data(fid, data, len);
      filesys_close(fid);
    }
  return res;
}


bool filesys_eof(byte fid)
{
  struct DirEntryStruct *entry = file_data+(fid-1);

  if( entry->name1=='R' )
    return entry->len==0;
  else if( entry->name1=='W' )
    {
#if HOST_BUFFERSIZE>1
      return entry->pos + entry->len + write_buffer_len + 1 >= dir_start;
#else
      return entry->pos + entry->len + 1 >= dir_start;
#endif
    }
  else
	return true;
}


bool filesys_is_write(byte fid)
{
  return file_data[fid-1].name1=='W';
}


bool filesys_is_read(byte fid)
{
  return file_data[fid-1].name1=='R';
}


bool filesys_read_char(byte fid, byte *c)
{
  return filesys_read_data(fid, c, 1)>0;
}


uint16_t filesys_read_data(byte fid, void *d, uint16_t len)
{
  struct DirEntryStruct *entry = file_data+(fid-1);
  if( entry->name1!='R' )
    return false;

  if( entry->len < len ) 
    len = entry->len;

  if( len>0 )
    {
      host_read_data(d, entry->pos, len);
      entry->pos += len;
      entry->len -= len;
    }
  return len;
}


void filesys_close(byte fid)
{
  //printf("filesys_close()\n");
  struct DirEntryStruct *fileinfo = file_data+(fid-1);
  if( fileinfo->name1 == 'W' )
    {
      // file was opened for writing
#if HOST_BUFFERSIZE>1
      if( write_buffer_len>0 )
        {
          host_write_data(write_buffer, fileinfo->pos + fileinfo->len, write_buffer_len);
          fileinfo->len += write_buffer_len;
          write_buffer_len = 0;
        }
#endif
      
      // update length in directory entry
      struct DirEntryStruct entry;
      dir_read_entry(fileinfo->name2, &entry);
      entry.len = fileinfo->len;
      dir_write_entry(fileinfo->name2, &entry);
    }
  
  free_file_id(fid);
}


void filesys_print_dir()
{
  // ID Type            Name        Size
  // -----------------------------------
  // 01 memory page     C000-C0FF    256
  // 02 captured data   file #00     30
  // 03 BASIC program   a            800
  // 04 <80>            <81>         

  Serial.print(F("\033[2J\n"));
  Serial.print(F("ID Type            Name        Size\n"));
  Serial.print(F("-----------------------------------\n"));
  byte numEntries = dir_get_num_entries();
  struct DirEntryStruct entry;
  uint16_t totalUsed = 4+numEntries*sizeof(struct DirEntryStruct);
  for(byte i=0; i<numEntries; i++)
    {
      dir_read_entry(i, &entry);
      numsys_print_byte(i);
      Serial.print(' ');
      switch( entry.name1 )
        {
        case 'M': 
          {
            Serial.print(F("Memory page     #"));
            numsys_print_byte(entry.name2);
            Serial.print(F("        "));
            break;
          }

        case 'D': 
          {
            Serial.print(F("Captured data   #")); 
            numsys_print_byte(entry.name2);
            Serial.print(F("        "));
            break;
          }

        case 'B': 
          {
            Serial.print(F("BASIC program   "));
            Serial.print((char) entry.name2);
            Serial.print(F("          "));
            break;
          }
          
        case 'C': 
          {
            Serial.print(F("Configuration   #"));
            numsys_print_byte(entry.name2);
            Serial.print(F("        "));
            break;
          }
          
        default: 
          {
            Serial.print('<');
            numsys_print_byte(entry.name1);
            Serial.print('>');
            Serial.print(F("            "));
            Serial.print('<');
            numsys_print_byte(entry.name2);
            Serial.print('>');
            Serial.print(F("       "));
            break;
          }
        }

      if( entry.len<10000 ) Serial.print(' ');
      if( entry.len<1000  ) Serial.print(' ');
      if( entry.len<100   ) Serial.print(' ');
      if( entry.len<10    ) Serial.print(' ');
      Serial.print(entry.len);

      for(byte j=0; j<MAX_OPEN_FILES; j++)
        if( file_data[j].name1!=0 && file_data[j].name2==i )
          {
            switch( file_data[j].name1 )
              {
              case 'W': Serial.print(F("  [currently open for writing]")); break;
              case 'R': Serial.print(F("  [currently open for reading]")); break;
              default:  Serial.print(F("  ???")); break;
              }
          }

      Serial.println();
      totalUsed += entry.len;
    }
  
  Serial.print(F("\nFilesystem size: ")); Serial.print(HOST_STORAGESIZE); Serial.println(F(" bytes"));
  Serial.print(F("         in use: ")); Serial.print(totalUsed); Serial.println(F(" bytes"));
  Serial.print(F("      available: ")); Serial.print(HOST_STORAGESIZE-totalUsed); Serial.println(F(" bytes"));
}


void filesys_manage()
{
  if( num_open_files>0 )
    {
      Serial.print(F("\n\n"));
      filesys_print_dir();
      Serial.println(F("\n\n[File system is locked]\n"));
      while( !serial_available() ) delay(50);
      serial_read();
      return;
    }

  while( true )
    {
      Serial.print(F("\n\n"));
      filesys_print_dir();
      Serial.print(F("\n\nCommand (dFrx): "));
      
      while( !serial_available() ) delay(50);
      Serial.println();

      switch( serial_read() )
        {
        case 27:
        case 'x': 
          return;

        case 'd':
          {
            Serial.print(F("\nDelete file with id: "));
            byte i = (byte) numsys_read_word();
            Serial.print(F("\nReally delete file with id "));
            numsys_print_byte(i);
            Serial.print(F(" (y/n)? "));
            while( !serial_available() );
            if( serial_read()=='y' )
              filesys_delete(i);
            break;
          }

        case 'r':
          {
            Serial.print(F("\nRead file with id: "));
            byte i = (byte) numsys_read_word();
            Serial.println();
            
            struct DirEntryStruct entry;
            dir_read_entry(i, &entry);

            byte fid = filesys_open_read(entry.name1, entry.name2);
            if( fid )
              {
                uint16_t addr = 0;
                byte n, i, buf[16];
                while( (n=(byte) filesys_read_data(fid, &buf, 16))>0 )
                  {
                    numsys_print_word(addr); Serial.print(':'); Serial.print(' ');
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
                  }
                
                filesys_close(fid);
                while( !serial_available() );
                serial_read();
              }
            break;
          }
              
        case 'F': 
          {
            Serial.print(F("\nReally re-format and lose all data (y/n)? "));
            while( !serial_available() );
            if( serial_read()=='y' )
              dir_set_num_entries(0);
            break;
          }
        }
    }
}


void filesys_setup()
{
  // check id
  char buf[4];
  host_read_data(buf, HOST_STORAGESIZE-4, 4);
  if( buf[1]!='A' || buf[2]!='F' || buf[3]!='S' )
    {
      buf[0] = 0;
      buf[1] = 'A';
      buf[2] = 'F';
      buf[3] = 'S';

      // write id + zero directoty entries
      host_write_data(buf, HOST_STORAGESIZE-4, 4);
    }
  
  // set the current lower border of the directory
  dir_start = HOST_STORAGESIZE-4-(dir_get_num_entries()*sizeof(struct DirEntryStruct));

  // zero out file_data structure
  memset(file_data, 0, MAX_OPEN_FILES*sizeof(struct DirEntryStruct));
}
