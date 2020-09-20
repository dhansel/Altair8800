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

#define MAX_OPEN_FILES 3
static byte num_open_files = 0;

#ifndef HOST_BUFFERSIZE
#define HOST_BUFFERSIZE 0
#endif


#if USE_HOST_FILESYS>0 && defined(HOST_HAS_FILESYS)


// --------------------------- Host provides filesystem ---------------------------


static byte file_open[MAX_OPEN_FILES];
static HOST_FILESYS_FILE_TYPE file_info[MAX_OPEN_FILES];

static byte alloc_file_id(bool write)
{
  for(byte i=0; i<MAX_OPEN_FILES; i++)
    if( file_open[i]==0 )
      {
        file_open[i] = write ? 2 : 1;
        num_open_files++;
        return i+1;
      }
  
  return 0;
}


static void free_file_id(byte fid)
{
  if( num_open_files>0 && fid>0 && fid<=MAX_OPEN_FILES && file_open[fid-1] != 0 )
    {
      file_open[fid-1] = 0;
      num_open_files--;
    }
}


static const char* filesys_get_fname(char nm1, char nm2)
{
  static char buf[10];

  switch( nm1 )
    {
    case 'C': sprintf(buf, "%02X.CFG", (unsigned char) nm2); break;
    case 'B': sprintf(buf, "%c.BAS", nm2); break;
    case 'M': sprintf(buf, "%02X.MEM", (unsigned char) nm2); break;
    case 'D': sprintf(buf, "%02X.DAT", (unsigned char) nm2); break;
    default : sprintf(buf, "%02X.T%02X", (unsigned char) nm2, (unsigned char) nm1); break;
    }
  
  return buf;
}


static byte filesys_open_file(char nm1, char nm2, bool write)
{
  byte fid = alloc_file_id(write);

  if( fid>0 )
    {
      const char *fname = filesys_get_fname(nm1, nm2);

      if( write && host_filesys_file_exists(fname) ) host_filesys_file_remove(fname);
      HOST_FILESYS_FILE_TYPE f = host_filesys_file_open(fname, write);
      if( f )
        file_info[fid-1] = f;
      else
        { free_file_id(fid); fid = 0; }
    }
  
  return fid;
}


byte filesys_open_read(char nm1, char nm2)
{
  return filesys_open_file(nm1, nm2, false);
}

byte filesys_open_write(char nm1, char nm2)
{
  return filesys_open_file(nm1, nm2, true);
}

bool filesys_exists(char nm1, char nm2)
{
  return host_filesys_file_exists(filesys_get_fname(nm1, nm2));
}

bool filesys_write_data(byte fid, const void *data, uint16_t len)
{
  if( fid<=MAX_OPEN_FILES && file_open[fid-1]==2 )
    return host_filesys_file_write(file_info[fid-1], len, data)==len;
  else
    return false;
}

bool filesys_write_char(byte fid, byte c)
{
  return filesys_write_data(fid, &c, 1);
}

bool filesys_eof(byte fid)
{
  if( fid<=MAX_OPEN_FILES && file_open[fid-1]>0 )
    return host_filesys_file_eof(file_info[fid-1]);
  else
    return true;
}

bool filesys_is_read(byte fid)
{
  return fid<=MAX_OPEN_FILES && file_open[fid-1]==1;
}

bool filesys_is_write(byte fid)
{
  return fid<=MAX_OPEN_FILES && file_open[fid-1]==2;
}

bool filesys_seek(byte fid, uint32_t pos)
{
  return host_filesys_file_seek(file_info[fid-1], pos);
}

uint32_t filesys_getpos(byte fid)
{
  return host_filesys_file_pos(file_info[fid-1]);
}

uint16_t filesys_read_data(byte fid, void *data, uint16_t len)
{
  if( fid<=MAX_OPEN_FILES && file_open[fid-1]==1 )
    return host_filesys_file_read(file_info[fid-1], len, data);
  else
    return 0;
}

bool filesys_read_char(byte fid, byte *c)
{
  return filesys_read_data(fid, c, 1)>0;
}

void filesys_close(byte fid)
{
  if( fid<=MAX_OPEN_FILES && file_open[fid-1]>0 )
    {
      host_filesys_file_close(file_info[fid-1]);
      free_file_id(fid);
    }
}


bool filesys_write_file(char nm1, char nm2, const void *data, uint16_t len)
{
  byte fid = filesys_open_write(nm1, nm2);
  if( fid )
    {
      filesys_write_data(fid, data, len);
      filesys_close(fid);
      return true;
    }
  else
    return false;
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


void filesys_convert()
{
  struct DirEntryStruct
  {
    byte     name1, name2;
    uint16_t len;
    uint32_t pos;
  } entry;

  byte numEntries;
  host_storage_read(&numEntries, HOST_STORAGESIZE-4, 1);

  Serial.print(F("Converting file system..."));
  uint16_t totalUsed = 4+numEntries*sizeof(struct DirEntryStruct);
  for(byte i=0; i<numEntries; i++)
    {
      Serial.print('.');
      uint32_t entrypos = HOST_STORAGESIZE-4-((i+1)*sizeof(struct DirEntryStruct));
      host_storage_read(&entry, entrypos, sizeof(struct DirEntryStruct));

      byte fid = filesys_open_write(entry.name1, entry.name2);
      if( fid )
        {
          uint16_t bufsize = max(HOST_BUFFERSIZE, 1);
          byte buf[max(HOST_BUFFERSIZE, 1)];
          uint16_t pos = 0;

          while( pos<entry.len )
            {
              uint16_t len = min(entry.len-pos, bufsize);
              host_storage_read(buf, entry.pos+pos, len);
              filesys_write_data(fid, buf, len);
              pos += len;
            }
          filesys_close(fid);
        }
    }
}


void filesys_setup()
{
  for(byte i=0; i<MAX_OPEN_FILES; i++)
    file_open[i] = 0;

  // try to initialize host-specific data storage memory for reading
  if( host_storage_init(false) )
    {
      // check whether we need to convert an old mini-filesystem 
      // to the host filesystem
      char buf[4];
      host_storage_read(buf, HOST_STORAGESIZE-4, 4);
      if( buf[1]=='A' && buf[2]=='F' && buf[3]=='S' )
        {
          filesys_convert();

          // tell the host to invalidate the storage memory
          // so we don't convert it again next time
          host_storage_invalidate();
        }
      
      // close storage memorg
      host_storage_close();
    }
}

#else

// ----------------------------- Use own mini-filesystem -------------------------------


struct DirEntryStruct
{
  byte     name1, name2;
  uint16_t len;
  uint32_t pos;
};

static uint32_t dir_start = 0;
static struct DirEntryStruct file_data[MAX_OPEN_FILES];

#if HOST_BUFFERSIZE>1
static uint32_t write_buffer_len = 0;
static byte     write_buffer[HOST_BUFFERSIZE];
#endif


static byte dir_get_num_entries()
{
  byte n;
  host_storage_read(&n, HOST_STORAGESIZE-4, 1);
  return n;
}

static void dir_set_num_entries(byte n)
{
  host_storage_write(&n, HOST_STORAGESIZE-4, 1);
  dir_start = HOST_STORAGESIZE-4-(n*sizeof(struct DirEntryStruct));
}

static void dir_write_entry(byte num, struct DirEntryStruct *entry)
{
  uint32_t entrypos = HOST_STORAGESIZE-4-((num+1)*sizeof(struct DirEntryStruct));
  host_storage_write(entry, entrypos, sizeof(struct DirEntryStruct));
}

static void dir_read_entry(byte num, struct DirEntryStruct *entry)
{
  uint32_t entrypos = HOST_STORAGESIZE-4-((num+1)*sizeof(struct DirEntryStruct));
  host_storage_read(entry, entrypos, sizeof(struct DirEntryStruct));
}


static byte dir_find_file(char nm1, char nm2, struct DirEntryStruct *entry)
{
  //printf("filesys_find_file(%c, %c)\n", nm1, nm2);
  byte num_entries = dir_get_num_entries();
  for(byte i=0; i<num_entries; i++)
    {
      dir_read_entry(i, entry);
      if( entry->name1==(byte) nm1 && entry->name2==(byte) nm2 )
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
      host_storage_move(entry.pos, entry.pos+entry.len, filespaceend-entry.len);
      
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
      host_storage_move(HOST_STORAGESIZE-4-((numentries-1)*sizeof(struct DirEntryStruct)),
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
          host_storage_move(entry.pos, entry.pos+entry.len, filespaceend-entry.len);
          
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


bool filesys_exists(char nm1, char nm2)
{
  struct DirEntryStruct entry;
  if( num_open_files < MAX_OPEN_FILES )
    if( dir_find_file(nm1, nm2, &entry)!=0xff && entry.len>0 )
      return true;

  return false;
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
          host_storage_write(write_buffer, entry->pos + entry->len, write_buffer_len);
          entry->len += write_buffer_len;
          write_buffer_len = 0;
        }
      
      if( len > HOST_BUFFERSIZE/2 )
        {
          host_storage_write(data, entry->pos + entry->len, len);
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
      host_storage_write(data, entry->pos + entry->len, len);
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
      host_storage_write(data, entry.pos, entry.len);
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


bool filesys_seek(byte fid, uint32_t pos)
{
  if( pos < file_data[fid].len )
    {
      file_data[fid-1].pos = pos;
      return true;
    }
  else
    return false;
}


uint32_t filesys_getpos(byte fid)
{
  return file_data[fid-1].pos;
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
      host_storage_read(d, entry->pos, len);
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
          host_storage_write(write_buffer, fileinfo->pos + fileinfo->len, write_buffer_len);
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

  Serial.println(F("\033[2J\033[0;0H"));
  Serial.println(F("ID Type            Name        Size"));
  Serial.println(F("-----------------------------------"));

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
  
  Serial.println();
  Serial.print(F("Filesystem size: ")); Serial.print(HOST_STORAGESIZE); Serial.println(F(" bytes"));
  Serial.print(F("         in use: ")); Serial.print(totalUsed); Serial.println(F(" bytes"));
  Serial.print(F("      available: ")); Serial.print(HOST_STORAGESIZE-totalUsed); Serial.println(F(" bytes"));
}


void filesys_manage()
{
  bool redraw = true;
  if( num_open_files>0 )
    {
      Serial.println(F("\n"));
      filesys_print_dir();
      Serial.println(F("\n\nFile system is locked."));
      Serial.println(F("Press any key to return to main menu..."));
      while( !serial_available() ) delay(50);
      serial_read();
      return;
    }

  while( true )
    {
      if( redraw )
        {
          Serial.print(F("\n\n"));
          filesys_print_dir();
          Serial.print(F("\n\nCommand (dFrx): "));
        }
      
      while( !serial_available() ) delay(50);
      
      redraw = true;
      switch( serial_read() )
        {
        case 27:
        case 'x': 
          return;

        case 'd':
          {
            byte i;
            Serial.println();
            Serial.print(F("\nDelete file with id: "));
            if( numsys_read_byte(&i) )
              {
                Serial.println();
                Serial.print(F("Really delete file with id "));
                numsys_print_byte(i);
                Serial.print(F(" (y/n)? "));
                while( !serial_available() );
                if( serial_read()=='y' )
                  filesys_delete(i);
              }
            break;
          }

        case 'r':
          {
            byte i;
            Serial.println();
            Serial.print(F("\nRead file with id: "));
            if( !numsys_read_byte(&i) ) break;
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

                    while( !serial_available() ) delay(50);
                    if( serial_read()== 27 ) break;
                  }
                
                filesys_close(fid);
              }
            break;
          }
              
        case 'F': 
          {
            Serial.println();
            Serial.print(F("\nReally re-format and lose all data (y/n)? "));
            while( !serial_available() );
            if( serial_read()=='y' )
              dir_set_num_entries(0);
            break;
          }

        default:
          redraw = false;
          break;
        }
    }
}

void filesys_setup()
{
  // check file system id, must be either "AFS" (Altair File System) or "AFC"
  // "AFC" means this is a valid file system that has already been converted 
  // to use the host file system.
  char buf[4];
  host_storage_read(buf, HOST_STORAGESIZE-4, 4);
  if( buf[1]!='A' || buf[2]!='F' || buf[3]!='S' )
    {
      buf[0] = 0;
      buf[1] = 'A';
      buf[2] = 'F';
      buf[3] = 'S';

      // write id + zero directoty entries
      host_storage_write(buf, HOST_STORAGESIZE-4, 4);
    }
  
  // set the current lower border of the directory
  dir_start = HOST_STORAGESIZE-4-(dir_get_num_entries()*sizeof(struct DirEntryStruct));

  // zero out file_data structure
  memset(file_data, 0, MAX_OPEN_FILES*sizeof(struct DirEntryStruct));
}


#endif
