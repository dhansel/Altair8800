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

#include "host.h"
#include "image.h"

static const char *fname_template[2] = {"DISK%02X.DSK", "HDSK%02X.DSK"};
static const char *dirfilename[2]    = {"DISKDIR.TXT",  "HDSKDIR.TXT"};


const char *image_get_dir_content(byte image_type)
{
  static byte  contenttype  = -1;
  static char *contentcache = NULL;

  if( image_type != contenttype )
    {
      contenttype = image_type;
      if( contentcache!=NULL ) { free(contentcache); contentcache = NULL; }
    }
  
  if( contentcache==NULL )
    {
      int dirlen = host_get_file_size(dirfilename[image_type]);
      if( dirlen>0 )
        {
          contentcache = (char *) malloc(dirlen+1);
          if( contentcache!=NULL )
            {
              if( host_read_file(dirfilename[image_type], 0, dirlen, contentcache) )
                contentcache[dirlen] = 0;
              else
                {
                  free(contentcache);
                  contentcache = (char *) "";
                }
            }
          else
            {
              free(contentcache);
              contentcache = (char *) "";
            }
        }
      else
        contentcache = (char *) "";
    }

  return contentcache;
}


bool image_get_filename(byte image_type, byte image_num, char *filename, int buf_len, bool check_exist)
{
  snprintf(filename, buf_len, fname_template[image_type], image_num);
  return check_exist ? host_file_exists(filename) : true;
}


const char *image_get_filename(byte image_type, byte image_num, bool check_exist)
{
  static char buf[13];
  if( image_get_filename(image_type, image_num, buf, 13, check_exist) )
    return buf;
  else
    return NULL;
}


const char *image_get_description(byte image_type, byte image_num)
{
  static char *buf = NULL;
  const char *fname = image_get_filename(image_type, image_num);

  if( fname!=NULL )
    {
      const char *c = strstr(image_get_dir_content(image_type), fname);
      if( c )
        {
          if( buf ) free(buf);
          buf = (char *) malloc(strlen(c)+1);
          if( buf )
            {
              char *b = buf;
              while(*c != 13 && *c!=10 && *c!=0) *b++ = *c++;
              *b = 0;
              return buf;
            }
          else
            return fname;
        }
      else
        return fname;
    }
  else
    return fname;
}


