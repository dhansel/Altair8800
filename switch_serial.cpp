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

#if !defined(__AVR_ATmega2560__)

#include <Arduino.h>
#include "host.h"
#include "switch_serial.h"


SwitchSerialClass SwitchSerial;


SwitchSerialClass::SwitchSerialClass() : Stream()
{
  m_selected = 0;
}


void SwitchSerialClass::begin(unsigned long baud)
{
  host_serial_setup(m_selected, baud, SERIAL_8N1, false);
}


void SwitchSerialClass::end()
{
  host_serial_end(m_selected);
}


int SwitchSerialClass::available(void)
{
  return host_serial_available(m_selected);
}


int SwitchSerialClass::availableForWrite(void)
{
  return host_serial_available_for_write(m_selected);
}


int SwitchSerialClass::peek(void)
{
  return host_serial_peek(m_selected);
}


int SwitchSerialClass::read(void)
{
  return host_serial_read(m_selected);
}


void SwitchSerialClass::flush(void)
{
  return host_serial_flush(m_selected);
}


size_t SwitchSerialClass::write(uint8_t b)
{
  return host_serial_write(m_selected, b);
}


SwitchSerialClass::operator bool() 
{
  return host_serial_ok(m_selected);
}

#endif
