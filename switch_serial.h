// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------


#ifndef SWITCH_SERIAL_H
#define SWITCH_SERIAL_H

#include <Arduino.h>

class SwitchSerialClass : public Stream
{
  public:
    SwitchSerialClass();
    virtual void begin(unsigned long);
    virtual void end();
    virtual int available(void);
    virtual int availableForWrite(void);
    virtual int peek(void);
    virtual int read(void);
    virtual void flush(void);
    virtual size_t write(uint8_t);
    using Print::write; // pull in write(str) and write(buf, size) from Print
    virtual operator bool();

    void    select(uint8_t n) { m_selected = n; }
    uint8_t getSelected() { return m_selected; }

 private:
    uint8_t m_selected;
};

extern SwitchSerialClass SwitchSerial;
#define Serial SwitchSerial


#endif
