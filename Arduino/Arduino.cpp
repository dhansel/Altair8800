#include "Arduino.h"

#include <stdio.h>
#include <iostream>
#include <sys/timeb.h>
#include <conio.h>
#include <Windows.h>
using namespace std;

SerialClass Serial;


unsigned long millis()
{
  struct timeb ftime;
  ::ftime(&ftime);
  return ftime.time * 1000 + ftime.millitm;
}


unsigned long micros()
{
  return millis()*1000;
}


void delay(unsigned long i)
{
  unsigned long m = millis()+i;
  while(millis() < m);
}


void SerialClass::print(char c) { write(c); }
void SerialClass::print(const char *s) { cout << s; }
void SerialClass::print(int i) { cout << i; }
void SerialClass::print(unsigned int i) { cout << i; }
void SerialClass::print(long int i) { cout << i; }
void SerialClass::print(unsigned long i) { cout << i; }
void SerialClass::print(float f) { cout << f; }
void SerialClass::print(double d)  { cout << d; }

void SerialClass::println() { cout << endl; }
void SerialClass::println(char c) { cout << c << endl; }
void SerialClass::println(const char *s) { cout << s << endl; }
void SerialClass::println(int i) { cout << i << endl; }
void SerialClass::println(unsigned int i) { cout << i << endl; }
void SerialClass::println(long int i) { cout << i << endl; }
void SerialClass::println(unsigned long i) { cout << i << endl; }
void SerialClass::println(float f) { cout << f << endl; }
void SerialClass::println(double d)  { cout << d << endl; }

static bool kbhit_prev_result = false;
static unsigned long kbhit_next_check = 0;

void SerialClass::write(char c) { if( c==127 ) cout << "\b \b"; else cout << c; }
char SerialClass::read() { kbhit_prev_result = false; kbhit_next_check = 0; if( kbhit() ) return getch(); else return 0; }
char SerialClass::peek() { if( kbhit() ) { char c =  getch(); ungetch(c); return c; } else return 0; }
bool SerialClass::availableForWrite() { return true; }

// the kbhit() functions is very inefficient and slows
// down emulation if called too often. 100 times a second is enough.
bool SerialClass::available() 
{ 
  if( !kbhit_prev_result && millis()>kbhit_next_check )
    {
      kbhit_prev_result = kbhit();
      kbhit_next_check  = millis()+10;
    }

  return kbhit_prev_result;
}


void setup();
void loop();
int main(int argc, char **argv)
{
  // send CTRL-C to input instead of processing it (otherwise the
  // emulator would immediately quit if CTRL-C is pressed) and we
  // could not use CTRL-C to stop a running BASIC example.
  // CTRL-C is handled in host_pc.cpp such that pressing it twice
  // within 250ms will cause the emulator to terminate.
  DWORD mode;
  HANDLE hstdin = GetStdHandle(STD_INPUT_HANDLE);
  GetConsoleMode(hstdin, &mode);
  SetConsoleMode(hstdin, mode & ~ENABLE_PROCESSED_INPUT);

  setup();
  while(1) loop();
}
