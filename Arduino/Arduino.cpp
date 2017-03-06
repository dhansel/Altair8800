#include "Arduino.h"

#include <stdio.h>
#include <string>
#include <iostream>
#include <sys/timeb.h>
using namespace std;

#ifdef _WIN32
#include <conio.h>
#include <Windows.h>
#define FixNewline(s) s
#else

#include <ncurses.h>
#include <termios.h>
#include <unistd.h>
#define endl "\r\n" << std::flush

#define _getch   getch
#define _ungetch ungetch

bool _kbhit()
{
  int ch = _getch();
  if (ch != ERR) {
    ungetch(ch);
    return 1;
  } else {
    return 0;
  }
}

string FixNewline(string subject)
{
  size_t pos = 0;
  while ((pos = subject.find("\n", pos)) != string::npos) {
    subject.replace(pos, 1, "\r\n");
    pos += 2;
  }
  return subject;
}

void ncurses_exit()
{
  endwin();
}

#endif

SerialClass Serial;


unsigned long millis()
{
  struct timeb ftime;
  ::ftime(&ftime);
  return (unsigned long) ftime.time * 1000 + ftime.millitm;
}


unsigned long micros()
{
  return millis()*1000;
}


void delay(unsigned long i)
{
#ifdef _WIN32
  Sleep(i);
#else
  usleep(i*1000);
#endif
}


void SerialClass::print(char c) { write(c); }
void SerialClass::print(const char *s) { cout << FixNewline(s) << std::flush; }
void SerialClass::print(int i) { cout << i << std::flush; }
void SerialClass::print(unsigned int i) { cout << i << std::flush; }
void SerialClass::print(long int i) { cout << i << std::flush; }
void SerialClass::print(unsigned long i) { cout << i << std::flush; }
void SerialClass::print(float f) { cout << f << std::flush; }
void SerialClass::print(double d)  { cout << d << std::flush; }

void SerialClass::println() { cout << endl; }
void SerialClass::println(char c) { write(c); cout << endl; }
void SerialClass::println(const char *s) { cout << FixNewline(s) << endl; }
void SerialClass::println(int i) { cout << i << endl; }
void SerialClass::println(unsigned int i) { cout << i << endl; }
void SerialClass::println(long int i) { cout << i << endl; }
void SerialClass::println(unsigned long i) { cout << i << endl; }
void SerialClass::println(float f) { cout << f << endl; }
void SerialClass::println(double d)  { cout << d << endl; }

static bool kbhit_prev_result = false;
static unsigned long kbhit_next_check = 0;

void SerialClass::flush() { cout << std::flush; }
char SerialClass::peek() { if( _kbhit() ) { char c =  _getch(); _ungetch(c); return c; } else return 0; }
bool SerialClass::availableForWrite() { return true; }
bool SerialClass::available() { return _kbhit(); }

#ifdef _WIN32
void SerialClass::write(char c) { cout << (c==127 ? string("\b \b") : string(1,c)) << std::flush; }
#else
void SerialClass::write(char c) { cout << (c=='\n' ? "\r\n" : (c==127 ? "\b \b" : string(1,c))) << std::flush; }
#endif


char SerialClass::read()
{
  kbhit_prev_result = false;
  kbhit_next_check  = 0;
  if( _kbhit() )
    {
#ifdef _WIN32
      return _getch();
#else
      // on linux, getch() returns 127 for backspace (we need 8) and 10 for linefeed/CR (we need 13)
      char c = getch();
      return c==10 ? 13 : (c==127 ? 8 : c);
#endif
    }
  else
    return 0;
}


void setup();
void loop();
int main(int argc, char **argv)
{
#ifndef _WIN32
  // initialize ncurses library
  initscr();
  scrollok(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  cbreak();
  noecho();
  getch();
  atexit(ncurses_exit);
#endif

  setup();
  while(1) loop();
}
