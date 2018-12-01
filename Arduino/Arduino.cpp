#ifdef _WIN32
#include <conio.h>
#include <Windows.h>
#include <Wincon.h>
#define FixNewline(s) s

#include <stdio.h>
#include <string>
#include <iostream>
#include <sys/timeb.h>
using namespace std;

#include "Arduino.h"

int EnableANSI()
{
  HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
  if (hOut == INVALID_HANDLE_VALUE)
    {
      return false;
    }
  HANDLE hIn = GetStdHandle(STD_INPUT_HANDLE);
  if (hIn == INVALID_HANDLE_VALUE)
    {
      return false;
    }

  DWORD dwOriginalOutMode = 0;
  DWORD dwOriginalInMode = 0;
  if (!GetConsoleMode(hOut, &dwOriginalOutMode))
    {
      return false;
    }
  if (!GetConsoleMode(hIn, &dwOriginalInMode))
    {
      return false;
    }

  DWORD dwRequestedOutModes = 0x000C; // ENABLE_VIRTUAL_TERMINAL_PROCESSING | DISABLE_NEWLINE_AUTO_RETURN;
  DWORD dwRequestedInModes  = 0x0200; // ENABLE_VIRTUAL_TERMINAL_INPUT;

  DWORD dwOutMode = dwOriginalOutMode | dwRequestedOutModes;
  if (!SetConsoleMode(hOut, dwOutMode))
    {
      // we failed to set both modes, try to step down mode gracefully.
      dwRequestedOutModes = 0x0004; // ENABLE_VIRTUAL_TERMINAL_PROCESSING;
      dwOutMode = dwOriginalOutMode | dwRequestedOutModes;
      if (!SetConsoleMode(hOut, dwOutMode))
        {
          // Failed to set any VT mode, can't do anything here.
          return -1;
        }
    }

  DWORD dwInMode = dwOriginalInMode | 0x0200; // ENABLE_VIRTUAL_TERMINAL_INPUT;
  if (!SetConsoleMode(hIn, dwInMode))
    {
      // Failed to set VT input mode, can't do anything here.
      return -1;
    }

  return 0;
}

#else

#include <ncurses.h>
#include <termios.h>
#include <unistd.h>

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


static bool kbhit_prev_result = false;
static unsigned long kbhit_next_check = 0;

void SerialClass::flush() { cout << std::flush; }
char SerialClass::peek() { if( _kbhit() ) { char c =  _getch(); _ungetch(c); return c; } else return 0; }
int  SerialClass::availableForWrite() { return 1; }
int  SerialClass::available() { return _kbhit(); }

#ifdef _WIN32
size_t SerialClass::write(uint8_t c) { cout << (c==127 ? string("\b \b") : string(1,(char) c)) << std::flush; return 1; }
#else
size_t SerialClass::write(uint8_t c) { cout << (c=='\n' ? "\r\n" : (c==127 ? "\b \b" : string(1,c))) << std::flush; return 1; }
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


int    g_argc;
char **g_argv;

void setup();
void loop();
int main(int argc, char **argv)
{
  g_argc = argc;
  g_argv = argv;

#ifdef _WIN32
  // enable ANSI model in Windows
  EnableANSI();
#else
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
