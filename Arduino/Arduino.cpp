#include "Arduino.h"

#include <stdio.h>
#include <iostream>
#include <sys/timeb.h>
#include <conio.h>
using namespace std;

SerialClass Serial;


unsigned long millis()
{
  struct timeb ftime;
  ::ftime(&ftime);
  return ftime.time * 1000 + ftime.millitm;
}


void delay(unsigned long i)
{
  unsigned long m = millis()+i;
  while(millis() < m);
}


void SerialClass::print(char c) { cout << c; }
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

void SerialClass::write(char c) { cout << c; }
char SerialClass::read() { return getch(); }
char SerialClass::peek() { if( kbhit() ) { char c =  getch(); ungetch(c); return c; } else return 0; }


bool SerialClass::availableForWrite()
{
  return true;
}


bool SerialClass::available() 
{ 
  return kbhit();
}


void setup();
void loop();
int main(int argc, char **argv)
{
  setup();
  while(1) loop();
}
