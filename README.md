# Altair8800
Source code for Arduino Altair 8800 simulator<br>
For details about the simulator see:<br>
https://www.hackster.io/david-hansel/arduino-altair-8800-simulator-3594a6

For extending the simulator with external harware see:
https://github.com/dhansel/Altair8800-IOBus

Support for Teensy 3.6 host implemented by Dirk Herrendoerfer:
https://github.com/dherrendoerfer/altair-8800-again

Please post any questions or issues in:
https://groups.google.com/forum/#!forum/altair-duino

## Altair 8800B Features

  * Turn on the "B" mode emulation from the Config menu (STOP+AUX1 Up)
  * AUX1 Up will display the Accumulator on the DATA LEDs. 
  * AUX1 Down will set the Accumulator to the value in Switches A0-A7
  * AUX2 Up will Read from an input port and place the value in the accumulator. The port number is the _left_ 8 switches (A8-A16)
  * AUX2 Down will Write the Accumulator to the output port set in the _left_ 8 switches
  * EXAMINE+AUX1 Up/Down will access the original function. 
  * If Serial Input is enabled, the function of u/U and l/c always use the non-B versions.

The LEDs in B mode don't always operate exactly like a real 8800B. On a real system, the LEDs would be on momentarily and go back off as soon as
the switch is released. That is not yet implemented in the emulator, as it will require additional code to track the state of the switches
across cycles. We would like to test the current behavior first and iron out any bugs before tracking the switches over time. 
