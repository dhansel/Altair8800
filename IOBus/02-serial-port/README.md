## Serial port (88-2SIO)

The 88-2SIO card for the Altair had two serial ports with RTS/CTS
flow control. This card implements one such serial port. 

![Serial card](serial.jpg)

Note that the output signals are 5V TTL level signals. To get true
serial level signals they will need to be converted through a MAX232
or similar converter.

This card uses the same IC (Motorola 6850) as the original 88-2SIO
card and therefore is 100% software compatible except that the card
can not cause interrupts in the emulator.

Obviously the Altair Simulator already comes with a serial interface
but this card offers the possibility of adding more serial interfaces
as well as supporting RTS/CTS flow control which can be helpfule when
talking to slow peripherals such as printers.

Schematics and PCB as well as a Gerber file for PCB production are in this directory. 
The project is also available on EasyEDA: https://oshwlab.com/hansel72/2sio_copy
