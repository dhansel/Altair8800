## Disk Controller

This card replicates the function of the MITS 88-DCDD 8-inch 
and 88-MDS (Minidisk) 5.25-inch disk controllers.

![Floppy Disk Controller Card](diskcontroller.jpg)

It can be connected to a 5.25-inch disk drive and either use
it in double-density mode to replicate the Minidisk system
or in high-density mode to replicate the 8-inch system. Note
that a 5.25-inch disk in high density mode can hold the exact
same amount of data as the original 8-inch disks.

The card can also be connected to a Shugart SA-800 8-inch
disk drive.

Since the interface for 3.5-inch drives is identical to the
5.25-inch drives the card also works with 3.5-inch drives.

To connect the card to one or two drives use a regular PC disk 
drive cable (with the cable twist between the two drive connectors). 

The 4 DIP switches on the card have the following functions:

DIP | Function when on         | Function when off
----|--------------------------|------------------
1   | Act as 88-MDS controller | Act as 88-DCDD controller
2   | Swap drives A and B      | Do not swap drives
3   | Drive B is Shugart SA-800| Drive B is generic 5.25-inch
4   | Drive A is Shugart SA-800| Drive A is generic 5.25-inch

The serial port present on the card can be connected to a PC via an FTDI
USB-to-serial converter and has two functions:
1. Upload the firmware to the ATmega328P processor on the card. Load the 
firmware into the Arduino programming environment, select the
"Arduino Pro or Pro Mini" board at 16MHz and upload the software.
2. Provide a monitor to interact directly with the disk drive. To enter
the monitor, connect a terminal to the serial port at 115200 baud, then press and
hold the MONITOR button on the board and briefly press RESET. Hold the
MONITOR button until the monitor prompt can be seen on the terminal.
Enter 'h' at the monitor prompt for information about valid commands.

Schematics and PCB as well as a Gerber file for PCB production are in this directory. 
The project is also available on EasyEDA: https://oshwlab.com/hansel72/diskcontrolleruno
