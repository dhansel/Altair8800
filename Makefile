ifeq ($(OSTYPE),msys)
  CFLAGS=-O3
  LFLAGS=-O3 -lws2_32
  OBJ=obj-msys
  EXT=.exe
else
  CFLAGS=-O3
  LFLAGS=-O3 -lncurses -lpthread
  OBJ=obj-linux
  EXT=
endif


OBJECTS=$(OBJ)/cpucore.o $(OBJ)/cpucore_z80.o $(OBJ)/cpucore_i8080.o $(OBJ)/mem.o $(OBJ)/io.o $(OBJ)/serial.o $(OBJ)/profile.o $(OBJ)/breakpoint.o $(OBJ)/numsys.o $(OBJ)/filesys.o $(OBJ)/drive.o $(OBJ)/cdrive.o $(OBJ)/tdrive.o $(OBJ)/disassembler.o $(OBJ)/disassembler_z80.o $(OBJ)/disassembler_i8080.o $(OBJ)/prog_basic.o $(OBJ)/prog_ps2.o $(OBJ)/prog_examples.o $(OBJ)/prog_tools.o $(OBJ)/prog_games.o $(OBJ)/prog_dazzler.o $(OBJ)/host_pc.o $(OBJ)/config.o $(OBJ)/timer.o $(OBJ)/prog.o $(OBJ)/printer.o $(OBJ)/hdsk.o $(OBJ)/image.o $(OBJ)/switch_serial.o $(OBJ)/sdmanager.o $(OBJ)/dazzler.o $(OBJ)/vdm1.o $(OBJ)/XModem.o

Altair8800$(EXT): $(OBJ) $(OBJECTS) $(OBJ)/Altair8800.o $(OBJ)/Arduino.o $(OBJ)/Print.o
	g++ $(OBJECTS) $(OBJ)/Altair8800.o $(OBJ)/Arduino.o $(OBJ)/Print.o $(LFLAGS) -o Altair8800$(EXT)

$(OBJ):
	mkdir $(OBJ)

$(OBJECTS): $(OBJ)/%.o: %.cpp
	g++ $(CFLAGS) -c $< -I Arduino -o $@

$(OBJ)/Altair8800.o: Altair8800.ino
	g++ $(CFLAGS) -c -o $(OBJ)/Altair8800.o -I Arduino -x c++ Altair8800.ino

$(OBJ)/Arduino.o: Arduino/Arduino.cpp
	g++ $(CFLAGS) -c -o $(OBJ)/Arduino.o -I Arduino Arduino/Arduino.cpp

$(OBJ)/Print.o: Arduino/Print.cpp
	g++ $(CFLAGS) -c -o $(OBJ)/Print.o -I Arduino Arduino/Print.cpp

clean:
	rm -rf $(OBJ) Altair8800.exe

deps:
	@echo
	@gcc -MM -x c++ Altair8800.ino *.cpp Arduino/Arduino.cpp -I . -I Arduino | sed "s/.*\:/\$$(OBJ)\/&/"
	@echo


# --------------------------------------------------------------------------------------------------
# The following list of dependencies can be created by typing "make deps"
# --------------------------------------------------------------------------------------------------


$(OBJ)/Altair8800.o: Altair8800.ino Altair8800.h Arduino/Arduino.h \
 Arduino/inttypes.h Arduino/Print.h config.h cpucore.h host.h host_pc.h \
 switch_serial.h mem.h prog_basic.h breakpoint.h dazzler.h vdm1.h \
 serial.h printer.h profile.h disassembler.h numsys.h filesys.h drive.h \
 tdrive.h cdrive.h hdsk.h timer.h prog.h io.h
$(OBJ)/breakpoint.o: breakpoint.cpp breakpoint.h config.h Arduino/Arduino.h \
 Arduino/inttypes.h Arduino/Print.h host.h host_pc.h switch_serial.h \
 Altair8800.h numsys.h cpucore.h
$(OBJ)/cdrive.o: cdrive.cpp cdrive.h Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h config.h host.h host_pc.h switch_serial.h Altair8800.h \
 cpucore.h timer.h image.h mem.h prog_basic.h breakpoint.h dazzler.h \
 vdm1.h prog_tools.h io.h
$(OBJ)/config.o: config.cpp Altair8800.h Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h config.h mem.h host.h host_pc.h switch_serial.h \
 prog_basic.h breakpoint.h cpucore.h dazzler.h vdm1.h serial.h printer.h \
 filesys.h numsys.h drive.h cdrive.h tdrive.h hdsk.h prog.h sdmanager.h \
 io.h
$(OBJ)/cpucore.o: cpucore.cpp cpucore.h Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h config.h Altair8800.h cpucore_z80.h cpucore_i8080.h
$(OBJ)/cpucore_i8080.o: cpucore_i8080.cpp cpucore.h Arduino/Arduino.h \
 Arduino/inttypes.h Arduino/Print.h config.h timer.h mem.h host.h \
 host_pc.h switch_serial.h Altair8800.h prog_basic.h breakpoint.h \
 dazzler.h vdm1.h numsys.h disassembler.h
$(OBJ)/cpucore_z80.o: cpucore_z80.cpp cpucore.h Arduino/Arduino.h \
 Arduino/inttypes.h Arduino/Print.h config.h timer.h mem.h host.h \
 host_pc.h switch_serial.h Altair8800.h prog_basic.h breakpoint.h \
 dazzler.h vdm1.h numsys.h disassembler.h
$(OBJ)/dazzler.o: dazzler.cpp dazzler.h Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h mem.h config.h host.h host_pc.h switch_serial.h \
 Altair8800.h prog_basic.h breakpoint.h cpucore.h vdm1.h serial.h timer.h \
 numsys.h io.h
$(OBJ)/disassembler.o: disassembler.cpp Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h disassembler.h disassembler_i8080.h disassembler_z80.h \
 cpucore.h config.h
$(OBJ)/disassembler_i8080.o: disassembler_i8080.cpp Arduino/Arduino.h \
 Arduino/inttypes.h Arduino/Print.h disassembler.h numsys.h mem.h \
 config.h host.h host_pc.h switch_serial.h Altair8800.h prog_basic.h \
 breakpoint.h cpucore.h dazzler.h vdm1.h
$(OBJ)/disassembler_z80.o: disassembler_z80.cpp Arduino/Arduino.h \
 Arduino/inttypes.h Arduino/Print.h disassembler.h numsys.h mem.h \
 config.h host.h host_pc.h switch_serial.h Altair8800.h prog_basic.h \
 breakpoint.h cpucore.h dazzler.h vdm1.h
$(OBJ)/drive.o: drive.cpp drive.h Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h config.h host.h host_pc.h switch_serial.h Altair8800.h \
 cpucore.h timer.h image.h io.h
$(OBJ)/filesys.o: filesys.cpp filesys.h Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h host.h config.h host_pc.h switch_serial.h Altair8800.h \
 numsys.h serial.h
$(OBJ)/hdsk.o: hdsk.cpp hdsk.h Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h config.h host.h host_pc.h switch_serial.h Altair8800.h \
 cpucore.h timer.h image.h io.h
$(OBJ)/host_due.o: host_due.cpp
$(OBJ)/host_mega.o: host_mega.cpp
$(OBJ)/host_pc.o: host_pc.cpp Altair8800.h Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h mem.h config.h host.h host_pc.h switch_serial.h \
 prog_basic.h breakpoint.h cpucore.h dazzler.h vdm1.h serial.h profile.h \
 timer.h Arduino/dirent_win.h
$(OBJ)/image.o: image.cpp host.h config.h Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h host_pc.h switch_serial.h Altair8800.h image.h
$(OBJ)/io.o: io.cpp io.h config.h Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h host.h host_pc.h switch_serial.h Altair8800.h numsys.h
$(OBJ)/mem.o: mem.cpp Altair8800.h Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h mem.h config.h host.h host_pc.h switch_serial.h \
 prog_basic.h breakpoint.h cpucore.h dazzler.h vdm1.h numsys.h filesys.h
$(OBJ)/numsys.o: numsys.cpp Arduino/Arduino.h Arduino/inttypes.h Arduino/Print.h \
 numsys.h mem.h config.h host.h host_pc.h switch_serial.h Altair8800.h \
 prog_basic.h breakpoint.h cpucore.h dazzler.h vdm1.h serial.h
$(OBJ)/printer.o: printer.cpp printer.h Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h config.h host.h host_pc.h switch_serial.h Altair8800.h \
 timer.h cpucore.h io.h
$(OBJ)/profile.o: profile.cpp Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h profile.h config.h disassembler.h timer.h host.h \
 host_pc.h switch_serial.h Altair8800.h cpucore.h
$(OBJ)/prog.o: prog.cpp Arduino/Arduino.h Arduino/inttypes.h Arduino/Print.h \
 prog.h Altair8800.h prog_basic.h prog_tools.h prog_games.h prog_ps2.h \
 prog_dazzler.h numsys.h host.h config.h host_pc.h switch_serial.h mem.h \
 breakpoint.h cpucore.h dazzler.h vdm1.h
$(OBJ)/prog_basic.o: prog_basic.cpp Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h prog_basic.h host.h config.h host_pc.h switch_serial.h \
 Altair8800.h prog.h mem.h breakpoint.h cpucore.h dazzler.h vdm1.h
$(OBJ)/prog_dazzler.o: prog_dazzler.cpp Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h prog_dazzler.h host.h config.h host_pc.h switch_serial.h \
 Altair8800.h mem.h prog_basic.h breakpoint.h cpucore.h dazzler.h vdm1.h \
 prog.h
$(OBJ)/prog_examples.o: prog_examples.cpp Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h config.h prog_basic.h mem.h host.h host_pc.h \
 switch_serial.h Altair8800.h breakpoint.h cpucore.h dazzler.h vdm1.h \
 prog_examples_basic_due.h prog_examples_asm.h
$(OBJ)/prog_games.o: prog_games.cpp Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h prog_games.h prog.h mem.h config.h host.h host_pc.h \
 switch_serial.h Altair8800.h prog_basic.h breakpoint.h cpucore.h \
 dazzler.h vdm1.h
$(OBJ)/prog_ps2.o: prog_ps2.cpp Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h prog_tools.h prog.h serial.h Altair8800.h
$(OBJ)/prog_tools.o: prog_tools.cpp Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h prog_tools.h host.h config.h host_pc.h switch_serial.h \
 Altair8800.h mem.h prog_basic.h breakpoint.h cpucore.h dazzler.h vdm1.h \
 prog.h
$(OBJ)/sdmanager.o: sdmanager.cpp config.h Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h sdmanager.h XModem.h host.h host_pc.h switch_serial.h \
 Altair8800.h numsys.h serial.h
$(OBJ)/serial.o: serial.cpp Altair8800.h Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h config.h host.h host_pc.h switch_serial.h serial.h \
 filesys.h prog_examples.h cpucore.h prog_ps2.h timer.h numsys.h io.h
$(OBJ)/soft_uart.o: soft_uart.cpp
$(OBJ)/switch_serial.o: switch_serial.cpp Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h host.h config.h host_pc.h switch_serial.h Altair8800.h
$(OBJ)/tdrive.o: tdrive.cpp tdrive.h Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h config.h host.h host_pc.h switch_serial.h Altair8800.h \
 cpucore.h timer.h image.h io.h
$(OBJ)/timer.o: timer.cpp timer.h Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h config.h
$(OBJ)/vdm1.o: vdm1.cpp vdm1.h Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h mem.h config.h host.h host_pc.h switch_serial.h \
 Altair8800.h prog_basic.h breakpoint.h cpucore.h dazzler.h serial.h \
 timer.h io.h cdrive.h
$(OBJ)/XModem.o: XModem.cpp XModem.h
$(OBJ)/Arduino.o: Arduino/Arduino.cpp Arduino/Arduino.h Arduino/inttypes.h \
 Arduino/Print.h

