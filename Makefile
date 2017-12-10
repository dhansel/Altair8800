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


OBJECTS=$(OBJ)/cpucore.o $(OBJ)/mem.o $(OBJ)/serial.o $(OBJ)/profile.o $(OBJ)/breakpoint.o $(OBJ)/numsys.o $(OBJ)/filesys.o $(OBJ)/drive.o $(OBJ)/disassembler.o $(OBJ)/prog_basic.o $(OBJ)/prog_ps2.o $(OBJ)/prog_examples.o $(OBJ)/prog_tools.o $(OBJ)/prog_games.o $(OBJ)/host_pc.o $(OBJ)/config.o $(OBJ)/timer.o $(OBJ)/prog.o $(OBJ)/printer.o $(OBJ)/hdsk.o $(OBJ)/image.o $(OBJ)/switch_serial.o $(OBJ)/sdmanager.o

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


$(OBJ)/Altair8800.o: Altair8800.ino Altair8800.h Arduino/Arduino.h config.h \
 cpucore.h host.h host_pc.h switch_serial.h mem.h prog_basic.h \
 breakpoint.h serial.h printer.h profile.h disassembler.h numsys.h \
 filesys.h drive.h hdsk.h timer.h prog.h
$(OBJ)/breakpoint.o: breakpoint.cpp breakpoint.h config.h Arduino/Arduino.h \
 host.h host_pc.h switch_serial.h numsys.h Altair8800.h cpucore.h
$(OBJ)/config.o: config.cpp Altair8800.h Arduino/Arduino.h config.h mem.h host.h \
 host_pc.h switch_serial.h prog_basic.h breakpoint.h serial.h filesys.h \
 numsys.h drive.h hdsk.h prog.h
$(OBJ)/cpucore.o: cpucore.cpp cpucore.h Arduino/Arduino.h timer.h mem.h config.h \
 host.h host_pc.h switch_serial.h prog_basic.h breakpoint.h Altair8800.h
$(OBJ)/cpucore_v1.o: cpucore_v1.cpp cpucore.h Arduino/Arduino.h timer.h mem.h \
 config.h host.h host_pc.h switch_serial.h prog_basic.h breakpoint.h \
 Altair8800.h
$(OBJ)/cpucore_v2.o: cpucore_v2.cpp cpucore.h Arduino/Arduino.h timer.h mem.h \
 config.h host.h host_pc.h switch_serial.h prog_basic.h breakpoint.h \
 Altair8800.h
$(OBJ)/disassembler.o: disassembler.cpp Arduino/Arduino.h disassembler.h \
 numsys.h mem.h config.h host.h host_pc.h switch_serial.h prog_basic.h \
 breakpoint.h
$(OBJ)/drive.o: drive.cpp drive.h Arduino/Arduino.h config.h host.h host_pc.h \
 switch_serial.h cpucore.h Altair8800.h timer.h image.h
$(OBJ)/filesys.o: filesys.cpp filesys.h Arduino/Arduino.h host.h config.h \
 host_pc.h switch_serial.h numsys.h serial.h
$(OBJ)/hdsk.o: hdsk.cpp hdsk.h Arduino/Arduino.h config.h host.h host_pc.h \
 switch_serial.h cpucore.h Altair8800.h timer.h image.h
$(OBJ)/host_due.o: host_due.cpp
$(OBJ)/host_mega.o: host_mega.cpp
$(OBJ)/host_pc.o: host_pc.cpp Arduino/Arduino.h Altair8800.h mem.h config.h \
 host.h host_pc.h switch_serial.h prog_basic.h breakpoint.h serial.h \
 cpucore.h profile.h timer.h
$(OBJ)/image.o: image.cpp host.h config.h Arduino/Arduino.h host_pc.h \
 switch_serial.h image.h
$(OBJ)/mem.o: mem.cpp Altair8800.h Arduino/Arduino.h mem.h config.h host.h \
 host_pc.h switch_serial.h prog_basic.h breakpoint.h
$(OBJ)/numsys.o: numsys.cpp Arduino/Arduino.h numsys.h mem.h config.h host.h \
 host_pc.h switch_serial.h prog_basic.h breakpoint.h serial.h
$(OBJ)/printer.o: printer.cpp printer.h Arduino/Arduino.h config.h host.h \
 host_pc.h switch_serial.h timer.h cpucore.h Altair8800.h
$(OBJ)/profile.o: profile.cpp Arduino/Arduino.h profile.h config.h \
 disassembler.h timer.h host.h host_pc.h switch_serial.h
$(OBJ)/prog.o: prog.cpp Arduino/Arduino.h prog.h Altair8800.h prog_basic.h \
 prog_tools.h prog_games.h prog_ps2.h numsys.h host.h config.h host_pc.h \
 switch_serial.h mem.h breakpoint.h
$(OBJ)/prog_basic.o: prog_basic.cpp Arduino/Arduino.h prog_basic.h host.h \
 config.h host_pc.h switch_serial.h prog.h mem.h breakpoint.h
$(OBJ)/prog_basic_examples_due.o: prog_basic_examples_due.cpp
$(OBJ)/prog_basic_examples_mega.o: prog_basic_examples_mega.cpp
$(OBJ)/prog_examples.o: prog_examples.cpp Arduino/Arduino.h config.h \
 prog_basic.h mem.h host.h host_pc.h switch_serial.h breakpoint.h \
 prog_examples_basic_due.h prog_examples_asm.h
$(OBJ)/prog_games.o: prog_games.cpp Arduino/Arduino.h prog_games.h host.h \
 config.h host_pc.h switch_serial.h
$(OBJ)/prog_ps2.o: prog_ps2.cpp Arduino/Arduino.h prog_tools.h host.h config.h \
 host_pc.h switch_serial.h serial.h Altair8800.h
$(OBJ)/prog_tools.o: prog_tools.cpp Arduino/Arduino.h prog_tools.h host.h \
 config.h host_pc.h switch_serial.h mem.h prog_basic.h breakpoint.h \
 prog.h
$(OBJ)/serial.o: serial.cpp Altair8800.h Arduino/Arduino.h config.h host.h \
 host_pc.h switch_serial.h serial.h filesys.h prog_examples.h cpucore.h \
 prog_ps2.h timer.h
$(OBJ)/soft_uart.o: soft_uart.cpp
$(OBJ)/switch_serial.o: switch_serial.cpp Arduino/Arduino.h
$(OBJ)/timer.o: timer.cpp timer.h Arduino/Arduino.h
$(OBJ)/Arduino.o: Arduino/Arduino.cpp Arduino/Arduino.h
