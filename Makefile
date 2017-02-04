OBJ=obj

OBJECTS=$(OBJ)/cpucore_v1.o $(OBJ)/cpucore_v2.o $(OBJ)/mem.o $(OBJ)/serial.o $(OBJ)/profile.o $(OBJ)/breakpoint.o $(OBJ)/numsys.o $(OBJ)/filesys.o $(OBJ)/drive.o $(OBJ)/disassembler.o $(OBJ)/prog_basic.o $(OBJ)/prog_ps2.o $(OBJ)/prog_examples.o $(OBJ)/prog_tools.o $(OBJ)/prog_games.o $(OBJ)/host_pc.o $(OBJ)/config.o

Altair8800.exe: $(OBJ) $(OBJECTS) $(OBJ)/Altair8800.o $(OBJ)/Arduino.o 
	g++ $(OBJECTS) $(OBJ)/Altair8800.o $(OBJ)/Arduino.o -o Altair8800.exe

$(OBJ):
	mkdir $(OBJ)

$(OBJECTS): $(OBJ)/%.o: %.cpp
	g++ -c $< -I Arduino -o $@

$(OBJ)/Altair8800.o: Altair8800.ino
	g++ -c -o $(OBJ)/Altair8800.o -I Arduino -x c++ Altair8800.ino

$(OBJ)/Arduino.o: Arduino/Arduino.cpp
	g++ -c -o $(OBJ)/Arduino.o -I Arduino Arduino/Arduino.cpp

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
 cpucore.h cpucore_v2.h host.h host_pc.h mem.h prog_basic.h breakpoint.h \
 serial.h profile.h disassembler.h numsys.h filesys.h drive.h prog_ps2.h \
 prog_games.h prog_tools.h
$(OBJ)/breakpoint.o: breakpoint.cpp breakpoint.h config.h Arduino/Arduino.h \
 host.h host_pc.h numsys.h Altair8800.h cpucore.h cpucore_v2.h
$(OBJ)/config.o: config.cpp config.h Arduino/Arduino.h mem.h host.h host_pc.h \
 prog_basic.h breakpoint.h serial.h filesys.h numsys.h
$(OBJ)/cpucore_v1.o: cpucore_v1.cpp cpucore.h config.h Arduino/Arduino.h \
 cpucore_v2.h profile.h mem.h host.h host_pc.h prog_basic.h breakpoint.h \
 Altair8800.h
$(OBJ)/cpucore_v2.o: cpucore_v2.cpp cpucore.h config.h Arduino/Arduino.h \
 cpucore_v2.h profile.h mem.h host.h host_pc.h prog_basic.h breakpoint.h \
 Altair8800.h
$(OBJ)/disassembler.o: disassembler.cpp Arduino/Arduino.h disassembler.h \
 numsys.h mem.h config.h host.h host_pc.h prog_basic.h breakpoint.h
$(OBJ)/drive.o: drive.cpp drive.h Arduino/Arduino.h config.h host.h host_pc.h \
 cpucore.h cpucore_v2.h
$(OBJ)/filesys.o: filesys.cpp filesys.h Arduino/Arduino.h host.h config.h \
 host_pc.h numsys.h serial.h
$(OBJ)/host_due.o: host_due.cpp
$(OBJ)/host_mega.o: host_mega.cpp
$(OBJ)/host_pc.o: host_pc.cpp Arduino/Arduino.h Altair8800.h mem.h config.h \
 host.h host_pc.h prog_basic.h breakpoint.h serial.h cpucore.h \
 cpucore_v2.h
$(OBJ)/mem.o: mem.cpp Altair8800.h Arduino/Arduino.h mem.h config.h host.h \
 host_pc.h prog_basic.h breakpoint.h
$(OBJ)/numsys.o: numsys.cpp Arduino/Arduino.h numsys.h mem.h config.h host.h \
 host_pc.h prog_basic.h breakpoint.h serial.h
$(OBJ)/profile.o: profile.cpp Arduino/Arduino.h profile.h config.h \
 disassembler.h host.h host_pc.h
$(OBJ)/prog_basic.o: prog_basic.cpp Arduino/Arduino.h prog_basic.h host.h \
 config.h host_pc.h
$(OBJ)/prog_examples.o: prog_examples.cpp Arduino/Arduino.h config.h \
 prog_examples_basic_due.h prog_examples_asm.h
$(OBJ)/prog_games.o: prog_games.cpp Arduino/Arduino.h prog_games.h host.h \
 config.h host_pc.h
$(OBJ)/prog_ps2.o: prog_ps2.cpp Arduino/Arduino.h prog_tools.h host.h config.h \
 host_pc.h
$(OBJ)/prog_tools.o: prog_tools.cpp Arduino/Arduino.h prog_tools.h host.h \
 config.h host_pc.h
$(OBJ)/serial.o: serial.cpp Altair8800.h Arduino/Arduino.h config.h host.h \
 host_pc.h serial.h filesys.h prog_examples.h cpucore.h cpucore_v2.h \
 prog_ps2.h
$(OBJ)/Arduino.o: Arduino/Arduino.cpp Arduino/Arduino.h

