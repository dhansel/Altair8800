INCLUDE=Arduino
OBJ=obj

Altair8800.exe: $(OBJ) $(OBJ)/Altair8800.o $(OBJ)/cpucore_v1.o $(OBJ)/cpucore_v2.o $(OBJ)/mem.o $(OBJ)/profile.o $(OBJ)/breakpoint.o $(OBJ)/numsys.o $(OBJ)/filesys.o $(OBJ)/drive.o $(OBJ)/disassembler.o $(OBJ)/prog_basic.o $(OBJ)/prog_ps2.o $(OBJ)/prog_examples.o $(OBJ)/prog_tools.o $(OBJ)/prog_games.o $(OBJ)/host_pc.o $(OBJ)/Arduino.o
	g++ $(OBJ)/Altair8800.o $(OBJ)/cpucore_v1.o $(OBJ)/cpucore_v2.o $(OBJ)/mem.o $(OBJ)/profile.o $(OBJ)/breakpoint.o $(OBJ)/numsys.o $(OBJ)/filesys.o $(OBJ)/drive.o $(OBJ)/disassembler.o $(OBJ)/prog_basic.o $(OBJ)/prog_ps2.o $(OBJ)/prog_examples.o $(OBJ)/prog_tools.o $(OBJ)/prog_games.o $(OBJ)/host_pc.o $(OBJ)/Arduino.o -o Altair8800.exe

$(OBJ)/mem.o: mem.cpp mem.h config.h
	g++ mem.cpp -c -o $(OBJ)/mem.o -I $(INCLUDE)

$(OBJ)/cpucore_v1.o: cpucore_v1.cpp cpucore_v1.h config.h
	g++ cpucore_v1.cpp -c -o $(OBJ)/cpucore_v1.o -I $(INCLUDE)

$(OBJ)/cpucore_v2.o: cpucore_v2.cpp cpucore_v2.h config.h
	g++ cpucore_v2.cpp -c -o $(OBJ)/cpucore_v2.o -I $(INCLUDE)

$(OBJ)/profile.o: profile.cpp profile.h config.h
	g++ profile.cpp -c -o $(OBJ)/profile.o -I $(INCLUDE)

$(OBJ)/breakpoint.o: breakpoint.cpp breakpoint.h config.h
	g++ breakpoint.cpp -c -o $(OBJ)/breakpoint.o -I $(INCLUDE)

$(OBJ)/numsys.o: numsys.cpp numsys.h config.h
	g++ numsys.cpp -c -o $(OBJ)/numsys.o -I $(INCLUDE)

$(OBJ)/filesys.o: filesys.cpp filesys.h config.h
	g++ filesys.cpp -c -o $(OBJ)/filesys.o -I $(INCLUDE)

$(OBJ)/drive.o: drive.cpp drive.h config.h
	g++ drive.cpp -c -o $(OBJ)/drive.o -I $(INCLUDE)

$(OBJ)/disassembler.o: disassembler.cpp disassembler.h config.h
	g++ disassembler.cpp -c -o $(OBJ)/disassembler.o -I $(INCLUDE)

$(OBJ)/prog_basic.o: prog_basic.cpp prog_basic.h config.h
	g++ prog_basic.cpp -c -o $(OBJ)/prog_basic.o -I $(INCLUDE)

$(OBJ)/prog_ps2.o: prog_ps2.cpp prog_ps2.h config.h
	g++ prog_ps2.cpp -c -o $(OBJ)/prog_ps2.o -I $(INCLUDE)

$(OBJ)/prog_examples.o: prog_examples.cpp config.h prog_examples_basic_due.h prog_examples_asm.h
	g++ -c -o $(OBJ)/prog_examples.o -D __SAM3X8E__ prog_examples.cpp  -I $(INCLUDE)

$(OBJ)/prog_tools.o: prog_tools.cpp prog_tools.h config.h
	g++ prog_tools.cpp -c -o $(OBJ)/prog_tools.o -I $(INCLUDE)

$(OBJ)/prog_games.o: prog_games.cpp prog_games.h config.h
	g++ prog_games.cpp -c -o $(OBJ)/prog_games.o -I $(INCLUDE)

$(OBJ)/host_pc.o: host_pc.cpp host_pc.h config.h
	g++ host_pc.cpp -c -o $(OBJ)/host_pc.o -I $(INCLUDE)

$(OBJ)/Altair8800.o: Altair8800.ino Altair8800.h config.h
	g++ -c -o $(OBJ)/Altair8800.o -I $(INCLUDE) -x c++ Altair8800.ino

$(OBJ)/Arduino.o: Arduino/Arduino.cpp Arduino/Arduino.h
	g++ -c -o $(OBJ)/Arduino.o -I $(INCLUDE) Arduino/Arduino.cpp

$(OBJ):
	mkdir $(OBJ)

clean:
	rm -rf $(OBJ) Altair8800.exe
