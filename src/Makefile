CFLAGS = -O3 -Wall --std=c++11 -g
CC = clang++
# INC = -I/Users/csayres/miniconda2/include/python2.7 -I/Users/csayres/.local/include/python2.7 -I/Users/csayres/code/kaiju/include
INC = -I$KAIJU_DIR/include
# INCMOX = -I/usr/lusers/csayres/miniconda2/include/python2.7 -I/usr/lusers/csayres/.local/include/python2.7

kaiju: robot.o utils.o robotGrid.o target.o main.cpp
	$(CC) $(CFLAGS) $(INC) -o kaiju main.cpp robot.o robotGrid.o utils.o target.o

# cKaiju: robot.o utils.o robotGrid.o betaArm.o robotGrid.h betaArm.h cKaiju.cpp
# 	$(CC) $(CFLAGS) $(INC) -shared -undefined dynamic_lookup cKaiju.cpp robot.o robotGrid.o utils.o betaArm.o -o cKaiju.so

# cKaijuMox: robot.o utils.o robotGrid.o betaArm.o robotGrid.h betaArm.h cKaiju.cpp
# 	$(CC) $(CFLAGS) $(INCMOX) -shared -fPIC cKaiju.cpp robot.o robotGrid.o utils.o betaArm.o -o cKaiju.so

robot.o: robot.cpp
	$(CC) $(CFLAGS) $(INC) -c robot.cpp

robotGrid.o: robotGrid.cpp
	$(CC) $(CFLAGS) $(INC) -c robotGrid.cpp

utils.o: utils.cpp
	$(CC) $(CFLAGS) $(INC) -c utils.cpp

target.o: target.cpp
	$(CC) $(CFLAGS) $(INC) -c target.cpp

clean:
	rm -f kaiju cKaiju.so *.o
	rm -r *.dSYM
