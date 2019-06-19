CFLAGS = -O3 -Wall --std=c++11 -g
CC = clang++
INC = -I/Users/csayres/miniconda2/include/python2.7 -I/Users/csayres/.local/include/python2.7
# INCMOX = -I/usr/lusers/csayres/miniconda2/include/python2.7 -I/usr/lusers/csayres/.local/include/python2.7

# kaiju: robot.o utils.o robotGrid.o betaArm.o main.cpp robotGrid.h betaArm.h
# 	$(CC) $(CFLAGS) -o kaiju main.cpp robot.o robotGrid.o utils.o betaArm.o

cKaiju: robot.o utils.o robotGrid.o betaArm.o robotGrid.h betaArm.h cKaiju.cpp
	$(CC) $(CFLAGS) $(INC) -shared -undefined dynamic_lookup cKaiju.cpp robot.o robotGrid.o utils.o betaArm.o -o cKaiju.so

# cKaijuMox: robot.o utils.o robotGrid.o betaArm.o robotGrid.h betaArm.h cKaiju.cpp
# 	$(CC) $(CFLAGS) $(INCMOX) -shared -fPIC cKaiju.cpp robot.o robotGrid.o utils.o betaArm.o -o cKaiju.so

robot.o: robot.cpp robot.h utils.h robotGrid.h
	$(CC) $(CFLAGS) -c robot.cpp

robotGrid.o: robotGrid.cpp robotGrid.h betaArm.h
	$(CC) $(CFLAGS) $(INC) -c robotGrid.cpp

utils.o: utils.cpp utils.h
	$(CC) $(CFLAGS) -c utils.cpp

betaArm.o: betaArm.cpp betaArm.h
	$(CC) $(CFLAGS) -c betaArm.cpp

clean:
	rm -f kaiju cKaiju.so *.o