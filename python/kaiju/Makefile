CFLAGS = -O3 -Wall --std=c++11 -g
CC = clang++

kaiju: robot.o utils.o robotGrid.o betaArm.o main.cpp robotGrid.h betaArm.h
	$(CC) $(CFLAGS) -o kaiju main.cpp robot.o robotGrid.o utils.o betaArm.o

cKaiju: robot.o utils.o robotGrid.o betaArm.o robotGrid.h betaArm.h bind.cpp
	$(CC) $(CFLAGS) -shared -undefined dynamic_lookup -I/Users/csayres/miniconda2/include/python2.7 -I/Users/csayres/.local/include/python2.7 bind.cpp robot.o robotGrid.o utils.o betaArm.o -o cKaiju.so

robot.o: robot.cpp robot.h utils.h robotGrid.h
	$(CC) $(CFLAGS) -c robot.cpp

robotGrid.o: robotGrid.cpp robotGrid.h betaArm.h
	$(CC) $(CFLAGS) -c robotGrid.cpp

utils.o: utils.cpp utils.h
	$(CC) $(CFLAGS) -c utils.cpp

betaArm.o: betaArm.cpp betaArm.h
	$(CC) $(CFLAGS) -c betaArm.cpp

clean:
	rm -f kaiju *.o