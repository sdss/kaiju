CFLAGS = -O3 -Wall --std=c++11
CC = clang++

kaiju: robot.o utils.o robotGrid.o main.cpp
	$(CC) $(CFLAGS) -o kaiju main.cpp robot.o robotGrid.o utils.o

robot.o: robot.cpp
	$(CC) $(CFLAGS) -c robot.cpp

robotGrid.o: robotGrid.cpp
	$(CC) $(CFLAGS) -c robotGrid.cpp

utils.o: utils.cpp
	$(CC) $(CFLAGS) -c utils.cpp

clean:
	rm -f kaiju *.o