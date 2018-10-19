CFLAGS = -O3 --std=c++11
CC = clang++

kaiju: robot.o utils.o
	$(CC) $(CFLAGS) -o kaiju robot.o utils.o

robot.o: robot.cpp
	$(CC) $(CFLAGS) -c robot.cpp

utils.o: utils.cpp
	$(CC) $(CFLAGS) -c utils.cpp

clean:
	rm -f kaiju *.o