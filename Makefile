Main: main.o
	g++ main.cpp `pkg-config --cflags --libs opencv` -o Main -lrealsense2
clean:
	rm -f *.o Main
