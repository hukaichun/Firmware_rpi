# it is a test
# LIB=
INC=-I/home/indigo/src/Firmware/src/lib

all:
	python3 tf2cpp.py
	arm-linux-gnueabihf-g++ -std=c++11 -fPIC -O3 $(INC) -c libnncontroller.cpp
	arm-linux-gnueabihf-g++ -shared -fPIC -O3 -o libnncontroller.so libnncontroller.o
	rsync libnncontroller.so pi@navio.local:/home/pi/lib
gencpp:
	python3 tf2cpp.py
compile:
	arm-linux-gnueabihf-g++ -std=c++11 -fPIC -O3 $(INC) -c libnncontroller.cpp
	arm-linux-gnueabihf-g++ -shared -fPIC -O3 -o libnncontroller.so libnncontroller.o
upload:
	rsync libnncontroller.so pi@navio.local:/home/pi/lib
clean:
	rm -f libnncontroller.o libnncontroller.so