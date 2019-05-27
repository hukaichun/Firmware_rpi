
CC := arm-linux-gnueabihf-g++
EIGEN_DIR := -I.
CFLAG := -O3 -std=c++11 

SO_LIB := neural_network_L.so neural_network_H.so

.PHONY: all
all: neural_network_L.so neural_network_H.so upload

%.so: %.cpp parameter.hpp
	${CC} -fPIC -shared ${EIGEN_DIR} ${CFLAG} $< -o $@ 


.PHONY: clean upload
clean:
	rm -f *.so

upload: 
	rsync -arh ${SO_LIB} pi:/home/pi/nn_so
