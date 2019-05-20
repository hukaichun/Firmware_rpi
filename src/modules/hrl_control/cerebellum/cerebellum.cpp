#include "cerebellum.hpp"




Cerebellum::Cerebellum(const char* dir_H, const char* dir_L):
	low_level(dir_L), 
	high_level(dir_H),
	_state_buff(18),
	_delta_p(3)
{}


std::vector<float>* Cerebellum::respond(float* rota, float* position, float* angular_v, float* velocity){

	static std::vector<float> output(4);

	for(int i=0; i<9; ++i)
		_state_buff[i] = rota[i];

	for(int i=0; i<3; ++i)
		_state_buff[i+9] = position[i];

	for(int i=0; i<3; ++i)
		_state_buff[i+12] = angular_v[i];

	for(int i=0; i<3; ++i)
		_state_buff[i+15] = velocity[i];



	high_level(&_state_buff, &_delta_p);




	for(int i=0; i<3; ++i)
		_state_buff[i+9] += _delta_p[i];



	low_level(&_state_buff, &output);

	return &output;
}


