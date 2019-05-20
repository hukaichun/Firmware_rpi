#pragma once

#include <dlfcn.h>
#include <array>
#include <vector>

#include <stdexcept>




template<typename OUTPUT, typename INPUT>
class NN
{

typedef OUTPUT(*nn_ptr)(INPUT);

private:
	struct {
		void* handle;
		nn_ptr api;
		const char* name;
	} info, backup;


public:
	
	NN() = delete;


	NN(const char* DIR)
	:info({.handle=nullptr, .api=nullptr, .name=nullptr}),
 	 backup({.handle=nullptr, .api=nullptr, .name=nullptr}) {
		dlerror();
		if(!load_nn(DIR)) {
			throw std::runtime_error(dlerror());
		}
	}


	~NN() {
		if(info.handle) dlclose(info.handle);
		if(backup.handle) dlclose(backup.handle);
	}


	OUTPUT operator()(INPUT state) {
		return info.api(state);
	}


	bool load_nn(const char* DIR) {
		// backup
		if(backup.handle) dlclose(backup.handle);
		backup = info;

		info.handle = dlopen(DIR, RTLD_LAZY);
		if(!info.handle)
		{
			info = backup;
			return false;
		}

		info.api = (nn_ptr)dlsym(info.handle, "neural_network_respond");
		if(!info.api)
		{ 
			info = backup;
			return false;
		}

		info.name = (char*)dlsym(info.handle, "name");
		if(!info.name)
		{
			info = backup;
			return false;
		}


		return true;
	}
};





template<int OBS_DIM, int H_LEVLE_OUT_DIM>
class Cerebellum
{
private:
	NN<std::array<float, 4>, std::array<float, OBS_DIM>>               low_level;
	NN<std::array<float, H_LEVLE_OUT_DIM>, std::array<float, OBS_DIM>> high_level;

public:

	std::array<float, OBS_DIM>              _state_buff;
	std::array<float, H_LEVLE_OUT_DIM>      _delta_p;
	
	Cerebellum() = delete;
	Cerebellum(const char* dir_H, const char* dir_L)
	:low_level(dir_L),high_level(dir_H),_state_buff(),_delta_p() {
		for(auto &v: _state_buff) v=0;
		for(auto &v: _delta_p) v=0;
	}


	std::array<float, 4> 
		respond(const std::array<float,9>& rotation_matrix, 
			const std::array<float,3>& position, 
			const std::array<float,3>& angular_velocity, 
			const std::array<float,3>& velocity) {

		for(int i=0; i<9; ++i)
			_state_buff[i] = rotation_matrix[i];

		for(int i=0; i<3; ++i)
			_state_buff[i+9] = position[i];

		for(int i=0; i<3; ++i)
			_state_buff[i+12] = angular_velocity[i];

		for(int i=0; i<3; ++i)
			_state_buff[i+15] = velocity[i];

		_delta_p = high_level(_state_buff);

		for(int i=0; i<3; ++i)
			_state_buff[i+9] += _delta_p[i];

		return low_level(_state_buff);

	}
};












