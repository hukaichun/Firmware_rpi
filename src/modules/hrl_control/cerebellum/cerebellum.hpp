#pragma once

#include <dlfcn.h>
#include <vector>





template<typename OUTPUT, typename INPUT>
class NN
{

typedef void(*nn_ptr)(INPUT,OUTPUT);

private:
	struct {
		void* handle;
		nn_ptr api;
		const char* name;
	} info, backup;


public:
	
	NN() = delete;
	NN(const char*); 
	~NN();


	void operator()(INPUT, OUTPUT);

	bool load_nn(const char*);
};


class Cerebellum
{
private:
	NN<std::vector<float>*, std::vector<float>*> low_level;
	NN<std::vector<float>*, std::vector<float>*> high_level;

	

public:

	std::vector<float> _state_buff,
	                   _delta_p;
	
	Cerebellum() = delete;
	Cerebellum(const char* dir_H, const char* dir_L);


	std::vector<float>* respond(float rotation_matrix[9], float position[3], float angular_velocity[3], float velocity[3]);
};













/**
 * NN implement
 */
template<typename OUTPUT, typename INPUT>
NN<OUTPUT, INPUT>::NN(const char* DIR):
	info({.handle=nullptr, .api=nullptr, .name=nullptr}),
	backup({.handle=nullptr, .api=nullptr, .name=nullptr})
{
	dlerror();
	if(!load_nn(DIR)) throw dlerror();
}


template<typename OUTPUT, typename INPUT>
NN<OUTPUT, INPUT>::~NN(){
	if(info.handle) dlclose(info.handle);
	if(backup.handle) dlclose(backup.handle);
}


template<typename OUTPUT, typename INPUT>
bool NN<OUTPUT, INPUT>::load_nn(const char* DIR)
{
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


template<typename OUTPUT, typename INPUT>
void NN<OUTPUT, INPUT>::operator()(INPUT state, OUTPUT output){
		info.api(state, output);
		return;
}





