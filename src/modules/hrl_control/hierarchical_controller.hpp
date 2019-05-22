#pragma once

#include <px4_posix.h>
#include <poll.h>

#include "diary/diary.hpp"
#include "uORBInterface/uORBInterface.hpp"
#include "cerebellum/cerebellum.hpp"


#include <array>
#include <future>



#define AUXILIARY_OBJ(dim, var) std::array<float, dim> var; void update##var();




class HierarchicalController {
public:
	
	// void status();
	// void stop();
	HierarchicalController(Cerebellum<18,3>&, Diary&, uORBInterface&);


	//dirty objs
	AUXILIARY_OBJ(4,_quaternion);
	AUXILIARY_OBJ(3,_angular_rate);
	AUXILIARY_OBJ(3,_position);
	AUXILIARY_OBJ(3,_velocity);
	
	AUXILIARY_OBJ(3,_position_sp);
	AUXILIARY_OBJ(3,_set_point_error);
	AUXILIARY_OBJ(9,_rotation_matrix);
	AUXILIARY_OBJ(1,_voltage);


	void main_loop();
	
private: 

	

	Cerebellum<18,3>&     _policy;
	Diary&                _log;
	uORBInterface&        _uORB;


	


	bool _task_should_stop;
};



//#undef AUXILIARY_OBJ(dim, var)