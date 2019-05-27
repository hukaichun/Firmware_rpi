#pragma once

#include <px4_posix.h>
#include <poll.h>

#include "diary/diary.hpp"
#include "uORBInterface/uORBInterface.hpp"
#include "cerebellum/cerebellum.hpp"


#include <array>
#include <future>


/** 
 *  automatic declare   
 *        std::array<float, dim>  var;
 *        void update_var();
 */
#define AUXILIARY_OBJ(dim, var) std::array<float, dim> var; void update##var();




class HierarchicalController {
public:
	
	
	


	static HierarchicalController* unique_handle; 

	static HierarchicalController* get_instance(Cerebellum<18,3>& ce, Blabbermouth& bl, uORBInterface& uo){
		if(unique_handle==nullptr)
			unique_handle = new HierarchicalController(ce, bl, uo);
		return unique_handle;
	}

	
	
private: 

	

	Cerebellum<18,3>&     _policy;
	Blabbermouth&         _log;
	uORBInterface&        _uORB;

	volatile bool _task_should_stop;


public:
	void main_loop();	
	// void status();
	void stop(){_task_should_stop = true;};


private:
	void update(); 
	//dirty objs
	AUXILIARY_OBJ(4,_quaternion);
	AUXILIARY_OBJ(3,_angular_rate);
	AUXILIARY_OBJ(3,_position);
	AUXILIARY_OBJ(3,_velocity);
	
	AUXILIARY_OBJ(3,_position_sp);
	AUXILIARY_OBJ(3,_set_point_error);
	AUXILIARY_OBJ(9,_rotation_matrix);
	AUXILIARY_OBJ(1,_voltage);
	HierarchicalController(Cerebellum<18,3>&, Blabbermouth&, uORBInterface&);
};



#undef AUXILIARY_OBJ