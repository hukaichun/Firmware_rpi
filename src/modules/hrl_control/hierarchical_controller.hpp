#pragma once

#include <px4_posix.h>
#include <poll.h>

#include "diary/diary.hpp"
#include "uORBInterface/uORBInterface.hpp"
#include "cerebellum/cerebellum.hpp"

class HierarchicalController {
public:
	
	// void status();
	// void stop();
	HierarchicalController(Cerebellum&, Diary&, uORBInterface&);
	
private: 

	

	void main_loop();

	Cerebellum&     _policy;
	Diary&          _log;
	uORBInterface&  _uORB;

	bool _task_should_stop;
};