#include "hierarchical_controller.hpp"


#include <vector>
#include <array>


namespace dirty_functions {
	
}




HierarchicalController::HierarchicalController(Cerebellum& ce, Diary& di, uORBInterface& uo)
:_policy(ce), 
 _log(di), 
 _uORB(uo), 
 _task_should_stop(false) {
	_diary.accept();
}


void HierarchicalController::main_loop() {

	/* wakeup source: gyro data from sensor selected by the sensor app */
	px4_pollfd_struct_t poll_fds = {
		.fd     = _uORB._vehicle_attitude_sub,
                .events = POLLIN};

        /* wait for up to 10ms for data */
	while(!_task_should_stop_) {
		int pret = px4_poll(&poll_fds, 1, 10);

		if(pret<0) {
			PX4_ERR("ERROR return value from poll(): %d", pret);
		} else {
			_uORB.update_all();

		}




	}
}

