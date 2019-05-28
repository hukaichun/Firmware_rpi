#include <vector>
#include <utility>
#include <future>

#include <cstdint>
#include <cstring>
#include <cstdlib>

#include <px4_log.h>


#include "hierarchical_controller.hpp"


#define PWM_DEVICE  "/sys/class/pwm/pwmchip0"

#define HIGH_LEVEL_NN "/home/pi/nn_so/neural_network_H.so"
#define LOW_LEVEL_NN  "/home/pi/nn_so/neural_network_L.so"

extern "C" __EXPORT int hrl_control_main(int argc, char *argv[]);


static int start();


static void stop();


static void register_controller();


static void usage();



int hrl_control_main(int argc, char *argv[])
{
	static Cerebellum<18,3>	       the_handle_of_neural_networks(HIGH_LEVEL_NN, LOW_LEVEL_NN);
	static Blabbermouth            the_handle_of_sanding_log_info(7777,1);
	static uORBInterface           the_handle_of_all_uORB_topics(PWM_DEVICE);
	HierarchicalController::get_instance(the_handle_of_neural_networks, the_handle_of_sanding_log_info, the_handle_of_all_uORB_topics);




	switch(argc) {
		case 2:
			if(!strcmp(argv[1], "start")) {
				register_controller();
			} else if (!strcmp(argv[1], "stop")) {
				stop();
			}else if (!strcmp(argv[1], "tcp_connect")) {
				the_handle_of_sanding_log_info.accept();
			} else {
				usage();
			}
			break;

		case 5:
			if(!strcmp(argv[1], "register_partner")) {
				int id = atoi(argv[2]);
				int port = atoi(argv[4]);
				PX4_INFO("partner id:%d, ip:%s, port:%d", id, argv[3], port);
				the_handle_of_sanding_log_info.register_partner(id,argv[3],port);
			} 
			break;


		default:
			usage();
			break;
	}



    	return 0;
}


int start() {
	PX4_INFO("START HRL CONTROLLER");
	if (HierarchicalController::unique_handle == nullptr) {
		PX4_ERR("NO Controller Instance");
		return -1;
	} else {
		HierarchicalController::unique_handle->main_loop();
		return 0;
	}
}


void stop() {
	if (HierarchicalController::unique_handle == nullptr) {
		PX4_ERR("NO Controller Instance");
		return;
	} else {
		PX4_INFO("STOPING MAIN_LOOP");
		HierarchicalController::unique_handle->stop();
		return;
	}
}


void register_controller() {
	static int main_task = 0;

	bool flag = HierarchicalController::unique_handle->_task_should_stop;

	if(flag) {
		main_task = px4_task_spawn_cmd("hrl_control",
					SCHED_DEFAULT,
					SCHED_PRIORITY_ATTITUDE_CONTROL,
					2000,
					(px4_main_t)start,
					nullptr);
	} else {
		PX4_INFO("HRL_CONTROLLER IS RUNNING");
	}

	if (main_task<0) {
		PX4_ERR("START FAILE");
	}

}


void usage() {
	PX4_INFO("usage: hrl_control {start|stop|register_partner <id> <ip> <port>}");	
}











