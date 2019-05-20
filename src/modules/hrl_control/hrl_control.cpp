#include <vector>
#include <cstring>
#include <string>
#include <utility>
#include <future>
#include <cstdint>

#include <px4_log.h>


#include "hierarchical_controller.hpp"


#define PWM_DEVICE  "/sys/class/pwm/pwmchip0"

#define HIGH_LEVEL_NN "/home/pi/nn_so/neural_network_H.so"
#define LOW_LEVEL_NN  "/home/pi/nn_so/neural_network_L.so"

extern "C" __EXPORT int hrl_control_main(int argc, char *argv[]);

int hrl_control_main(int argc, char *argv[])
{

	static Cerebellum<18,3>	       the_handle_of_neural_networks("/home/pi/nn_so/neural_network_H.so", "/home/pi/nn_so/neural_network_L.so");
	static Diary                   the_handle_of_sanding_log_info(7777,1);
	static uORBInterface           the_handle_of_all_uORB_topics(PWM_DEVICE);
	static HierarchicalController  the_controller(the_handle_of_neural_networks, 
		                                      the_handle_of_sanding_log_info, 
		                                      the_handle_of_all_uORB_topics);

    	return OK;
}

