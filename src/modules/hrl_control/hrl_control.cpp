#include <vector>
#include <cstring>
#include <string>
#include <utility>
#include <future>
#include <cstdint>

#include <array>

#include <px4_log.h>

#include "diary/diary.hpp"
#include "uORBInterface/uORBInterface.hpp"
#include "cerebellum/cerebellum.hpp"


#define PWM_DEVICE  "/sys/class/pwm/pwmchip0"



extern "C" __EXPORT int hrl_control_main(int argc, char *argv[]);

int hrl_control_main(int argc, char *argv[])
{

    PX4_INFO("INIT uORBInterface");
    uORBInterface Interface(PWM_DEVICE);

    PX4_INFO("INIT Cerebellum");
    Cerebellum brain("/home/pi/nn_so/neural_network_H.so", "/home/pi/nn_so/neural_network_L.so");

    PX4_INFO("INIT Diary");
    Diary log(7777, 1);


        

    return OK;
}