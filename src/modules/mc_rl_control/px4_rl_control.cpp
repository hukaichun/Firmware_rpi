/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file rl_control.cpp
 *
 * Bottle drop module for Outback Challenge 2014, Team Swiss Fang
 *
 * @author Dominik Juchli <juchlid@ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_log.h>
#include <px4_posix.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <sys/ioctl.h>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <iostream>
#include <thread>

// Dynamic link
#include <dlfcn.h>
#include <sys/stat.h>

// PWM output
#include <drivers/linux_pwm_out/navio_sysfs.h>
#include <perf/perf_counter.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/led_control.h>


#include <parameters/param.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <lib/ecl/geo/geo.h>
#include <dataman/dataman.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <matrix/matrix/math.hpp>

#include "log_socket.cpp"



#define NN_LIB	 	"/home/pi/lib/libnncontroller.so"
#define PWM_DEVICE 	"/sys/class/pwm/pwmchip0"

#define BASE_RATE 		1000
#define SUBTASK1_RATE 	1		// 1s
#define SUBTASK2_RATE 	0.1	// 100s
#define MASS 			0.665



using namespace matrix;
using namespace std;
/**
 * rl_control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_rl_control_main(int argc, char *argv[]);

class MulticopterRLControl
{
public:
	/**
	 * Constructor
	 */
	MulticopterRLControl();

	/**
	 * Destructor, also kills task.
	 */
	~MulticopterRLControl();

	/**
	 * Start the task.
	 *
	 * @return		OK on success.
	 */
	int	start();

	/**
	 * Display status.
	 */
	void	status();

	/**
	 * Update input rc data.
	 */
	void 	input_rc_poll();

	/**
	 * Update vehicle attitude data.
	 */
	void 	vehicle_attitude_poll();

	/**
	 * Update local position data.
	 */
	void 	vehicle_local_position_poll();

	/**
	 * Update battery status data.
	 */
	void 	battery_status_poll();

	/**
	 * Init pwm output device.
	 */
	void 	pwm_device_init();

	/**
	 * Send pwm output.
	 */
	void	pwm_device_output(const uint16_t *);

	/**
	 * Load neural network variable from shared library.
	 */
	void	load_external_variable();

	/**
	 * Publish vehicle arm status.
	 */
	int	arm_publish();

	/**
	 * Publish vehicle setpoint.
	 */
	int	vehicle_local_position_setpoint_publish();

	int 	actuator_outputs_publish();

	int 	led_control_publish();

	void 	prepare_nn_states();
	
	inline void 	calc_rl_pwm_output();

	inline void 	calc_preheat_pwm_output();

	void 	subtask_list();

	Vector<float, 4>	*rl_controller(Dcmf, Vector3f, Vector3f, Vector3f);

	// void 	write_2_file();

private:
	bool	_task_should_exit;		/**< if true, task should exit */
	int		_main_task;			/**< handle for task */
	int 	subtask_counter[3] = {0};
	hrt_abstime now;

	int _input_rc_sub = -1;
	int _vehicle_attitude_sub = -1;
	int _vehicle_local_position_sub = -1;
	int _battery_status_sub = -1;

	orb_advert_t	_mavlink_log_pub{nullptr};
	orb_advert_t	_vehicle_status_pub{nullptr};
	orb_advert_t	_position_sp_pub{nullptr};
	orb_advert_t	_actuator_outputs_pub{nullptr};
	orb_advert_t	_led_control_pub{nullptr};

	struct input_rc_s _input_rc{};
	struct vehicle_attitude_s _vehicle_attitude{};
	struct vehicle_local_position_s _vehicle_local_position{};
	struct vehicle_local_position_setpoint_s _vehicle_local_position_setpoint{};
	struct battery_status_s _battery_status{};
	struct vehicle_status_s _vehicle_status{};
	struct actuator_outputs_s _actuator_outputs{};
	struct led_control_s _led_control{};

	perf_counter_t	_perf_control_latency = nullptr;

	static const int _max_num_outputs = 8; ///< maximum number of outputs the driver should use
	linux_pwm_out::NavioSysfsPWMOut *pwm_out_handle;

	Dcmf R;
	Vector3f _position;
	Vector3f _position_sp;
	Vector3f _position_err;
	Vector3f _velocity;
	Vector3f _angular_rate;

	uint8_t	armed;
	float	battery_voltage;
	int 	battery_warning;

	void 	*fHandle;
	char 	*policy_version;

	Vector<float, 18> 	input_states;
	float 				rotation_array[9];
	Vector<float, 4> 	(*func)(Vector<float, 18> &);

	Vector<float, 4> 	nn_output;
	double				pwm_calc_tmp[4] = {0};
	uint16_t			pwm_output[4] = {0};

	uint8_t				led_color = led_control_s::COLOR_GREEN;
	uint8_t				led_mode = led_control_s::MODE_BLINK_NORMAL;

	void		task_main();
	Log_socket 	log_socket;

	/**
	 * Shim for calling task_main from task_create.
	 */
	static int	task_main_trampoline(int argc, char *argv[]);

};

namespace rl_control
{
	MulticopterRLControl	*g_rl_control;
}

MulticopterRLControl::MulticopterRLControl() :

	_task_should_exit(false),
	_main_task(-1),
	_mavlink_log_pub(nullptr),
	_vehicle_status_pub(nullptr),
	_position_sp_pub(nullptr),
	_led_control_pub(nullptr),
	R(),
	armed(vehicle_status_s::ARMING_STATE_INIT),
	battery_voltage(12),
	battery_warning(0)
{
	_position.zero();
	_position_sp.zero();
	_position_err.zero();
	_velocity.zero();
	_angular_rate.zero();

	input_states.zero();
	nn_output.zero();
}

MulticopterRLControl::~MulticopterRLControl()
{
	if (_main_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			px4_usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_main_task);
				break;
			}
		} while (_main_task != -1);
	}

	rl_control::g_rl_control = nullptr;
}

int
MulticopterRLControl::start()
{
	/* start the task */
	_main_task = px4_task_spawn_cmd("mc_rl_control",
					SCHED_DEFAULT,
					SCHED_PRIORITY_ATTITUDE_CONTROL,
					1500,
					(px4_main_t)&MulticopterRLControl::task_main_trampoline,
					nullptr);

	if (_main_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void
MulticopterRLControl::input_rc_poll()
{
	bool input_rc_updated = false;
	orb_check(_input_rc_sub, &input_rc_updated);
	if (input_rc_updated) {
		
		Vector3f tmp;
		int vehicle_mode;

		orb_copy(ORB_ID(input_rc), _input_rc_sub, &_input_rc);
		tmp(0) 			= _input_rc.values[3];
		tmp(1) 			= _input_rc.values[1];
		tmp(2) 			= _input_rc.values[2];
		vehicle_mode 	= _input_rc.values[6];

		if (vehicle_mode > 1700) {
			if (armed != vehicle_status_s::ARMING_STATE_ARMED)
			{
				mavlink_log_info(&_mavlink_log_pub, "[mc_rl_control] armed");
				led_color	= led_control_s::COLOR_PURPLE;
				led_mode	= led_control_s::MODE_ON;
				armed		= vehicle_status_s::ARMING_STATE_ARMED;
			}
			if (battery_voltage < 10.5f)
			{
				led_mode	= led_control_s::MODE_BLINK_FAST;
			}
		} else if (vehicle_mode > 1300) {
			if (armed != vehicle_status_s::ARMING_STATE_STANDBY)
			{
				mavlink_log_info(&_mavlink_log_pub, "[mc_rl_control] standby");
				_position_sp.zero();
				led_color	= led_control_s::COLOR_RED;
				led_mode	= led_control_s::MODE_BLINK_FAST;
				armed		= vehicle_status_s::ARMING_STATE_STANDBY;
			}
		} else {
			if (armed != vehicle_status_s::ARMING_STATE_INIT)
			{
				mavlink_log_info(&_mavlink_log_pub, "[mc_rl_control] disarmed");
				led_color	= led_control_s::COLOR_GREEN;
				led_mode	= led_control_s::MODE_BLINK_NORMAL;
				armed		= vehicle_status_s::ARMING_STATE_INIT;
			}
		}

		for (int i = 0; i < 3; ++i)
		{
			if (tmp(i) > 1570) {
				tmp(i) = tmp(i) - 1570;
			} else if (tmp(i) >= 1430) {
				tmp(i) = 0.0F;
			} else {
				tmp(i) = tmp(i) - 1430;
			}
		}

		_position_sp += tmp/10000;
	}

}


void
MulticopterRLControl::vehicle_attitude_poll()
{
	bool vehicle_attitude_updated = false;
	orb_check(_vehicle_attitude_sub, &vehicle_attitude_updated);
	if (vehicle_attitude_updated) {
		orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_vehicle_attitude);
		R = Dcmf(Quatf(_vehicle_attitude.q)).T();
		_angular_rate(0) = _vehicle_attitude.rollspeed;
		_angular_rate(1) = _vehicle_attitude.pitchspeed;
		_angular_rate(2) = _vehicle_attitude.yawspeed;
	}
}

void
MulticopterRLControl::vehicle_local_position_poll()
{
	bool vehicle_local_position_update = false;
	orb_check(_vehicle_local_position_sub, &vehicle_local_position_update);
	if (vehicle_local_position_update) {
		orb_copy(ORB_ID(vehicle_local_position), _vehicle_local_position_sub, &_vehicle_local_position);
		_position(0) = _vehicle_local_position.x;
		_position(1) = _vehicle_local_position.y;
		_position(2) = _vehicle_local_position.z;

		_velocity(0) = _vehicle_local_position.vx;
		_velocity(1) = _vehicle_local_position.vy;
		_velocity(2) = _vehicle_local_position.vz;
	}
}

void
MulticopterRLControl::battery_status_poll()
{
	bool battery_status_update = false;
	orb_check(_battery_status_sub, &battery_status_update);
	if (battery_status_update) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
		battery_voltage = _battery_status.voltage_filtered_v;
		battery_warning = _battery_status.warning;
	}
}

void
MulticopterRLControl::status()
{
	warnx("rl controller state: good");
}

void
MulticopterRLControl::pwm_device_init()
{
	PX4_INFO("Starting PWM output in Navio mode");
	pwm_out_handle = new linux_pwm_out::NavioSysfsPWMOut(PWM_DEVICE, _max_num_outputs);
	if (pwm_out_handle->init() != 0) {
		PX4_ERR("PWM output init failed");
		delete pwm_out_handle;
		return;
	}
}

void
MulticopterRLControl::pwm_device_output(const uint16_t *pwm)
{
	//TODO: check whether output signal is correct
	pwm_out_handle->send_output_pwm(pwm, 8);
}

int
MulticopterRLControl::arm_publish()
{
	_vehicle_status.timestamp = hrt_absolute_time();
	_vehicle_status.arming_state = armed;

	// lazily publish _vehicle_status only once available
	if (_vehicle_status_pub != nullptr) {
		return orb_publish(ORB_ID(vehicle_status), _vehicle_status_pub, &_vehicle_status);

	} else {
		_vehicle_status_pub = orb_advertise(ORB_ID(vehicle_status), &_vehicle_status);

		if (_vehicle_status_pub != nullptr) {
			return OK;

		} else {
			return -1;
		}
	}
}

int
MulticopterRLControl::led_control_publish()
{
	_led_control.timestamp = hrt_absolute_time();
	_led_control.led_mask = 0xff;
	_led_control.color = led_color;
	_led_control.mode = led_mode;

	// lazily publish _led_control only once available
	if (_led_control_pub != nullptr) {
		return orb_publish(ORB_ID(led_control), _led_control_pub, &_led_control);

	} else {
		_led_control_pub = orb_advertise(ORB_ID(led_control), &_led_control);

		if (_led_control_pub != nullptr) {
			return OK;

		} else {
			return -1;
		}
	}
}


int
MulticopterRLControl::vehicle_local_position_setpoint_publish()
{
	_vehicle_local_position_setpoint.timestamp = hrt_absolute_time();
	_vehicle_local_position_setpoint.x = -_position_sp(1);
	_vehicle_local_position_setpoint.y = _position_sp(0);
	_vehicle_local_position_setpoint.z = -_position_sp(2);

	// lazily publish _vehicle_status only once available
	if (_position_sp_pub != nullptr) {
		return orb_publish(ORB_ID(vehicle_local_position_setpoint), _position_sp_pub, &_vehicle_local_position_setpoint);

	} else {
		_position_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_vehicle_local_position_setpoint);

		if (_position_sp_pub != nullptr) {
			return OK;

		} else {
			return -1;
		}
	}
}

int
MulticopterRLControl::actuator_outputs_publish()
{
	_actuator_outputs.output[0] = pwm_output[0];
    _actuator_outputs.output[1] = pwm_output[1];
    _actuator_outputs.output[2] = pwm_output[2];
    _actuator_outputs.output[3] = pwm_output[3];

    /* now publish to actuator_outputs in case anyone wants to know... */
    _actuator_outputs.timestamp = hrt_absolute_time();

    // lazily publish _vehicle_status only once available
	if (_position_sp_pub != nullptr) {
		return orb_publish(ORB_ID(actuator_outputs), _actuator_outputs_pub, &_actuator_outputs);

	} else {
		_actuator_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &_actuator_outputs);

		if (_actuator_outputs_pub != nullptr) {
			return OK;

		} else {
			return -1;
		}
	}
}

void
MulticopterRLControl::prepare_nn_states()
{
	R.copyToColumnMajor(rotation_array);
	for (int i = 0; i < 9; ++i)
	{
		input_states(i) = rotation_array[i];
	} 

	for (int i = 9; i < 12; ++i)	{
		input_states(i) = _angular_rate(i-9) * 0.15f;
	}

	for (int i = 12; i < 15; ++i)
	{
		input_states(i) = _velocity(i-12) * 0.15f;
	}

	for (int i = 15; i < 18; ++i)
	{
		input_states(i) = _position_err(i-15) * 0.5f;
	}
}

void
MulticopterRLControl::load_external_variable()
{	

	struct stat buffer;   
	if (stat(NN_LIB, &buffer) != 0 && fHandle == nullptr){
		errx(1, ".so file does not exist and cannot create fHandle");
		return;
	} else if (stat(NN_LIB, &buffer) != 0) {
		warn(".so file does not exist");
		return;
	} else if (fHandle != nullptr) {
		warn("release fHandle");
		dlclose(fHandle);
	}


    fHandle = dlopen(NN_LIB, RTLD_LAZY);

    if (!fHandle) {
        fprintf (stderr, "%s\n", dlerror());
        return;
    }

    dlerror();

  	policy_version = (char *)dlsym(fHandle, "file_name");

    func = (Vector<float, 4>(*)(Vector<float, 18> &))dlsym(fHandle,"nn_controller");

    if (!func) {
        warn("cannot load nn_controller");
        exit(1);
    } else {
    	warn("nn param changed");
    	mavlink_log_info(&_mavlink_log_pub, "[mc_rl_control] nn param changed");
    }
}

inline void
MulticopterRLControl::calc_rl_pwm_output()
{
	double battery_voltage_pwm_calc = battery_voltage;
	if (battery_voltage_pwm_calc < 1.0)
		battery_voltage_pwm_calc = 11.1;

	for (int i = 0; i < 4; ++i)
	{
		pwm_calc_tmp[i] = ((double)nn_output(i)*4 + MASS/4*9.8);

		if (pwm_calc_tmp[i] < 0)
			pwm_calc_tmp[i] = 0;

		pwm_output[i] = (uint16_t)((-4e-4 + sqrt(16e-8+(4*3e-7*pwm_calc_tmp[i])/battery_voltage_pwm_calc))/(2*3e-7)) + 1000;

		if (pwm_output[i] < 900)
			pwm_output[i] = 900;
		else if (pwm_output[i] > 2100)
			pwm_output[i] = 2100;
	}
}

inline void
MulticopterRLControl::calc_preheat_pwm_output()
{
	for (int i = 0; i < 4; ++i)
	{
		pwm_output[i] = 900;
	}
}

void
MulticopterRLControl::subtask_list()
{
	subtask_counter[0]++;
	if (subtask_counter[0] > 10)	
	{
		arm_publish();
		vehicle_local_position_setpoint_publish();
		actuator_outputs_publish();
		led_control_publish();
		subtask_counter[0] = 0;
	}

	subtask_counter[1]++;
	if (subtask_counter[1] > 1e6/SUBTASK2_RATE)	
	{
		load_external_variable();
		subtask_counter[1] = 0;
	}

	// subtask_counter[2]++;
	// if (subtask_counter[2] > 1e5)	// 100s
	// {
	// 	// load_external_variable();
	// 	subtask_counter[2] = 0;
	// }
}

void
MulticopterRLControl::task_main()
{
	/* init pwm output*/
	pwm_device_init();

	_input_rc_sub = orb_subscribe(ORB_ID(input_rc));
	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	_vehicle_status_pub = orb_advertise(ORB_ID(vehicle_status), &_vehicle_status);
	_position_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_vehicle_local_position_setpoint);
	_actuator_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &_actuator_outputs);

	orb_set_interval(_input_rc_sub, 100);			//set update rate 10Hz
	// orb_set_interval(_vehicle_attitude_sub, 5);	// Pull loop uprate to maximum speed which is equal to uorb publish speed
	orb_set_interval(_vehicle_local_position_sub, 10);
	orb_set_interval(_battery_status_sub, 200);

	load_external_variable();

	/* wakeup source: gyro data from sensor selected by the sensor app */
	px4_pollfd_struct_t poll_fds = {.fd = _vehicle_attitude_sub,	.events = POLLIN};

	while(!_task_should_exit){

		/* wait for up to 10ms for data */
		int pret = px4_poll(&poll_fds, 1, 10);

		if (pret < 0) {
			/* this is seriously bad - should be an emergency */
			// PX4_ERR("ERROR return value from poll(): %d", pret);
		} else {

			input_rc_poll();
			vehicle_attitude_poll();
			vehicle_local_position_poll();
			battery_status_poll();

			_position_err = _position - _position_err;
			prepare_nn_states();
			log_socket.store(input_states, nn_output);
			nn_output = func(input_states);
			if (armed == vehicle_status_s::ARMING_STATE_INIT)
			{
				calc_preheat_pwm_output();
			}
			else if (armed == vehicle_status_s::ARMING_STATE_STANDBY)
			{
				calc_preheat_pwm_output();
			}
			else if (armed == vehicle_status_s::ARMING_STATE_ARMED)
			{
				calc_rl_pwm_output();
			}

			pwm_device_output(pwm_output);
			
			subtask_list();
			// arm_publish();
			// vehicle_local_position_setpoint_publish();
			// actuator_outputs_publish();
			
		}
	}
}

int
MulticopterRLControl::task_main_trampoline(int argc, char *argv[])
{
	rl_control::g_rl_control->task_main();
	return 0;
}

static void usage()
{
	errx(1, "usage: rl_control {start|stop|status}");
}

int mc_rl_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (rl_control::g_rl_control != nullptr) {
			errx(1, "already running");
		}

		rl_control::g_rl_control = new MulticopterRLControl;

		if (rl_control::g_rl_control == nullptr) {
			errx(1, "alloc failed");
		}

		if (OK != rl_control::g_rl_control->start()) {
			delete rl_control::g_rl_control;
			rl_control::g_rl_control = nullptr;
			err(1, "start failed");
		}

		return 0;
	}

	if (rl_control::g_rl_control == nullptr) {
		errx(1, "not running");
	}

	if (!strcmp(argv[1], "stop")) {
		delete rl_control::g_rl_control;
		rl_control::g_rl_control = nullptr;

	} else if (!strcmp(argv[1], "status")) {
		rl_control::g_rl_control->status();

	// } else if (!strcmp(argv[1], "drop")) {
	// 	rl_control::g_rl_control->drop();

	// } else if (!strcmp(argv[1], "open")) {
	// 	rl_control::g_rl_control->open_bay();

	// } else if (!strcmp(argv[1], "close")) {
	// 	rl_control::g_rl_control->close_bay();

	// } else if (!strcmp(argv[1], "lock")) {
	// 	rl_control::g_rl_control->lock_release();

	} else {
		usage();
	}

	return 0;
}
