#pragma once

#include <px4_log.h>
#include <drivers/drv_hrt.h>

//pwm
#include <drivers/linux_pwm_out/navio_sysfs.h>
#include <perf/perf_counter.h>

//uORB
#include <uORB/uORB.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/led_control.h>


#include <stdexcept>


#define MAX_NUM_OUTPUTS 8





class uORBInterface{
public:

	uORBInterface() = delete;
	uORBInterface(const char* pwm_device, int max_num_outputs=MAX_NUM_OUTPUTS);
	~uORBInterface();

	/**
	 * Update input rc data.
	 */
	bool
		input_rc_poll();

	/**
	 * Update vehicle attitude data.
	 */
	bool
		vehicle_attitude_poll();

	/**
	 * Update local position data.
	 */
	bool
		vehicle_local_position_poll();

	/**
	 * Update battery status data.
	 */
	bool
		battery_status_poll();


	/**
	 * Led info
	 */
	void
		led_control_publish();


	inline void
		pool_all() {
			input_rc_poll();
			vehicle_attitude_poll();
			vehicle_local_position_poll();
			battery_status_poll();
			sync();
		}


	/**
	 * Send pwm output.
	 */
	// void pwm_device_output(const uint16_t *);


	inline void 
		sync(){_now = hrt_absolute_time();}


	enum class VEHICLE_STATE{
	ARMED, STANDBY, INIT, LOW_BATTERY
        } _state;

	
	hrt_abstime _now;

	input_rc_s               _input_rc;
	vehicle_attitude_s       _vehicle_attitude;
	vehicle_local_position_s _vehicle_local_position;
	battery_status_s         _battery_status;
	led_control_s            _led_control;

	int _input_rc_sub;
	int _vehicle_attitude_sub;
	int _vehicle_local_position_sub;
	int _battery_status_sub;

private:
	

	orb_advert_t    _led_control_pub;

	linux_pwm_out::NavioSysfsPWMOut* _pwm_out_handle;


	void uORBTopicInit();
	void pwm_init(const char* pwm_device, int max_num_outputs);
	
};

#undef MAX_NUM_OUTPUTS

