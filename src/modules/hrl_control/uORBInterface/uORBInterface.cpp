#include "uORBInterface.hpp"

#include <cstring>


uORBInterface::~uORBInterface() {
	if(_pwm_out_handle)
		delete _pwm_out_handle;
}

uORBInterface::uORBInterface(const char* pwm_device, int max_num_outputs)
:_now(hrt_absolute_time()),
 _input_rc(),
 _vehicle_attitude(),
 _vehicle_local_position(),
 _battery_status(),
 _led_control(),
 _input_rc_sub(-1),
 _vehicle_attitude_sub(-1),
 _vehicle_local_position_sub(-1),
 _battery_status_sub(-1),
 _pwm_out_handle(nullptr) {
	uORBTopicInit();
	pwm_init(pwm_device, max_num_outputs);
	

}


bool uORBInterface::input_rc_poll() {
	
	static bool input_rc_updated = false;
	orb_check(_input_rc_sub, &input_rc_updated);
	
	
	if (input_rc_updated) {
		
		orb_copy(ORB_ID(input_rc), _input_rc_sub, &_input_rc);
		const int& vehicle_mode = _input_rc.values[6];

		if(vehicle_mode > 1700) {
			if(_state != VEHICLE_STATE::ARMED) {
				_state          = VEHICLE_STATE::ARMED;
			}
		}

		else if(vehicle_mode > 1300) {
			if(_state != VEHICLE_STATE::STANDBY) {
				_state          = VEHICLE_STATE::STANDBY;
			}
		}
			
		else {
			if(_state != VEHICLE_STATE::INIT) {
				_state          = VEHICLE_STATE::INIT;
			}
		}
	}

	return input_rc_updated;
}


bool uORBInterface::vehicle_attitude_poll() {
	static bool vehicle_attitude_updated = false;
	orb_check(_vehicle_attitude_sub, &vehicle_attitude_updated);


	if (vehicle_attitude_updated) {
		orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_vehicle_attitude);
	}

	return vehicle_attitude_updated;
}


bool uORBInterface::vehicle_local_position_poll() {
	static bool vehicle_local_position_updated = false;
	orb_check(_vehicle_local_position_sub, &vehicle_local_position_updated);


	if (vehicle_local_position_updated) {
		orb_copy(ORB_ID(vehicle_local_position), _vehicle_local_position_sub, &_vehicle_local_position);
	}

	return vehicle_local_position_updated;
}


bool uORBInterface::battery_status_poll() {
	static bool battery_status_updated = false;
	orb_check(_battery_status_sub, &battery_status_updated);


	if (battery_status_updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
		if(_battery_status.voltage_filtered_v < 10.5f)
			_state = VEHICLE_STATE::LOW_BATTERY;
	}

	return battery_status_updated;
}


void uORBInterface::led_control_publish() {
	

	if(_led_control_pub==nullptr) {
		_led_control_pub = orb_advertise(ORB_ID(led_control), &_led_control);
		return;
	}

	switch(_state) {
		case VEHICLE_STATE::ARMED:
			_led_control.color = led_control_s::COLOR_PURPLE;
			_led_control.mode  = led_control_s::MODE_ON;   
			break;

		case VEHICLE_STATE::STANDBY:
			_led_control.color = led_control_s::COLOR_RED;
			_led_control.mode  = led_control_s::MODE_BLINK_FAST;
			break;

		case VEHICLE_STATE::INIT:
			_led_control.color = led_control_s::COLOR_GREEN;
			_led_control.mode  = led_control_s::MODE_BLINK_NORMAL;
			break;

		case VEHICLE_STATE::LOW_BATTERY:
			_led_control.color = led_control_s::COLOR_PURPLE;
			_led_control.mode  = led_control_s::MODE_BLINK_FAST;
			break;
	}
	
	_led_control.timestamp = _now;
	_led_control.led_mask  = 0xff;
	

	orb_publish(ORB_ID(led_control), _led_control_pub, &_led_control);
	return;
}




void uORBInterface::uORBTopicInit() {
	_input_rc_sub = orb_subscribe(ORB_ID(input_rc));
	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	

	PX4_INFO("rc_fd: %d", _input_rc_sub);
	PX4_INFO("vehicle_attitude_fd: %d", _vehicle_attitude_sub);
	PX4_INFO("vehicle_local_position_fd: %d", _vehicle_local_position_sub);
	PX4_INFO("battery_status_fd: %d", _battery_status_sub);

	if(_input_rc_sub<0)
		throw std::runtime_error("subscribe input_rc failed");
	if(_vehicle_attitude_sub<0)
		throw std::runtime_error("subscribe vehicle_attitude failed");
	if(_vehicle_local_position_sub<0)
		throw std::runtime_error("subscribe vehicle_local_position failed");
	if(_battery_status_sub<0)
		throw std::runtime_error("subscribe battery_status failed");

	return;
}


void uORBInterface::pwm_init(const char* pwm_device, int max_num_outputs) {
	PX4_INFO("Starting PWM output in Navio mode");
	_pwm_out_handle = new linux_pwm_out::NavioSysfsPWMOut(pwm_device, max_num_outputs);
	if (_pwm_out_handle->init() != 0) {
		delete _pwm_out_handle;
		_pwm_out_handle = nullptr;
		throw std::runtime_error("PWM output init failed");
	}
}