#include "hierarchical_controller.hpp"


HierarchicalController* HierarchicalController::unique_handle = nullptr;


HierarchicalController::HierarchicalController(Cerebellum<18,3>& ce, Blabbermouth& di, uORBInterface& uo)
:_policy(ce), 
 _log(di), 
 _uORB(uo) {_task_should_stop = true;}


void HierarchicalController::main_loop() {


	/* wakeup source: gyro data from sensor selected by the sensor app */
	px4_pollfd_struct_t poll_fds = {
		.fd     = _uORB._vehicle_attitude_sub,
                .events = POLLIN
        };

        std::array<float,4> low_level_control;

        _task_should_stop = false;

	while(!_task_should_stop) {
		/* wait for up to 1ms for data */
		int pret = px4_poll(&poll_fds, 1, 1);

		if(pret<0) {
			PX4_ERR("ERROR return value from poll(): %d", pret);
		} else {
			update();

			low_level_control=
				_policy.respond(_rotation_matrix, _set_point_error,
					        _angular_rate, _velocity);
			
			_log.take_note(_uORB._now,
			                _quaternion, 
			                _set_point_error,
			                _angular_rate,
			                _velocity,
			                _policy._delta_p,
			                low_level_control,
			                _position,
			                _position_sp,
			                _voltage);

		}
	}
}


void HierarchicalController::update() {
	_uORB.sync();

	bool att_update = _uORB.vehicle_attitude_poll();
	if(att_update) {
		update_quaternion();
		update_angular_rate();

		update_rotation_matrix();
	}

	bool loc_update = _uORB.vehicle_local_position_poll();
	if(loc_update) {
		update_position();
		update_velocity();
	}

	bool rc_update = _uORB.input_rc_poll();
	if(rc_update) {
		update_position_sp();
	}

	bool bat_update = _uORB.battery_status_poll();
	if(bat_update) {
		update_voltage();
	}

	if (loc_update or loc_update){
		update_set_point_error();
	}
}



void HierarchicalController::update_quaternion() {
	_quaternion[0] = _uORB._vehicle_attitude.q[0];
	_quaternion[1] = _uORB._vehicle_attitude.q[1];
	_quaternion[2] = _uORB._vehicle_attitude.q[2];
	_quaternion[3] = _uORB._vehicle_attitude.q[3];
}


void HierarchicalController::update_angular_rate() {
	_angular_rate[0] = _uORB._vehicle_attitude.rollspeed;
	_angular_rate[1] = _uORB._vehicle_attitude.pitchspeed;
	_angular_rate[2] = _uORB._vehicle_attitude.yawspeed;
}


void HierarchicalController::update_position() {
	_position[0] = _uORB._vehicle_local_position.x;
	_position[1] = _uORB._vehicle_local_position.y;
	_position[2] = _uORB._vehicle_local_position.z;
}


void HierarchicalController::update_velocity() {
	_velocity[0] = _uORB._vehicle_local_position.vx;
	_velocity[1] = _uORB._vehicle_local_position.vy;
	_velocity[2] = _uORB._vehicle_local_position.vz;
}


void HierarchicalController::update_position_sp() {
	std::array<float,3> tmp;
	tmp[0] = _uORB._input_rc.values[1];
	tmp[1] = _uORB._input_rc.values[3];
	tmp[2] = _uORB._input_rc.values[2];

	for(auto& v: tmp) {
		if(v>1570) {
			v-=1570;
		} else if (v >= 1430) {
			v = 0;
		} else {
			v -= 1430;
		}
		v/=10000;
	}

	_position_sp[0] += tmp[0];		
	_position_sp[1] -= tmp[1];		
	_position_sp[2] += tmp[2];		
}


void HierarchicalController::update_set_point_error() {
	for(int i=0; i<3; ++i)
		_set_point_error[i] = _position[i] - _position_sp[i];
}


void HierarchicalController::update_rotation_matrix() {
	float a = _quaternion[0];
	float b = _quaternion[1];
	float c = _quaternion[2];
	float d = _quaternion[3];
	float aSq = a * a;
	float bSq = b * b;
	float cSq = c * c;
	float dSq = d * d;
	_rotation_matrix[0*3+0] = aSq + bSq - cSq - dSq;
	_rotation_matrix[1*3+0] = 2 * (b * c - a * d);
	_rotation_matrix[2*3+0] = 2 * (a * c + b * d);
	_rotation_matrix[0*3+1] = 2 * (b * c + a * d);
	_rotation_matrix[1*3+1] = aSq - bSq + cSq - dSq;
	_rotation_matrix[2*3+1] = 2 * (c * d - a * b);
	_rotation_matrix[0*3+2] = 2 * (b * d - a * c);
	_rotation_matrix[1*3+2] = 2 * (a * b + c * d);
	_rotation_matrix[2*3+2] = aSq - bSq - cSq + dSq;
}


void HierarchicalController::update_voltage() {
	_voltage[0] = _uORB._battery_status.voltage_filtered_v;
}