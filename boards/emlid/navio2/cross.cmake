
px4_add_board(
	VENDOR emlid
	MODEL navio2
	LABEL cross
	PLATFORM posix
	ARCHITECTURE cortex-a53
	TOOLCHAIN arm-linux-gnueabihf
	TESTING

	DRIVERS
		#barometer # all available barometer drivers
		batt_smbus
		camera_trigger
		differential_pressure # all available differential pressure drivers
		distance_sensor # all available distance sensor drivers
		gps
		#imu # all available imu drivers
		#magnetometer # all available magnetometer drivers
		pwm_out_sim
		#telemetry # all available telemetry drivers

		linux_pwm_out
		linux_sbus

	DF_DRIVERS # NOTE: DriverFramework is migrating to intree PX4 drivers
		hmc5883
		isl29501
		lsm9ds1
		mpu9250
		ms5611
		trone

	MODULES
		attitude_estimator_q
		camera_feedback
		commander
		dataman
		ekf2
		events
		fw_att_control
		fw_pos_control_l1
		gnd_att_control
		gnd_pos_control
		land_detector
		landing_target_estimator
		load_mon
		local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_pos_control
		navigator
		sensors
		sih
		#simulator
		vmount
		vtol_att_control
		wind_estimator
		mc_rl_control

	SYSTEMCMDS
		dyn
		esc_calib
		led_control
		mixer
		motor_ramp
		param
		perf
		pwm
		reboot
		sd_bench
		shutdown
		tests # tests and test runner
		top
		topic_listener
		tune_control
		ver

	EXAMPLES
		
	)
