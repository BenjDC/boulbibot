/*************************************************************
Code de contrôle base à 4 roues motrices
Benjamin De Coninck
*************************************************************/

#include "WheelBase.h"


void WheelBase::set_torque(int torque)
{
	uint8_t byte_data = (uint8_t)torque;
	// enable torque
	byte_data = 1;
	_test_motor.writeByteCommand(_avG_id, REG_TORQUE_ENABLE, 1, &byte_data);
	_test_motor.writeByteCommand(_avD_id, REG_TORQUE_ENABLE, 1, &byte_data);
	_test_motor.writeByteCommand(_arD_id, REG_TORQUE_ENABLE, 1, &byte_data);
	_test_motor.writeByteCommand(_arG_id, REG_TORQUE_ENABLE, 1, &byte_data);
	
}

void WheelBase::ping_motors()
{
	uint16_t model_number;
	uint8_t firmware_version;
	int error_code;

	error_code = _test_motor.pingCommand(_avG_id, &model_number, &firmware_version);
	if (error_code == 0) ROS_INFO("Motor av gauche OK !"); else ROS_INFO("Motor av gauche NOK");
	// error_code = _test_motor.pingCommand(_avD_id, &model_number, &firmware_version);
	// if (error_code == 0) ROS_INFO("Motor av droit OK !"); else ROS_INFO("Motor av droit NOK");
	// error_code = _test_motor.pingCommand(_arG_id, &model_number, &firmware_version);
	// if (error_code == 0) ROS_INFO("Motor ar gauche OK !"); else ROS_INFO("Motor ar gauche NOK");
	// error_code = _test_motor.pingCommand(_arD_id, &model_number, &firmware_version);
	// if (error_code == 0) ROS_INFO("Motor ar droit OK !"); else ROS_INFO("Motor ar droit NOK");
    
}

void WheelBase::init_control_mode(int control_mode)
{
	uint8_t byte_data = (uint8_t)control_mode;
	// set control mode
	_test_motor.writeByteCommand(_avG_id, REG_CONTROL_MODE, 1, &byte_data);
	// _test_motor.writeByteCommand(_avD_id, REG_CONTROL_MODE, 1, &byte_data);
	// _test_motor.writeByteCommand(_arD_id, REG_CONTROL_MODE, 1, &byte_data);
	// _test_motor.writeByteCommand(_arG_id, REG_CONTROL_MODE, 1, &byte_data);
}

void WheelBase::test_speed(uint16_t target_speed)
{
	//set all motors to the same speed target
	uint16_t word_data = (uint16_t)target_speed;
	
    //_test_motor.writeWordCommand(_avD_id, REG_GOAL_VELOCITY_DPS_L, 1, &word_data);
	_test_motor.writeWordCommand(_avG_id, REG_GOAL_VELOCITY_DPS_L, 1, &word_data);
	//_test_motor.writeWordCommand(_arD_id, REG_GOAL_VELOCITY_DPS_L, 1, &word_data);
	//_test_motor.writeWordCommand(_arG_id, REG_GOAL_VELOCITY_DPS_L, 1, &word_data);
}

void WheelBase::test_pwm(int target_pwm)
{
	//set all motors to the same PWM target
	uint16_t word_data = (uint16_t)target_pwm;
	
    _test_motor.writeWordCommand(_avG_id, REG_GOAL_PWM_100_L, 1, &word_data);
	//_test_motor.writeWordCommand(_avD_id, REG_GOAL_PWM_100_L, 1, &word_data);
	//_test_motor.writeWordCommand(_arD_id, REG_GOAL_PWM_100_L, 1, &word_data);
	//_test_motor.writeWordCommand(_arG_id, REG_GOAL_PWM_100_L, 1, &word_data);
}

void WheelBase::kill()
{
	//TODO
	ROS_INFO("killin' boulbi");
}

void WheelBase::get_speed(uint16_t *test_speed)
{
	uint16_t word_data;

	_test_motor.readWordCommand(_avG_id, REG_PRESENT_VELOCITY_DPS_L, 1, &word_data);
	_avG_speed = word_data;
	
	// _test_motor.readWordCommand(_avD_id, REG_PRESENT_VELOCITY_DPS_L, 1, &word_data);
	// _avD_speed = word_data;

	// _test_motor.readWordCommand(_arG_id, REG_PRESENT_VELOCITY_DPS_L, 1, &word_data);
	// _arG_speed = word_data;

	// _test_motor.readWordCommand(_arD_id, REG_PRESENT_VELOCITY_DPS_L, 1, &word_data);
	// _arD_speed = word_data;

	*test_speed = _avG_speed;


	
	//TODO
	//linear x speed is mean of all motor linear speed
	//_odom.x_speed = (rpm_to_ms(_AvG.get_speed()) + \
			   rpm_to_ms(_AvD.get_speed()) + \
			   rpm_to_ms(_ArG.get_speed()) + \
			   rpm_to_ms(_ArD.get_speed()))/4;

	//y speed : todo
	
	//angular speed is difference between left and right speed
	//_odom.ang_speed = (((rpm_to_ms(_AvG.get_speed()) + rpm_to_ms(_ArG.get_speed()))/2 - \
			//   ((rpm_to_ms(_AvD.get_speed())  + rpm_to_ms(_ArD.get_speed()))/2))/2) 
			// 		/ (float)(WHEEL_DISTANCE * PI);	

}

nav_msgs::Odometry WheelBase::update_position()
{

	//TODO
	//get_speed();

	// int current_time = micros();
	// float delta_time = (current_time - _last_time)/1000000; // time delta in seconds
	// _last_time = current_time;

	// _c_odom.ang_pos += _c_odom.ang_speed * delta_time;	
	// _c_odom.x_pos += (_c_odom.x_speed * cos((_c_odom.ang_pos /180) * PI) + \
	// 			      _c_odom.y_speed * sin((_c_odom.ang_pos /180) * PI)) * delta_time;
	// _c_odom.y_pos += (_c_odom.x_speed * sin((_c_odom.ang_pos /180) * PI) + \
	// 			      _c_odom.y_speed * cos((_c_odom.ang_pos /180) * PI)) * delta_time;

	return _odom;
}


void OmniWheel::set_motors(float xspeed, float yspeed, float wspeed)
{
	//TODO
}

void DiffWheel::set_motors(float xspeed, float yspeed, float wspeed)
{
	//TODO 
	
	// yspeed is disregarded for the differential transmission motor
	// float lin_xspeed = ms_to_rpm(xspeed * XSPEED_MAX / JOY_MAX);
    // float lin_wspeed = ms_to_rpm((WHEEL_DISTANCE * (wspeed * WSPEED_MAX / JOY_MAX) * PI)/360); 
	
	//TODO
	// _AvG.set_speed(lin_xspeed + lin_wspeed);
	// _AvD.set_speed(lin_xspeed + lin_wspeed);
	// _ArG.set_speed(lin_xspeed - lin_wspeed);
	// _ArD.set_speed(lin_xspeed - lin_wspeed);
}


