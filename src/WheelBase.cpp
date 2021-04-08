/*************************************************************
Code de contrôle base à 4 roues motrices
Benjamin De Coninck
*************************************************************/

#include "WheelBase.h"
#include <ros/ros.h>


void WheelBase::set_torque(int torque)
{
	uint8_t byte_data = (uint8_t)torque;
	// enable torque
	byte_data = 1;

	for (int motor_id = 0; motor_id < MOTOR_NUMBER; motor_id++)
	{
		_test_motor.writeByteCommand(motor_id, REG_TORQUE_ENABLE, 1, &byte_data);
	}
	
}

void WheelBase::ping_motors()
{
	uint16_t model_number;
	uint8_t firmware_version;
	int error_code;

	for (int motor_id = 0; motor_id < MOTOR_NUMBER; motor_id++)
	{
		error_code = _test_motor.pingCommand(motor_id, &model_number, &firmware_version);
		if (error_code == 0) ROS_INFO("Motor %i OK !", motor_id); else ROS_INFO("Motor %i NOK", motor_id);
		_test_motor.enableLed(motor_id);
		ros::Duration(0.5).sleep();
	}  
}

void WheelBase::init_control_mode(int control_mode)
{
	uint8_t byte_data = (uint8_t)control_mode;
	// set control mode

	for (int motor_id = 0; motor_id < MOTOR_NUMBER; motor_id++)
	{
		_test_motor.writeByteCommand(motor_id, REG_CONTROL_MODE, 1, &byte_data);
		_test_motor.disableLed(motor_id);
	}
}

void WheelBase::set_motor_speed(uint16_t *motor_speed)
{
	//set all motors to the same speed target
	
	for (int motor_id = 0; motor_id < MOTOR_NUMBER; motor_id++)
	{
		_test_motor.writeWordCommand(motor_id, REG_GOAL_VELOCITY_DPS_L, 1, &motor_speed[motor_id]);
	}	
}

void WheelBase::test_pwm(int target_pwm)
{
	//set all motors to the same PWM target
	uint16_t word_data = (uint16_t)target_pwm;

	for (int motor_id = 0; motor_id < MOTOR_NUMBER; motor_id++)
	{
		_test_motor.writeWordCommand(motor_id, REG_GOAL_PWM_100_L, 1, &word_data);
	}
}

void WheelBase::kill()
{
	//TODO
	ROS_INFO("killin' boulbi");
}

void WheelBase::get_speed()
{
	int error;

	for (int motor_id = 0; motor_id < MOTOR_NUMBER; motor_id++)
	{
		_motor_speed[motor_id]= _test_motor.getVelocity(motor_id, &error);
	}
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


