/*************************************************************
Code de contrôle base à 4 roues motrices
Benjamin De Coninck
*************************************************************/

#include "WheelBase.h"

void WheelBase::set_break()
{
	_AvG.set_break();
	_AvD.set_break();
	_ArG.set_break();
	_ArD.set_break();
}

void WheelBase::test_motors(int target_speed)
{
	_AvG.set_speed(target_speed);
	_AvD.set_speed(target_speed);
	_ArG.set_speed(target_speed);
	_ArD.set_speed(target_speed);

}

void WheelBase::get_speed()
{
	//linear x speed is mean of all motor linear speed
	_odom.x_speed = (rpm_to_ms(_AvG.get_speed()) + \
			   rpm_to_ms(_AvD.get_speed()) + \
			   rpm_to_ms(_ArG.get_speed()) + \
			   rpm_to_ms(_ArD.get_speed()))/4;
	
	//angular speed is difference between left and right speed
	_odom.ang_speed = (((rpm_to_ms(_AvG.get_speed()) + rpm_to_ms(_ArG.get_speed()))/2 - \
			  ((rpm_to_ms(_AvD.get_speed())  + rpm_to_ms(_ArD.get_speed()))/2))/2) 
					/ (float)(WHEEL_DISTANCE * PI);	

}

nav_msgs::Odometry WheelBase::update_position()
{

	//TODO
	get_speed();

	int current_time = micros();
	float delta_time = (current_time - _last_time)/1000000; // time delta in seconds
	_last_time = current_time;

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
	// yspeed is disregarded for the differential transmission motor
	float lin_xspeed = ms_to_rpm(xspeed * XSPEED_MAX / JOY_MAX);
    float lin_wspeed = ms_to_rpm((WHEEL_DISTANCE * (wspeed * WSPEED_MAX / JOY_MAX) * PI)/360); 
	
	_AvG.set_speed(lin_xspeed + lin_wspeed);
	_AvD.set_speed(lin_xspeed + lin_wspeed);
	_ArG.set_speed(lin_xspeed - lin_wspeed);
	_ArD.set_speed(lin_xspeed - lin_wspeed);
}


