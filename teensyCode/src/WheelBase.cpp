/*************************************************************
Code de contrôle base à 4 roues motrices
Benjamin De Coninck
*************************************************************/

#include "Arduino.h"
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
	_xspeed_rpm = (_AvG.set_speed(target_speed) + _AvD.set_speed(target_speed) + _ArG.set_speed(target_speed) + _ArD.set_speed(target_speed))/4 * PI * WHEEL_DIAMETER / 60;
	//_wspeed_rpm = (_AvG.set_speed(target_speed) + _ArG.set_speed(target_speed)) / 2 - ;
	_yspeed = (_AvG.set_speed(target_speed) - _AvD.set_speed(target_speed) + _ArG.set_speed(target_speed) - _ArD.set_speed(target_speed))/4 * PI * WHEEL_DIAMETER / 60;
}

void OmniWheel::set_motors(float xspeed, float yspeed, float wspeed)
{
	float lin_speed_scaled = xspeed * XSPEED_MAX / JOY_MAX;
    float ang_speed_scaled = wspeed * WSPEED_MAX / JOY_MAX;
}

void DiffWheel::set_motors(float xspeed, float yspeed, float wspeed)
{
	float target_lin_xspeed_scaled = xspeed * XSPEED_MAX / JOY_MAX;
    float target_wspeed_scaled = wspeed * WSPEED_MAX / JOY_MAX;

    int target_x_rpm = (int)(target_lin_xspeed_scaled / (PI * WHEEL_DIAMETER));

    //float target_lin_wspeed = target_wspeed_scaled * WHEEL_DISTANCE
    	

    int target_left_rpm = target_lin_xspeed_scaled / (PI * WHEEL_DIAMETER);
	
}


