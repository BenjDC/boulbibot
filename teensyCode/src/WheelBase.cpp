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


void OmniWheel::set_motors(float xspeed, float yspeed, float wspeed)
{
	float lin_speed_scaled = xspeed * XSPEED_MAX / JOY_MAX;
    float ang_speed_scaled = wspeed * WSPEED_MAX / JOY_MAX;

	
}

void DiffWheel::set_motors(float xspeed, float yspeed, float wspeed)
{
	float lin_speed_scaled = xspeed * XSPEED_MAX / JOY_MAX;
    float ang_speed_scaled = wspeed * WSPEED_MAX / JOY_MAX;
	
}
