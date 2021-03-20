#ifndef __WheelBase_H__
#define __WheelBase_H__

#include "Motor.h"
#include "sm_protocol.h"
#include "math.h"
#include <nav_msgs/Odometry.h>


class WheelBase
{
	public:
		WheelBase(int avG_id, int avD_id, int arG_id, int arD_id):
			_avG_id(avG_id),
			_avD_id(avD_id),
			_arG_id(arG_id),
			_arD_id(arD_id),
			_last_time(0),
			_test_motor(sm_protocol(500000)){}
		
		void test_pwm(int test_pwm);
		void test_speed(int test_speed);
		void set_break();
		void get_speed();
		void init_pwm();
		void kill();
		nav_msgs::Odometry update_position();
		virtual void set_motors(float xspeed, float yspeed, float wspeed) = 0;
		
	protected:
		virtual float _get_wheel_diameter() = 0;
		float rpm_to_ms(int rpm_speed){return (rpm_speed * M_PI * _get_wheel_diameter() * 60 / 1000);}
		int ms_to_rpm(float ms_speed){return (int)((ms_speed * 1000) / (M_PI * _get_wheel_diameter() * 60 ));}
		
		
	private:
		int _avG_id;
		int _avD_id;
		int _arG_id;
		int _arD_id;
		int _last_time;		// µs
		sm_protocol _test_motor;
		//custom compact odometry structure
		nav_msgs::Odometry _odom;
};

class OmniWheel : public WheelBase
{
	public:
		OmniWheel(int avG_id, int avD_id, int arG_id, int arD_id):
		WheelBase(avG_id, avD_id, arG_id, arD_id){}

		void set_motors(float xspeed, float yspeed, float wspeed);
	private:
		float _get_wheel_diameter(){return 60.0;}		
};

class DiffWheel : public WheelBase
{
	public:
		DiffWheel(int avG_id, int avD_id, int arG_id, int arD_id):
		WheelBase(avG_id, avD_id, arG_id, arD_id){}
		void set_motors(float xspeed, float yspeed, float wspeed);
	private:
		float _get_wheel_diameter(){return 80.0;}		
};

#define WHEEL_DISTANCE	0.20

#define XSPEED_MAX 		0.80 	// m.s
#define XSACC_MAX 		0.10 	// m.s2
#define WSPEED_MAX  	40.0	// d.s
#define WACC_MAX  		30.0	// d.s2
#define JOY_MAX  		0.5 	// no unit


#endif