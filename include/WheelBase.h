#ifndef __WheelBase_H__
#define __WheelBase_H__

#include "Motor.h"
#include "math.h"
#include <nav_msgs/Odometry.h>


class WheelBase
{
	public:
		WheelBase(int M1_IN1, int M1_IN2, int M1_PWM, int M1_E1, int M1_E2,
				  int M2_IN1, int M2_IN2, int M2_PWM, int M2_E1, int M2_E2,
				  int M3_IN1, int M3_IN2, int M3_PWM, int M3_E1, int M3_E2,
				  int M4_IN1, int M4_IN2, int M4_PWM, int M4_E1, int M4_E2):
		
		_pi_id(pigpio_start("rospi.local", NULL)),
		_AvG(Motor(M1_IN1, M1_IN2, M1_PWM, M1_E1, M1_E2, _pi_id)),
		_AvD(Motor(M2_IN1, M2_IN2, M2_PWM, M2_E1, M2_E2, _pi_id)),
		_ArG(Motor(M3_IN1, M3_IN2, M3_PWM, M3_E1, M3_E2, _pi_id)),
		_ArD(Motor(M4_IN1, M4_IN2, M4_PWM, M4_E1, M4_E2, _pi_id)){}
		

		void test_motors(int target_speed);
		void set_break();
		void get_speed();
		nav_msgs::Odometry update_position();
		virtual void set_motors(float xspeed, float yspeed, float wspeed) = 0;
		
	protected:
		virtual float _get_wheel_diameter() = 0;
		float rpm_to_ms(int rpm_speed){return (rpm_speed * M_PI * _get_wheel_diameter() * 60 / 1000);}
		int ms_to_rpm(float ms_speed){return (int)((ms_speed * 1000) / (M_PI * _get_wheel_diameter() * 60 ));}
		
		Motor _AvG;
		Motor _AvD;
		Motor _ArG;
		Motor _ArD;		

	private:
		int _last_time;		// µs
		int _pi_id;
		//custom compact odometry structure
		nav_msgs::Odometry _c_odom;
};

class OmniWheel : public WheelBase
{
	public:
		OmniWheel(int M1_IN1, int M1_IN2, int M1_PWM, int M1_E1, int M1_E2,
				  int M2_IN1, int M2_IN2, int M2_PWM, int M2_E1, int M2_E2,
				  int M3_IN1, int M3_IN2, int M3_PWM, int M3_E1, int M3_E2,
				  int M4_IN1, int M4_IN2, int M4_PWM, int M4_E1, int M4_E2):
		WheelBase(M1_IN1, M1_IN2, M1_PWM, M1_E1, M1_E2,
				  M2_IN1, M2_IN2, M2_PWM, M2_E1, M2_E2,
				  M3_IN1, M3_IN2, M3_PWM, M3_E1, M3_E2,
				  M4_IN1, M4_IN2, M4_PWM, M4_E1, M4_E2){}
		void set_motors(float xspeed, float yspeed, float wspeed);
	private:
		float _get_wheel_diameter(){return 60.0;}		
};

class DiffWheel : public WheelBase
{
	public:
		DiffWheel(int M1_IN1, int M1_IN2, int M1_PWM, int M1_E1, int M1_E2,
				  int M2_IN1, int M2_IN2, int M2_PWM, int M2_E1, int M2_E2,
				  int M3_IN1, int M3_IN2, int M3_PWM, int M3_E1, int M3_E2,
				  int M4_IN1, int M4_IN2, int M4_PWM, int M4_E1, int M4_E2):
		WheelBase(M1_IN1, M1_IN2, M1_PWM, M1_E1, M1_E2,
				  M2_IN1, M2_IN2, M2_PWM, M2_E1, M2_E2,
				  M3_IN1, M3_IN2, M3_PWM, M3_E1, M3_E2,
				  M4_IN1, M4_IN2, M4_PWM, M4_E1, M4_E2){}
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