#ifndef __WheelBase_H__
#define __WheelBase_H__

#include "Encoder.h"
#include "Motor.h"
#include <tetra_ros/compactOdom.h>


class WheelBase
{
 
	public:
		WheelBase(Motor m_AvG, Motor m_AvD, Motor m_ArD, Motor m_ArG):
		_AvG(m_AvG),
		_AvD(m_AvD),
		_ArG(m_ArG),
		_ArD(m_ArD),
		_last_time(micros()){}

		void test_motors(int target_speed);
		void set_break();
		void get_speed();
		tetra_ros::compactOdom update_position();
		virtual void set_motors(float xspeed, float yspeed, float wspeed) = 0;
		
	protected:
		virtual float _get_wheel_diameter() = 0;
		float rpm_to_ms(int rpm_speed){return (rpm_speed * PI * _get_wheel_diameter() * 60 / 1000);}
		int ms_to_rpm(float ms_speed){return (int)((ms_speed * 1000) / (PI * _get_wheel_diameter() * 60 ));}
		
		Motor _AvG;
		Motor _AvD;
		Motor _ArG;
		Motor _ArD;	

	private:
		int _last_time;		// µs
		//custom compact odometry structure
		tetra_ros::compactOdom _c_odom;
};

class OmniWheel : public WheelBase
{
	public:
		OmniWheel(Motor m_AvG, Motor m_AvD, Motor m_ArD, Motor m_ArG):
		WheelBase(m_AvG,  m_AvD,  m_ArD, m_ArG){}
		void set_motors(float xspeed, float yspeed, float wspeed);
	private:
		float _get_wheel_diameter(){return 60.0;}		
};

class DiffWheel : public WheelBase
{
	public:
		DiffWheel(Motor m_AvG, Motor m_AvD, Motor m_ArD, Motor m_ArG):
		WheelBase(m_AvG,  m_AvD,  m_ArD, m_ArG){}
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