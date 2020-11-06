#ifndef __WheelBase_H__
#define __WheelBase_H__

#include "Encoder.h"
#include "Motor.h"




class WheelBase
{
 
	public:
		WheelBase(Motor m_AvG, Motor m_AvD, Motor m_ArD, Motor m_ArG):
		_AvG(m_AvG),
		_AvD(m_AvD),
		_ArG(m_ArG),
		_ArD(m_ArD),
		_last_time(micros()),
		_xspeed(0),
		_yspeed(0),
		_wspeed(0){}


		void test_motors(int target_speed);		
		void set_break();
		void get_speed();
		void rpm_to_ms();
		virtual void set_motors(float xspeed, float yspeed, float wspeed) = 0;

	private:
		float rpm_to_ms(Motor m){return (m.get_speed() * PI * _wheelDiameter * 60 / 1000);}
		Motor _AvG;
		Motor _AvD;
		Motor _ArG;
		Motor _ArD;	
		int _last_time;		// µs
		float _xspeed; 		// m.s
		float _yspeed; 		// m.s
		float _wspeed;		// deg.s
		static const int _wheelDiameter;
		static const int _wheelDistance;
};

class OmniWheel : public WheelBase
{
	public:
		OmniWheel(Motor m_AvG, Motor m_AvD, Motor m_ArD, Motor m_ArG):
		WheelBase(m_AvG,  m_AvD,  m_ArD, m_ArG){}
		void set_motors(float xspeed, float yspeed, float wspeed);
	private:
		static const int _wheelDiameter = 60; // mm
		static const int _wheelDistance = 200; //mm
};

class DiffWheel : public WheelBase
{
	public:
		DiffWheel(Motor m_AvG, Motor m_AvD, Motor m_ArD, Motor m_ArG):
		WheelBase(m_AvG,  m_AvD,  m_ArD, m_ArG){}
		void set_motors(float xspeed, float yspeed, float wspeed);
	private:
		static const int _wheelDiameter = 80; // mm
		static const int _wheelDistance = 200; //mm
};

#define WHEEL_DIAMETER	0.06
#define WHEEL_DISTANCE	0.20

#define XSPEED_MAX 		0.80 	// m.s
#define XSACC_MAX 		0.10 	// m.s2
#define WSPEED_MAX  	40.0	// dps
#define WACC_MAX  		30.0	// dps/s
#define JOY_MAX  		0.5 	// m.s


#endif