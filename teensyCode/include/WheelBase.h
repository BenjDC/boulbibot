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
		_last_time(micros()){}

		void test_motors(int target_speed);		
		void move_differential();
		void move_omni();
		void set_break();

		
	private:
		Motor _AvG;
		Motor _AvD;
		Motor _ArG;
		Motor _ArD;	
		int _last_time;

    
};

#endif