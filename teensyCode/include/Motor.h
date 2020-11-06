#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "Encoder.h"

class Motor
{
 public:
    Motor(int inA, int inB, int PWM, int encA, int encB):
      _InA(inA),
      _InB(inB),
      _PWM(PWM),
      _myEnc(encA, encB),
      _last_time(micros()),
      _last_encoder(0){}
      
    void set_pwm(int pwm_value);
	  void set_break();
	  int set_speed(int speed_value);
    int get_speed();
		
  private:
  	int _InA;
    int _InB;
    int _PWM;
    Encoder _myEnc;
    int _last_time;
    int _last_encoder;
    int _current_speed;
};

#define ENCODER_PULSES 11
#define REDUCTION_RATIO 30
#define PULSES_PER_REV (ENCODER_PULSES * REDUCTION_RATIO * 4)

#define CLOCKWISE HIGH
#define COUNTERCLOCKWISE LOW


#define MAX_SPEED_CLOCK 300
#define MAX_SPEED_COUNTERCLOCK 300

// PID values (tuned for 50 ms loop)
#define PID_Kp		5
#define PID_Kff1	0.8


#endif

