#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <pigpiod_if2.h>
#include <ros/ros.h>

// PID values (tuned for 50 ms loop)
#define PID_Kp    2
#define PID_Kff1  0.8


class Motor
{
 public:
    Motor(int inA, int inB, int PWM, int encA, int encB, int pi_id):
      _InA(inA),
      _InB(inB),
      _PWM(PWM),
      _ENCA(encA),
      _ENCB(encB),
      _last_time(ros::Time::now()),
      _measure_interval(0.05),
      _encoder_value(0),
      _total_pulse(0),
      _foward(1),
      _current_speed(0),
      _Kff1(PID_Kff1),
      _Kp(PID_Kp),
      _pi_id(pi_id)
      {

        set_mode(pi_id, _InA,  PI_OUTPUT);
        set_mode(pi_id, _InB, PI_OUTPUT);
        set_mode(pi_id, _PWM, PI_OUTPUT);
        set_mode(pi_id, _ENCA, PI_INPUT);
        set_mode(pi_id, _ENCB, PI_INPUT);

        ROS_INFO("creating callbaks");

        callback_ex(pi_id, _ENCA, EITHER_EDGE, encoder_cb_ex, this);
        callback_ex(pi_id, _ENCB, EITHER_EDGE, encoder_cb_ex, this);

        ROS_INFO("creating ok");
      }
      
    void set_pwm(int pwm_value);
	  void set_break();
	  int set_speed(int speed_value);
    int get_speed();
    void kill();
    int _encoder_value;
    int _total_pulse; 

    float _Kff1;
    float _Kp;
		
  private:
  	int _InA;
    int _InB;
    int _PWM;
    int _pi_id;
    int _ENCA;
    int _ENCB;
    int _foward;
    ros::Time _last_time;
    ros::Duration _measure_interval;
    
    int _current_speed;

    void _encoder_cb();
    static void encoder_cb_ex(int pi_id, uint32_t gpio, uint32_t level, uint32_t tick, void* pMotor);
    
};

#define ENCODER_PULSES 11
#define REDUCTION_RATIO 30
#define PULSES_PER_REV (ENCODER_PULSES * REDUCTION_RATIO * 4)

#define CLOCKWISE HIGH
#define COUNTERCLOCKWISE LOW


#define MAX_SPEED_CLOCK 300
#define MAX_SPEED_COUNTERCLOCK 300




#endif

