#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <pigpiod_if2.h>
#include <ros/ros.h>


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
      _measure_interval(.001),
      _pi_id(pi_id)
      {
        set_mode(pi_id, _InA,  PI_OUTPUT);
        set_mode(pi_id, _InB, PI_OUTPUT);
        set_mode(pi_id, _PWM, PI_OUTPUT);
        set_mode(pi_id, _ENCA, PI_INPUT);
        set_mode(pi_id, _ENCB, PI_INPUT);

        callback(pi_id, _ENCA, EITHER_EDGE, _encoder_cb_ex);
        callback(pi_id, _ENCB, EITHER_EDGE, _encoder_cb_ex);
      }
      
    void set_pwm(int pwm_value);
	  void set_break();
	  int set_speed(int speed_value);
    int get_speed();
    void kill();
    static int _encoder_value;
		
  private:
  	int _InA;
    int _InB;
    int _PWM;
    int _pi_id;
    int _ENCA;
    int _ENCB;
    ros::Time _last_time;
    ros::Duration _measure_interval;
    
    int _current_speed;

    void _encoder_cb(int pi_id, unsigned int gpio,unsigned  int edge,unsigned  int tick);
    static void _encoder_cb_ex(int pi_id, int gpio, int level, uint32_t tick);
    
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

