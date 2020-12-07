/*************************************************************
Code de contrôle moteur
Benjamin De Coninck

Composants utilisés :
Moteurs : https://fr.aliexpress.com/item/33001192874.html
Controlleurs : https://www.pololu.com/product/1451

*************************************************************/

#include "../include/Motor.h"
#include <ros/ros.h>
#include <pigpiod_if2.h>


void Motor::set_pwm(int pwm_value)
{
  if (pwm_value == 0)
  {
    set_break(); //Engage the Brake
  }
  else
  {
    if (pwm_value > 0) 
    {
      gpio_write(_pi_id, _InA, 1);
      gpio_write(_pi_id, _InB, 0);      
    }
    else
    {
      gpio_write(_pi_id, _InA, 0);
      gpio_write(_pi_id, _InB, 1);   
    }

    set_PWM_dutycycle(_pi_id, _PWM, abs(pwm_value));
  }
}

void Motor::kill()
{
  pigpio_stop(_pi_id);
}


void Motor::set_break()
{
  gpio_write(_pi_id, _InA, 0);
  gpio_write(_pi_id, _InB, 0);
  set_PWM_dutycycle(_pi_id,_PWM, 0);
}

// gets motor speed (revolutions per minute) 
int Motor::set_speed(int target_speed)
{
  int actual_speed = get_speed();
  int error = target_speed - actual_speed;

  int pwm_command = PID_Kp * error + PID_Kff1 * target_speed;

  set_pwm(pwm_command);

  return actual_speed;
}


void Motor::_encoder_cb(int pi_id, unsigned int gpio,unsigned  int edge,unsigned  int tick)
{
   _encoder_value++;
}

void Motor::_encoder_cb_ex(int gpio, int level, uint32_t tick)
{


   _encoder_cb(gpio, level, tick); /* Call the instance callback. */
}



// gets motor speed (revolutions per minute) 
int Motor::get_speed()
{
  ros::Time current_time = ros::Time::now();
  ros::Duration delta_time = current_time - _last_time;

  // do not compute speed more than once every ms
  if (delta_time < _measure_interval)
  {
    return _current_speed;
  }

  float interval = _measure_interval.toSec();

  float motor_speed = (float)((_encoder_value/interval)/PULSES_PER_REV);

  _encoder_value = 0;

  _current_speed = (int)(motor_speed * 60);

  return _current_speed;
}

