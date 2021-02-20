/*************************************************************
Code de contrôle moteur
Benjamin De Coninck

Composants utilisés :
Moteurs : https://fr.aliexpress.com/item/33001192874.html
Controlleurs : https://www.pololu.com/product/1451

*************************************************************/

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
      _foward = 1;
    }
    else
    {
      gpio_write(_pi_id, _InA, 0);
      gpio_write(_pi_id, _InB, 1);   
      _foward = -1;
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
void Motor::set_speed(int target_speed)
{
  int error = target_speed - _current_speed;

  int pwm_command = _Kp * error + _Kff1 * target_speed;

  pwm_command = pwm_command > 255 ? 255 : pwm_command;

  set_pwm(pwm_command);
}


void Motor::_encoder_cb()
{
  _encoder_value++;
  /*
  if (((gpio == _ENCA) && (level == gpio_read(_pi_id, _ENCB))) || 
    ((gpio == _ENCB) && (level != gpio_read(_pi_id, _ENCA))))
    {
      _encoder_value++;
    }
    else
    {
     _encoder_value--; 
    }
  */
  //ROS_INFO("CALLBAK");
}

void Motor::encoder_cb_ex(int pi_id, uint32_t gpio, uint32_t level, uint32_t tick, void* pMotor)
{
  Motor *cbMotor = (Motor*)pMotor;
  cbMotor->_encoder_cb();
}



// gets motor speed (revolutions per minute) 
int Motor::get_speed()
{


  ros::Time current_time = ros::Time::now();
  ros::Duration delta_time = current_time - _last_time;
  _last_time = current_time;

  //ROS_INFO("measuring !");
  float interval = delta_time.toSec();
  float motor_speed = (float)((_encoder_value/interval)/PULSES_PER_REV);
  _current_speed = (int)(motor_speed * 60) * _foward;

  
  _total_pulse += abs(_encoder_value);

  //ROS_INFO("interval : %f, encoder delta : %i, current speed %i, encoder_total %i", interval, _encoder_value, _current_speed, _total_pulse);

  
  _encoder_value = 0;
  return _current_speed;
}

