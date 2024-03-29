/*************************************************************
Code de contrôle moteur
Benjamin De Coninck

Composants utilisés :
Moteurs : https://fr.aliexpress.com/item/33001192874.html
Controlleurs : https://www.pololu.com/product/1451

*************************************************************/

#include "Arduino.h"
#include "Encoder.h"
#include "Motor.h"

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
      digitalWrite(_InA, HIGH); 
      digitalWrite(_InB, LOW);
    }
    else
    {
      digitalWrite(_InA, LOW); 
      digitalWrite(_InB, HIGH);  
    }

    analogWrite(_PWM, abs(pwm_value)); 
  }
}

void Motor::set_break()
{
  digitalWrite(_InA, LOW);
  digitalWrite(_InB, LOW);
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

// gets motor speed (revolutions per minute) 
int Motor::get_speed()
{
  
  int current_time = micros();
  int delta_time = current_time - _last_time;

  // do not compute speed more than once every ms
  if (delta_time < 1000)
  {
    return _current_speed;
  }

  int current_encoder = _myEnc.read();
  float delta_encoder = current_encoder - _last_encoder;

  // handle the encoder_delta overflow/underflow case
  if (abs(delta_encoder) > 32768)
  {
    if (delta_encoder > 0)
    {
      delta_encoder = -((32768 * 2) - delta_encoder);
    }
    else
    {
      delta_encoder = ((32768 * 2) + delta_encoder);
    }    
  }

  float motor_speed = (float)(((delta_encoder/delta_time)  * 1000000)/PULSES_PER_REV);

  _last_time = current_time;
  _last_encoder = current_encoder;

  _current_speed = (int)(motor_speed * 60);

  return _current_speed;
}

