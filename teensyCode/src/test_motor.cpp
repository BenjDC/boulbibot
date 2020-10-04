/*************************************************************
Motor Shield 1-Channel DC Motor Demo
by Randy Sarafan

For more information see:
https://www.instructables.com/id/Arduino-Motor-Shield-Tutorial/

*************************************************************/

#include "Arduino.h"
#include "Encoder.h"
#include "test_motor.h"


Encoder myEnc(PIN_ENCODER_A,PIN_ENCODER_B);
long last_encoder;
long last_time;

long sigma_error;
int last_error;


void setup() {

  last_time = millis();
  last_encoder = myEnc.read();
  sigma_error = 0;
  last_error = 0;
  
  //Setup Channel A
  pinMode(MOTOR_PIN_DIRECTION, OUTPUT); //Initiates Motor Channel A pin
  pinMode(MOTOR_PIN_BREAK, OUTPUT); //Initiates Brake Channel A pin
  pinMode(MOTOR_PIN_PWM, OUTPUT); //Initiates Brake Channel A pin
  pinMode(MOTOR_DEACTIVATION, INPUT_PULLUP); //Initiates Brake Channel A pin

  Serial.begin(9600);
  Serial.println("Boulbibot motor Encoder Test:");
  
}



void loop(){

  if (digitalRead(MOTOR_DEACTIVATION) == LOW)
  {
    // set motor speed (rpm)
    set_motor_speed(100);
  }
  else
  {
    Serial.println("motor deactivated");
    // stop the motor
    set_motor_pwm(LOW, 0);
  }

  
  
  delay(50);

  
}

void set_motor_pwm(int motor_direction, int pwm_value)
{
  if (pwm_value == 0)
  {
    digitalWrite(MOTOR_PIN_BREAK, HIGH); //Engage the Brake
  }
  else
  {
    digitalWrite(MOTOR_PIN_DIRECTION, motor_direction); // set motor direction
    digitalWrite(MOTOR_PIN_BREAK, LOW);   //Disengage the Brake 
    analogWrite(MOTOR_PIN_PWM, pwm_value);   //Spins the motor on Channel A at requested speed
  }
}

// gets motor speed (revolutions per minute) 
void set_motor_speed(int target_speed)
{
  int actual_speed = get_motor_rpm();
  int error = target_speed - actual_speed;
  int delta_error = error - last_error;

  sigma_error += error;
  last_error = error;

  int pwm_command = PID_Kp * error + PID_Ki * sigma_error + PID_Kd * delta_error;

  int motor_direction = (pwm_command > 0) ? HIGH : LOW;

  set_motor_pwm(motor_direction, abs(pwm_command));



}

// gets motor speed (revolutions per minute) 
int get_motor_rpm()
{
  int rpm_value;
  int current_encoder = myEnc.read();

  int current_time = millis();
  float delta_time = current_time - last_time;
  float delta_encoder = current_encoder - last_encoder;

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

  float motor_speed = (float)(((delta_encoder/delta_time)  * 1000)/PULSES_PER_REV);


  last_time = current_time;
  last_encoder = current_encoder;

  rpm_value = (int)(motor_speed * 60);

  Serial.println(rpm_value);

  return rpm_value;
}
