/*************************************************************
Motor Shield 1-Channel DC Motor Demo
by Randy Sarafan

For more information see:
https://www.instructables.com/id/Arduino-Motor-Shield-Tutorial/

*************************************************************/

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include "Arduino.h"
#include "Encoder.h"
#include "test_motor.h"


std_msgs::Int16 speed_msg;
std_msgs::Int16 target_msg;
ros::NodeHandle  nh;

ros::Publisher pub_speed("current_speed", &speed_msg);
ros::Publisher pub_target("target_speed", &target_msg);
Encoder myEnc(PIN_ENCODER_A,PIN_ENCODER_B);
long last_encoder;
long last_time;
long current_time;
long delta_time;

int target_speed = 0;

long sigma_error;
int last_error;

float PID_Ki = 1;
float PID_Kp = 0;
float PID_Kd = 0;


void KiCb( const std_msgs::Float32& Ki_msg){
  PID_Ki = Ki_msg.data;
}
void KpCb( const std_msgs::Float32& Kp_msg){
  PID_Kp = Kp_msg.data;
}
void KdCb( const std_msgs::Float32& Kd_msg){
  PID_Kd = Kd_msg.data;
}


ros::Subscriber<std_msgs::Float32> sub_Ki("set_Ki", &KiCb );
ros::Subscriber<std_msgs::Float32> sub_Kp("set_Kp", &KpCb );
ros::Subscriber<std_msgs::Float32> sub_Kd("set_Kd", &KdCb );


void setup() {

  nh.initNode();
  nh.advertise(pub_speed);
  nh.advertise(pub_target);
  nh.subscribe(sub_joy);
  nh.subscribe(sub_Ki);
  nh.subscribe(sub_Kp);
  nh.subscribe(sub_Kd);
  

  last_time = millis();
  last_encoder = myEnc.read();
  sigma_error = 0;
  last_error = 0;
  
  //Setup Channel A
  pinMode(MOTOR_PIN_A, OUTPUT); //Initiates Motor Channel A pin
  pinMode(MOTOR_PIN_B, OUTPUT); //Initiates Brake Channel A pin
  pinMode(MOTOR_PIN_PWM, OUTPUT); //Initiates Brake Channel A pin
  pinMode(MOTOR_DEACTIVATION, INPUT_PULLUP); //Initiates Brake Channel A pin
  pinMode(LED_PIN, OUTPUT);

  // set motor break
  break_motor();

  digitalWrite(LED_PIN, HIGH); 

}

void loop(){
  current_time = millis();
  delta_time = current_time - last_time;

  if (delta_time > 50)
  {
    nh.spinOnce();

    last_time = current_time;

    if (digitalRead(MOTOR_DEACTIVATION) == LOW)
    {
      // set motor speed (rpm)
      set_motor_speed(target_speed);
      //set_motor_pwm(HIGH, 255);
      //get_motor_rpm();
    }
    else
    {
      // stop the motor
      set_motor_pwm(LOW, 0);
    }

    target_msg.data = target_speed;

    //nh.loginfo(rpm_value);
    pub_speed.publish(&target_msg);
    
  }
}


void joy_cb( const sensor_msgs::Joy& cmd_msg) {

  //direction
  //target_speed = map(cmd_msg.axes[0], JOY_MIN, JOY_MAX, MAX_SPEED_COUNTERCLOCK, MAX_SPEED_CLOCK);
  target_speed = cmd_msg.axes[0] * MAX_SPEED_CLOCK;
}

void set_motor_pwm(int motor_direction, int pwm_value)
{
  if (pwm_value == 0)
  {
    break_motor(); //Engage the Brake
  }
  else
  {
    if (motor_direction == CLOCKWISE) 
    {
      digitalWrite(MOTOR_PIN_A, HIGH); 
      digitalWrite(MOTOR_PIN_B, LOW);
    }
    else
    {
      digitalWrite(MOTOR_PIN_A, LOW); 
      digitalWrite(MOTOR_PIN_B, HIGH);  
    }

    analogWrite(MOTOR_PIN_PWM, pwm_value);   //Spins the motor on Channel A at requested speed
  }
}

void break_motor()
{
  digitalWrite(MOTOR_PIN_A, LOW);
  digitalWrite(MOTOR_PIN_B, LOW);
  get_motor_rpm();
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

  speed_msg.data = rpm_value;

  //nh.loginfo(rpm_value);
  pub_speed.publish(&speed_msg);

  return rpm_value;
}