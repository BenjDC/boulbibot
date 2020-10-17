/*************************************************************
Code de contrôle moteur
Benjamin De Coninck

Moteurs utilisés :
https://fr.aliexpress.com/item/33001192874.html

*************************************************************/

#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include "Arduino.h"
#include "motor.h"
#include "boulbiTeensy.h"

std_msgs::Int16 speed_msg;
std_msgs::Int16 target_msg;
ros::NodeHandle  nh;

ros::Publisher pub_speed("current_speed", &speed_msg);
ros::Publisher pub_target("target_speed", &target_msg);

int last_time;

boublibot::boublibot boulbi(Motor(M1_INA, M1_INB, M1_PWM,M1_ENC1, M1_ENC2),
                            Motor(M2_INA, M2_INB, M2_PWM,M2_ENC1, M2_ENC2),
                            Motor(M3_INA, M3_INB, M3_PWM,M3_ENC1, M3_ENC2),
                            Motor(M4_INA, M4_INB, M4_PWM,M4_ENC1, M4_ENC2));


void setup() {

  nh.initNode();
  nh.advertise(pub_speed);
  nh.advertise(pub_target);
  nh.subscribe(sub_joy);

  last_time = millis();

  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN, HIGH); 

}

void loop(){
  int current_time = millis();
  int delta_time = current_time - last_time;

  if (delta_time > BOULBI_TEENSY_PERIOD)
  {
    nh.spinOnce();    

    // set motor speed (rpm)
    speed_msg.data = boulbi.test_motor_1(target_speed);
    pub_speed.publish(&speed_msg);

    target_msg.data = target_speed;
    pub_target.publish(&target_msg);
    
  }
}

void boublibot::boublibot(Motor m_AvG, Motor m_AvD, Motor m_ArD, Motor m_ArG)
{
  _AvG = m_AvG;
  _AvD = m_AvD;
  _ArG = m_ArG;
  _ArD = m_ArD;

  _last_time = millis();  
}

void joy_cb( const sensor_msgs::Joy& cmd_msg) {

 target_speed = cmd_msg.axes[0] * MAX_SPEED_CLOCK;

}

void boulbibot::boulbistop()
{
  _AvG.set_break();
  _AvD.set_break();
  _ArG.set_break();
  _ArD.set_break();

}

int boublibot::test_motor_1(int target_speed)
{
  return _AvG.set_speed(target_speed);
}

