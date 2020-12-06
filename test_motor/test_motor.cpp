#include <stdio.h>

#include "ros/console.h"
#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include <pigpiod_if2.h>

#define MOTOR1_A 2
#define MOTOR1_B 4
#define MOTOR1_PWM  3
#define MOTOR1_ENC1 14
#define MOTOR1_ENC2 15

#define ENCODER_PULSES 11
#define REDUCTION_RATIO 30
#define PULSES_PER_REV (ENCODER_PULSES * REDUCTION_RATIO * 4)

int encoder = 0;

static void _encoder(int whatever, unsigned int gpio,unsigned  int edge,unsigned  int tick)
{
   encoder++;
}

static int measure_speed(ros::Duration interval)
{
  float timeInterval = interval.toSec();

  float motor_speed = (float)((encoder/timeInterval)/PULSES_PER_REV);

  
  encoder = 0;

  return (int)(motor_speed * 60);

}


int main (int argc, char *argv[])
{
   ROS_INFO("HI MOM");

   int speed = 0; 
   ros::init(argc, argv, "test_motor");
   ros::NodeHandle n;

   char target_pi[20];
   strcpy(target_pi, "rospi.local");

   int i = 100;

   ROS_INFO("CONNECTING TO RPI");
   
   int pi_id = pigpio_start(target_pi, NULL);

   if (pi_id <0) return 1;

   ROS_INFO("RPI CONNECTED");

   set_mode(pi_id, MOTOR1_A,  PI_OUTPUT);
   set_mode(pi_id, MOTOR1_B, PI_OUTPUT);
   set_mode(pi_id, MOTOR1_PWM, PI_OUTPUT);
   set_mode(pi_id, MOTOR1_ENC1, PI_INPUT);
   set_mode(pi_id, MOTOR1_ENC2, PI_INPUT);

   callback(pi_id, MOTOR1_ENC1, EITHER_EDGE, _encoder);
   callback(pi_id, MOTOR1_ENC2, EITHER_EDGE, _encoder);

   ROS_INFO("starting motortest");

   ros::Publisher speed_pub = n.advertise<std_msgs::Int16>("speed", 1000);

   set_PWM_dutycycle(pi_id, MOTOR1_PWM, 0);

   ros::Duration one_sec(1.0);
   ros::Time last_time = ros::Time::now();

   ros::Rate loop_rate(500);

   gpio_write(pi_id, MOTOR1_A, 0);
   gpio_write(pi_id, MOTOR1_B, 1);
   set_PWM_dutycycle(pi_id, MOTOR1_PWM, i);

   ros::Duration interval;

   std_msgs::Int16 msg_speed;

   while (ros::ok())
   {
      interval = ros::Time::now() - last_time;
      last_time = ros::Time::now();
      if (interval > one_sec)
      {
         (i += 20) % 200;
         set_PWM_dutycycle(pi_id, MOTOR1_PWM, i);
      }
      speed = measure_speed(interval);
      msg_speed.data = speed;
      speed_pub.publish(msg_speed);

      ros::spinOnce();

   }

   set_PWM_dutycycle(pi_id, MOTOR1_PWM, 0);
   pigpio_stop(pi_id);
}