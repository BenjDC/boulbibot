/*************************************************************
Code Main de contrôle temps réel boulbibot
Benjamin De Coninck

Fonctionnalités:
* Controle moteur
* cinématique 4WD classique
* cinématique Mecanum

*************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32.h>
//#include "WheelBase.h"
#include "Boulbibot.h"
#include "Motor.h"
#include <pigpiod_if2.h>

DiffWheel boulbi()

// ros publishers definitions
std_msgs::Int16MultiArray actual_speed;
std_msgs::Int16MultiArray target_speed;


void joy_cb( const sensor_msgs::Joy& cmd_msg) {

  if (cmd_msg.buttons[0] == 1)
  {
    ROS_INFO("BREAK IT UP !");
    boulbi->set_break();
    target_msg.data = 0;
    return;
  }

  int target_pwm = (int)(cmd_msg.axes[1] * 200);
  int target_speed = (int)(cmd_msg.axes[4] * 330);

  if (target_pwm != 0)
  {
    target_msg.data = target_pwm; 
    boulbi->set_pwm(target_pwm);
  }
  
  else if(target_speed != 0)
  {
    target_msg.data = target_speed; 
    boulbi->set_speed(target_speed);
  }
}


int main (int argc, char *argv[])
{
  ROS_INFO("boulbibot starting !");
  ros::init(argc, argv, "test_motor");
  ros::NodeHandle nh;
 
  ros::Publisher target_pub = nh.advertise<std_msgs::Int16MultiArray>("target_speed", 50);

  ros::Publisher speed_pub = nh.subscribe("actual_speed", 100, actual_speed_cb);
  ros::Subscriber sub = nh.subscribe("joy", 100, joy_cb);
  
  
  set_break();

  ros::Rate loop_rate(20);

  ROS_INFO("starting loop !");

  while (ros::ok())
  {

    //ROS_INFO("looping !");
    ros::spinOnce();

    target_pub.publish(target_msg);
    
    loop_rate.sleep();
  }

  boulbi->kill();

  return 1;
}

