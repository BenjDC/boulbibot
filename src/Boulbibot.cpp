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
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include "WheelBase.h"
#include "Boulbibot.h"
//#include "Motor.h"
#include <pigpiod_if2.h>


DiffWheel *boulbi;

// ros publishers definitions
std_msgs::Int16 actual_speed;
std_msgs::Int16 target_speed;


void joy_cb( const sensor_msgs::Joy& cmd_msg) 
{
  //int pwm = (int)(abs(cmd_msg.axes[1]) * 250);
  int speed = (int)(cmd_msg.axes[4] * 330);

  // if (pwm != 0)
  // {
    

  //   target_speed.data = (uint16_t)pwm;
  //   boulbi->test_pwm(pwm);
  // }
  
  // else
   if(speed != 0)
  {
    target_speed.data = (uint16_t)speed;
    boulbi->test_speed(speed);
  }

  if (cmd_msg.buttons[0] == 1)
  {
    ROS_INFO("BREAK IT UP !");
    boulbi->test_pwm(0);
    return;
  }
}


int main (int argc, char *argv[])
{

  DiffWheel boulbi_bot(AV_G_ID, AV_D_ID, AR_G_ID, AR_D_ID);
  boulbi = &boulbi_bot;

  target_speed.data = 0;
  actual_speed.data = 0;

  int loop = 0;

  uint16_t a_speed;

  ROS_INFO("boulbibot starting !");
  ros::init(argc, argv, "test_motor");
  ros::NodeHandle nh;
 
  ros::Publisher target_pub = nh.advertise<std_msgs::Int16>("target_speed", 50);
  ros::Publisher speed_pub = nh.advertise<std_msgs::Int16>("actual_speed", 50);

  //ros::Subscriber speed_sub = nh.subscribe("actual_speed", 100, actual_speed_cb);
  ros::Subscriber joy_sub = nh.subscribe("joy", 100, joy_cb);
  
  
  ros::Rate loop_rate(2);

  
  boulbi->ping_motors();
  boulbi->init_control_mode(REG_CONTROL_MODE_VELOCITY_TORQUE);

  boulbi->set_torque(1);
  boulbi->test_speed(0);

  ROS_INFO("Init OK, starting boulbi");

  while (ros::ok())
  {
    //
    ros::spinOnce();
    

    boulbi->get_speed(&a_speed);
    actual_speed.data = a_speed;
    
    speed_pub.publish(actual_speed);
    target_pub.publish(target_speed);

    //ROS_INFO("loop %i ok", loop++);
        
    loop_rate.sleep();
  }

  boulbi->set_torque(0);
  boulbi->kill();

  return 1;
}

