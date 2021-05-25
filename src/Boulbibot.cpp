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

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include "WheelBase.h"
#include "Boulbibot.h"
//#include "Motor.h"
#include <pigpiod_if2.h>

#define SPEED_TEST
//#define PWM_TEST


OmniWheel *boulbi;

// ros publishers definitions
std_msgs::Int16 actual_speed;
std_msgs::Int16 target_speed;

ros::Publisher target_pub;

void joy_cb( const sensor_msgs::Joy& cmd_msg) 
{

#ifdef PWM_TEST
  int pwm = (int)(abs(cmd_msg.axes[1]) * 250);
  target_speed.data = (uint16_t)pwm;
  target_pub.publish(target_speed);
  boulbi->test_pwm(pwm);
#endif

#ifdef SPEED_TEST
  int x_speed = (int16_t)(cmd_msg.axes[1] * 1000);
  int y_speed = (int16_t)(cmd_msg.axes[0] * 1000);

  int rotate_speed = (int16_t)(cmd_msg.axes[3] * 1000);
  
  //target_speed.data = 
  //uint16_t speed;

  //speed = (uint16_t)target_speed.data;
  
  //target_pub.publish(target_speed);
  boulbi->set_motors(x_speed, y_speed, rotate_speed);


#endif
  
}


int main (int argc, char *argv[])
{

  OmniWheel boulbi_bot;
  boulbi = &boulbi_bot;

  target_speed.data = 0;
  actual_speed.data = 0;

  int loop = 0;

  uint16_t a_speed[4];

  ROS_INFO("Hello boulbibot !");
  ros::init(argc, argv, "test_motor");
  ros::NodeHandle nh;
 
  target_pub = nh.advertise<std_msgs::Int16>("target_speed", 50);
  ros::Publisher speed_pub = nh.advertise<std_msgs::Int16>("actual_speed", 50);

  //ros::Subscriber speed_sub = nh.subscribe("actual_speed", 100, actual_speed_cb);
  ros::Subscriber joy_sub = nh.subscribe("joy", 100, joy_cb);
  

  
  
  ros::Rate loop_rate(20);

  
  boulbi->ping_motors();

#ifdef SPEED_TEST
  boulbi->init_control_mode(REG_CONTROL_MODE_VELOCITY_TORQUE);
#endif

#ifdef PWM_TEST
boulbi->init_control_mode(REG_CONTROL_MODE_PWM);
#endif

  

  boulbi->set_torque(1);

  ROS_INFO("Init OK, starting boulbi");

  //boulbi->set_motor_speed(0);

  while (ros::ok())
  {
    //
    ros::spinOnce();
    

    actual_speed.data = boulbi->get_speed();
    
    
    speed_pub.publish(actual_speed);
    

    //ROS_INFO("loop %i ok", loop++);
        
    loop_rate.sleep();
  }

  boulbi->set_torque(0);
  boulbi->kill();

  return 1;
}

