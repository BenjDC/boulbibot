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
#include <std_msgs/Int16.h>
//#include "WheelBase.h"
#include "Boulbibot.h"


//ros handle object
ros::NodeHandle  nh;

// ros publishers definitions
std_msgs::Int16 speed_msg;
std_msgs::Int16 target_msg;

/*
DiffWheel boulbi(M1_INA, M1_INB, M1_PWM,M1_ENC1, M1_ENC2,
                M2_INA, M2_INB, M2_PWM,M2_ENC1, M2_ENC2,
                M3_INA, M3_INB, M3_PWM,M3_ENC1, M3_ENC2,
                M4_INA, M4_INB, M4_PWM,M4_ENC1, M4_ENC2);
*/

testMotor Motor(M1_INA, M1_INB, M1_PWM,M1_ENC1, M1_ENC2, pigpio_start("rospi.local", NULL));

int main (int argc, char *argv[])
{
  ROS_INFO("boulbibot starting !");

  ros::init(argc, argv, "test_motor");
  ros::NodeHandle nh;
  
  ros::Publisher speed_pub = nh.advertise<std_msgs::Int16>("motor_speed", 1000);
  ros::Publisher target_pub = nh.advertise<std_msgs::Int16>("target_speed", 1000);
  
  boulbi.set_break();
  /*
  nh.advertise(pub_speed);
  nh.advertise(pub_target);
  nh.advertise(pub_odom);
  nh.subscribe(sub_joy);
  */

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ros::spinOnce();

    speed_msg.data = testMotor.get_speed();    
    target_pub.publish(&target_msg);
    target_pub.publish(&speed_msg);
  }

  return 1
}


void joy_cb( const sensor_msgs::Joy& cmd_msg) {

  int target_speed = (int)(cmd_msg.axes[1] * 200);
  target_msg.data = target_speed; 
  testMotor.set_speed(target_speed);
}
