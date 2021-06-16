/*************************************************************
Code Main de contrôle temps réel boulbibot
Benjamin De Coninck

Fonctionnalités:
* Controle moteur
* cinématique 4WD classique
* cinématique Mecanum

*************************************************************/

#include "rclcpp/rclcpp.hpp"



#include "std_msgs/msg/string.hpp"
//#include "std_msgs/msgs/Int16.hpp"
#include "sensor_msgs/msg/joy.hpp"


//#include "geometry_msgs/msgs/Twist.hpp"

#include "WheelBase.h"
#include "Boulbibot.h"
//#include "Motor.h"
#include <pigpiod_if2.h>

#define SPEED_TEST
//#define PWM_TEST


OmniWheel *boulbi;

// ros publishers definitions
//std_msgs::Int16 actual_speed;
//std_msgs::Int16 target_speed;

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

  //target_speed.data = 0;
  //actual_speed.data = 0;

  int loop = 0;

  uint16_t a_speed[4];

  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("talker");

  ROS_INFO("Hello boulbibot !");

  rclcpp::Rate loop_rate(10);



  joy_sub = this->create_subscription<sensor_msgs::msg::Joy >("joy", 100, std::bind(&MinimalSubscriber::topic_callback, this, _1));

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
        
    loop_rate.sleep();
  }

  boulbi->set_torque(0);
  boulbi->kill();

  return 1;
}

