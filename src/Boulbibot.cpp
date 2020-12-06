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
#include "WheelBase.h"
#include "Boulbibot.h"


//ros handle object
ros::NodeHandle  nh;

// ros publishers definitions
std_msgs::Int16 speed_msg;
std_msgs::Int16 target_msg;
/* ros::Publisher pub_speed("current_speed", &speed_msg);
ros::Publisher pub_target("target_speed", &target_msg);
ros::Publisher pub_odom("compact_odom", &msg_compact_odom); */


DiffWheel boulbi(M1_INA, M1_INB, M1_PWM,M1_ENC1, M1_ENC2,
                M2_INA, M2_INB, M2_PWM,M2_ENC1, M2_ENC2,
                M3_INA, M3_INB, M3_PWM,M3_ENC1, M3_ENC2,
                M4_INA, M4_INB, M4_PWM,M4_ENC1, M4_ENC2);

int last_time;
bool connected; 

ROS_INFO("boulbibot starting !");

int main (int argc, char *argv[])
{
  ros::init(argc, argv, "test_motor");
  ros::NodeHandle nh;
  
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

    // update and publish odometry
    // TODO : set odom type for publisher
    boulbi.update_position();
    pub_odom.publish(&msg_compact_odom);
  }

  return 1
}


void joy_cb( const sensor_msgs::Joy& cmd_msg) {

  int target_speed = (int)(cmd_msg.axes[1] * 200);
  nh.loginfo("testing motor !");
  boulbi.test_motors(target_speed);

}

void cmd_vel_cb(const geometry_msgs::Twist& motor_command)
{
  boulbi.set_motors(motor_command.linear.x, motor_command.linear.y, motor_command.angular.z);
}