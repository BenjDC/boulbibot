/*************************************************************
Code Main de contrôle temps réel boulbibot
Benjamin De Coninck

Fonctionnalités:
* Controle moteur
* cinématique 4WD classique
* cinématique Mecanum

*************************************************************/

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "WheelBase.h"
#include "Boulbibot.h"

using std::placeholders::_1;

#define SPEED_TEST
//#define PWM_TEST


OmniWheel *boulbi;


//ros::Publisher target_pub;


class Boulbibot : public rclcpp::Node
{
  public:
    Boulbibot()
    : Node("boulbibot")
    {
      joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Boulbibot::joy_cb, this, _1));
    }
  private:
    void joy_cb( const sensor_msgs::msg::Joy::SharedPtr cmd_msg) 
    {

    #ifdef PWM_TEST
      int pwm = (int)(abs(cmd_msg.axes[1]) * 250);
      target_speed.data = (uint16_t)pwm;
      target_pub.publish(target_speed);
      boulbi->test_pwm(pwm);
    #endif

    #ifdef SPEED_TEST
      int x_speed = (int16_t)(cmd_msg->axes[1] * 1000);
      int y_speed = (int16_t)(cmd_msg->axes[0] * 1000);

      int rotate_speed = (int16_t)(cmd_msg->axes[3] * 1000);
      
      //target_speed.data = 
      //uint16_t speed;

      //speed = (uint16_t)target_speed.data;
      
      //target_pub.publish(target_speed);
      boulbi->set_motors(x_speed, y_speed, rotate_speed);
    #endif
      
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};


int main (int argc, char *argv[])
{

  OmniWheel boulbi_bot;
  boulbi = &boulbi_bot;

  boulbi->ping_motors();

#ifdef SPEED_TEST
  boulbi->init_control_mode(REG_CONTROL_MODE_VELOCITY_TORQUE);
#endif

#ifdef PWM_TEST
boulbi->init_control_mode(REG_CONTROL_MODE_PWM);
#endif

  

  boulbi->set_torque(1);

  

  //boulbi->set_motor_speed(0);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Boulbibot>());
  rclcpp::shutdown();
  
  return 0;
}

