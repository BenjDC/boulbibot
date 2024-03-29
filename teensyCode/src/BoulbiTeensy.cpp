/*************************************************************
Code Main de contrôle temps réel boulbibot
Benjamin De Coninck

Fonctionnalités:
* Controle moteur
* cinématique 4WD classique
* cinématique Mecanum


*************************************************************/

#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <tetra_ros/compactOdom.h>
#include <std_msgs/Int16.h>
#include <TimedBlink.h>
#include <Battery.h>
#include "Arduino.h"
#include "WheelBase.h"
#include "BoulbiTeensy.h"
#include "Battery.h"

//ros handle object
ros::NodeHandle  nh;

// ros publishers definitions
std_msgs::Int16 speed_msg;
std_msgs::Int16 target_msg;
tetra_ros::compactOdom msg_compact_odom;
ros::Publisher pub_speed("current_speed", &speed_msg);
ros::Publisher pub_target("target_speed", &target_msg);
ros::Publisher compactOdom_pub("compact_odom", &msg_compact_odom);

int target_speed =0;

// Led helper
TimedBlink monitor(LED_BUILTIN);

DiffWheel boulbi(Motor(M1_INA, M1_INB, M1_PWM,M1_ENC1, M1_ENC2),
                Motor(M2_INA, M2_INB, M2_PWM,M2_ENC1, M2_ENC2),
                Motor(M3_INA, M3_INB, M3_PWM,M3_ENC1, M3_ENC2),
                Motor(M4_INA, M4_INB, M4_PWM,M4_ENC1, M4_ENC2));


Battery battery(2550, 3150, VOLTAGE_PIN);


int last_time;
bool connected; 

void setup() {

  nh.initNode();
  nh.advertise(pub_speed);
  nh.advertise(pub_target);
  nh.subscribe(sub_joy);

  pinMode(LED_BUILTIN, OUTPUT);
  monitor.blink(1000,1000);
  connected=false;
}

void loop(){
  
  int current_time = micros();
  int delta_time = current_time - last_time;  

  nh.spinOnce();
  // led blinking
  monitor.blink();

  if (delta_time > BOULBI_TEENSY_PERIOD)
  {
    if (!nh.connected())
    {
      // stop the robot
      boulbi.set_break();
      connected = false;
      monitor.blink(500, 2500);

      digitalWrite(0, HIGH); 
      digitalWrite(2, LOW);
      analogWrite(1, 0);
    }
    else 
    {
      if (!connected)
      {
        connected = true;
        nh.loginfo("boulbibot teensy connected !");
        tellBatteryLevel();
        monitor.blink(2500, 500);
      }

      // update odometry
      msg_compact_odom = boulbi.update_position();
      // timestamp odometry message
		  msg_compact_odom.stamp = nh.now();
      // publish odometry message
		  compactOdom_pub.publish(&msg_compact_odom);
    }


    // activate code for battery usage
    /*
    if (battery.level() < BATTERY_LIMIT)
    {
      monitor.blink(200, 100);
    }
    */
    
  }
}

void tellBatteryLevel()
{
  float battery_voltage;

  //divider brige is 3.3k/1.1k (divided by 4)
  battery_voltage = battery.voltage() * 4.0;
  
  // Allocates storage
  char *battery_msg = (char*)malloc(40 * sizeof(char));
  char *voltage_msg = (char*)malloc(5 * sizeof(char));

  dtostrf(battery_voltage, 5, 1, voltage_msg);

  sprintf(battery_msg, "Battery voltage : %s\n", voltage_msg);

  nh.loginfo(battery_msg);

  free(battery_msg);
  free(voltage_msg);
}



void joy_cb( const sensor_msgs::Joy& cmd_msg) {

  target_speed = cmd_msg.axes[0] * MAX_SPEED_CLOCK;

  nh.loginfo("testing motor !");

  boulbi.test_motors(target_speed);

}

void cmd_vel_cb(const geometry_msgs::Twist& motor_command)
{
  boulbi.set_motors(motor_command.linear.x, motor_command.linear.y, motor_command.angular.z);
}