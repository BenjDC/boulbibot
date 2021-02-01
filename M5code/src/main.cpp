#define M5STACK_MPU6886 

#include <M5Stack.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>



//ros handle object
ros::NodeHandle  nh;
sensor_msgs::Imu imu_msg;
std_msgs::Int16 speed_msg;
ros::Publisher imu("IMU", &imu_msg);
ros::Publisher pub_speed("current_speed", &speed_msg);

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;


void setup()
{
  nh.getHardware()->setBaud(115200); //or what ever baud you want
  M5.begin();
  Wire.begin();
  
  M5.Power.begin();
  M5.IMU.Init();

  nh.initNode();  
  
  nh.advertise(imu);
  nh.advertise(pub_speed);
  
  imu_msg.header.frame_id = 0;

  while (!nh.connected()) {
    nh.spinOnce();
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(GREEN , BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(0, 0); M5.Lcd.print("Waiting ROS...");
    delay(100);
  }

}

void loop()
{

  M5.update();
  M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  M5.IMU.getAccelData(&accX,&accY,&accZ);
  M5.IMU.getAhrsData(&pitch,&roll,&yaw);


  imu_msg.linear_acceleration.x = 9.81 * accX;
  imu_msg.linear_acceleration.y = 9.81 * accY;
  imu_msg.linear_acceleration.y = 9.81 * accZ;

  imu_msg.angular_velocity.x = 3.141 / 180.0 * gyroX;
  imu_msg.angular_velocity.y = 3.141 / 180.0 * gyroY;
  imu_msg.angular_velocity.z = 3.141 / 180.0 * gyroZ;
  
  imu_msg.orientation.x = pitch;
  imu_msg.orientation.y = roll;
  imu_msg.orientation.z = yaw;

  imu_msg.header.stamp = nh.now();

  imu.publish( &imu_msg );

  nh.spinOnce();
  delay(10);
}
