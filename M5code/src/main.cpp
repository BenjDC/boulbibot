#define M5STACK_MPU6886 

#include <M5Stack.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
#include <WiFi.h>


const char* ssid     = "foo";
const char* password = "bar";

//ros handle object
ros::NodeHandle  nh;
sensor_msgs::Imu imu_msg;
std_msgs::Int16 speed_msg;
ros::Publisher imu("IMU", &imu_msg);
ros::Publisher pub_speed("current_speed", &speed_msg);

// Set the rosserial socket server IP address
IPAddress server(192,168,0,30);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;


float temp = 0.0F;


void setup()
{
  
  M5.begin();
  Wire.begin();
  
  M5.Power.begin();
  M5.IMU.Init();

  
  imu_msg.header.frame_id = 0;

  

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(2);
  
  M5.Lcd.setCursor(0, 0); 
  M5.Lcd.print("WiFi..."); 

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);    
    M5.Lcd.print(".");
  }
  M5.Lcd.print("OK !");

  M5.Lcd.setCursor(0, 20); 
  M5.Lcd.print("IP address: ");
  M5.Lcd.print(WiFi.localIP());

  nh.getHardware()->setConnection(server, serverPort);
  
  nh.initNode();  
  
  nh.advertise(imu);
  
  M5.Lcd.setCursor(0, 40); 
  M5.Lcd.print("ROSSerial...");

  while (!nh.connected()) {
    nh.spinOnce();
    delay(100);
  }

  M5.Lcd.print("OK !");

   delay(1000);

   M5.Lcd.fillScreen(BLACK);

}

void loop()
{
  

  M5.update();
  M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  M5.IMU.getAccelData(&accX,&accY,&accZ);
  M5.IMU.getAhrsData(&pitch,&roll,&yaw);
  M5.IMU.getTempData(&temp);

  M5.Lcd.setCursor(0, 20);
  M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ", gyroX, gyroY, gyroZ);
  M5.Lcd.setCursor(220, 42);
  M5.Lcd.print(" o/s");
  M5.Lcd.setCursor(0, 65);
  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", accX, accY, accZ);
  M5.Lcd.setCursor(220, 87);
  M5.Lcd.print(" G");
  M5.Lcd.setCursor(0, 110);
  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", pitch, roll, yaw);
  M5.Lcd.setCursor(220, 132);
  M5.Lcd.print(" degree");
  M5.Lcd.setCursor(0, 155);
  M5.Lcd.printf("Temperature : %.2f C", temp);


  imu_msg.header.frame_id= "/imu";

  imu_msg.linear_acceleration.x = 9.81 * accX;
  imu_msg.linear_acceleration.y = 9.81 * accY;
  imu_msg.linear_acceleration.y = 9.81 * accZ;

  imu_msg.angular_velocity.x = 3.141 / 180.0 * gyroX;
  imu_msg.angular_velocity.y = 3.141 / 180.0 * gyroY;
  imu_msg.angular_velocity.z = 3.141 / 180.0 * gyroZ;
  
  imu_msg.orientation.x = 3.141 / 180.0 * pitch;
  imu_msg.orientation.y = 3.141 / 180.0 * roll;
  imu_msg.orientation.z = 3.141 / 180.0 * yaw;

  imu_msg.header.stamp = nh.now();

  imu.publish( &imu_msg );

  nh.spinOnce();
  delay(10);
}
