

//Period in microseconds
#define BOULBI_TEENSY_PERIOD 2000

// pinout for boulbipcb

//MOTOR1 
#define M1_INA 0
#define M1_INB 2
#define M1_PWM 1
#define M1_ENC1 3
#define M1_ENC2 4

//MOTOR2
#define M2_INA 26
#define M2_INB 27
#define M2_PWM 9
#define M2_ENC1 30
#define M2_ENC2 31

//MOTOR3
#define M3_INA 32
#define M3_INB 16
#define M3_PWM 15
#define M3_ENC1 17
#define M3_ENC2 18

//MOTOR4
#define M4_INA 19
#define M4_INB 23
#define M4_PWM 22
#define M4_ENC1 20
#define M4_ENC2 21

//Volatage Check 
#define VOLTAGE_PIN 14


//Ros callbacks
// void joy_cb( const sensor_msgs::Joy& cmd_msg);
// void cmd_vel_cb(const geometry_msgs::Twist& motor_command);
// geometry_msgs::Twist msg_cmd_vel;

// ros::Subscriber<sensor_msgs::Joy> sub_joy("joy", &joy_cb);
// ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_cb);

//int target_speed = 0;
