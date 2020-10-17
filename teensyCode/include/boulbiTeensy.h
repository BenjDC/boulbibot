
class boublibot
{

	public:
		boublibot(Motor m_AvG, Motor m_AvD, Motor m_ArD, Motor m_ArG);
		
	private:
		Motor _AvG;
		Motor _AvD;
		Motor _ArG;
		Motor _ArD;	
		int _last_time;

		int test_motor_1(target_speed);
		void boulbistop()
};

#define LED_PIN 13
#define BOULBI_TEENSY_PERIOD 2

// pinout for boulbipcb

//MOTOR1 
#define M1_INA 2
#define M1_INB 0
#define M1_PWM 1
#define M1_ENC1 3
#define M1_ENC2 4

//MOTOR2
#define M2_INA 27
#define M2_INB 28
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


void joy_cb( const sensor_msgs::Joy& cmd_msg);

ros::Subscriber<sensor_msgs::Joy> sub_joy("joy", joy_cb);

int target_speed = 0;
