


#define PIN_ENCODER_A 3
#define PIN_ENCODER_B 4

#define MOTOR_PIN_PWM 0
#define MOTOR_PIN_A 1
#define MOTOR_PIN_B 2

#define MOTOR_DEACTIVATION 5 

#define LED_PIN 13

#define ENCODER_PULSES 11
#define REDUCTION_RATIO 30
#define PULSES_PER_REV (ENCODER_PULSES * REDUCTION_RATIO * 4)

#define CLOCKWISE HIGH
#define COUNTERCLOCKWISE LOW

#define JOY_MAX 32767
#define JOY_MIN -32767

#define MAX_SPEED_CLOCK 300
#define MAX_SPEED_COUNTERCLOCK 300

// PID values (tuned for 50 ms loop)
//#define PID_Kp	6.6
//#define PID_Ki	39.6
//#define PID_Kd	0.275

//#define PID_Kp	9
//#define PID_Ki	1.2
//#define PID_Kd	4

void set_motor_pwm(int motor_direction, int pwm_value);
void set_motor_speed(int speed_value);
void break_motor();
int get_motor_rpm();

void joy_cb( const sensor_msgs::Joy& cmd_msg);

ros::Subscriber<sensor_msgs::Joy> sub_joy("joy", joy_cb);

