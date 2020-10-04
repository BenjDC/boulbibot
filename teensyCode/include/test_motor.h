


#define PIN_ENCODER_A 2
#define PIN_ENCODER_B 3

#define MOTOR_PIN_BREAK 8
#define MOTOR_PIN_PWM 11
#define MOTOR_DEACTIVATION 7 
#define MOTOR_PIN_DIRECTION 13


#define ENCODER_PULSES 11
#define REDUCTION_RATIO 30
#define PULSES_PER_REV (ENCODER_PULSES * REDUCTION_RATIO * 4)

// PID values (tuned for 50 ms loop)
//#define PID_Kp	6.6
//#define PID_Ki	39.6
//#define PID_Kd	0.275

//#define PID_Kp	9
//#define PID_Ki	1.2
//#define PID_Kd	4

void set_motor_pwm(int motor_direction, int pwm_value);
void set_motor_speed(int speed_value);

int get_motor_rpm();