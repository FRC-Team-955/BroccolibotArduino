#include <PID_v1.h>
#include <Encoder.h>

//Limit switch settings
//Has to be pin 2 or 3 (interrupt pins)
#define LIMIT_SWITCH 2
#define HOME_PWM -60 //How fast should we home?

//Motor controller pins
#define RPWM 5
#define LPWM 6
#define L_EN 7
#define R_EN 8

//Wheel PWM pins
#define WHEEL_LEFT 9
#define WHEEL_RIGHT 10

//Encoder pins
#define ENC_A 13
#define ENC_B 12

//Minimum and maximum possible PWM values
#define MIN_PWM 0
#define MAX_PWM 80

//What should set the serial line into mode rather than regular operation
#define MODE_SET -32000

//What mode is home
#define HOME_MODE 1

//What input values do we accept? (MM)
#define SAFE_BOUND_LOWER 25
#define SAFE_BOUND_UPPER 280

#define MM_TO_ENC_CONV 20.83

int left_pwm_speed = 0;
int right_pwm_speed = 0;

//What type of input are we taking?
enum Input_state {
	regular, //Take positional input
	mode, //Take mode input
} input_state;

enum Input_state_key {
	none = 0,
	home_key = 1, //Start homing
	left_pwm_key = 2, //Change left pwm
	right_pwm_key = 3, //Change right pwm
} input_state_key;

enum Movement_state {
	idle, //Try to get to the set point
	home, //Homing
} movement_state;

double Setpoint=0, PID_In, PID_Out;
double Kp=0.4, Ki=0.08, Kd=0.020; //100

//PID
PID pid(&PID_In, &PID_Out, &Setpoint, Kp, Ki, Kd, DIRECT);

//Quadrature encoder reader
Encoder quad(ENC_A, ENC_B);

//Able to set negative and positive motor speeds
void multi_direction (int magnitude) {
	if (abs(magnitude) > MIN_PWM) {
		if (magnitude > 0) {
			analogWrite(LPWM, 0);
			analogWrite(RPWM, magnitude);
		} else {
			analogWrite(LPWM, -magnitude);
			analogWrite(RPWM, 0);
		}
	}
}

//Stop homing, set encoder position to zero, and go right to normal operation
void stop_homing () {
	if (movement_state == home) {
		multi_direction(0);
		movement_state = idle;
		quad.write(0);
	}
	Serial.println("STOP HOMING");
}

void setup() {
	Serial.begin(115200);
	Serial.setTimeout(5); //Set if serial is too slow
	quad.write(0);

	pinMode(WHEEL_LEFT, OUTPUT);
	pinMode(WHEEL_RIGHT, OUTPUT);
	digitalWrite(WHEEL_LEFT, LOW);
	digitalWrite(WHEEL_RIGHT, LOW);

	pinMode(RPWM, OUTPUT);
	pinMode(LPWM, OUTPUT);
	digitalWrite(RPWM, LOW);
	digitalWrite(LPWM, LOW);

	//Enable the motors on both directions
	//TODO: Tie these into 5v instead?
	pinMode(R_EN, OUTPUT);
	pinMode(L_EN, OUTPUT);
	digitalWrite(R_EN, HIGH);
	digitalWrite(L_EN, HIGH);

	//Encoder pins
	pinMode(ENC_A, INPUT);
	pinMode(ENC_B, INPUT);

	//Limit switch pullup
	pinMode(LIMIT_SWITCH, INPUT_PULLUP);

	pid.SetMode(AUTOMATIC);
	pid.SetOutputLimits(-MAX_PWM, MAX_PWM);
	pid.SetSampleTime(5);

	//Interrupt for the limit switch
	attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH), stop_homing, FALLING);
}

void loop() {
	switch (movement_state) {
		case idle: //Try to move to the setpoint
			PID_In = quad.read();
			pid.Compute();
			multi_direction(PID_Out);
			break;

		case home: //Homing
			multi_direction(HOME_PWM);
			break;

		default: //How did we get here?!
			movement_state = idle; //Go right to something sane
			break;
	}	
}

void serialEvent () {
	int input = Serial.parseInt();
	
	//Put us in a mode setting mode
	if (input == MODE_SET) {
		input_state = mode;	
		input_state_key = none;	
		return;
	}

	switch (input_state) {
		case regular: //Take input as setpoints
			if (input <= SAFE_BOUND_UPPER && input >= SAFE_BOUND_LOWER) {
				Setpoint = ((float)input * MM_TO_ENC_CONV);
			}
			break;

		case mode: //Allow setting changes 
			switch(input_state_key) { //Which should we set?
				case none: //We don't know yet, let's find out
					input_state_key = input;
					return;
					break;

				case home_key: //We're setting the home key
					movement_state = home;
					input_state_key = none;
					input_state = regular;
					break;

				case left_pwm_key:
					analogWrite(WHEEL_LEFT, input);
					break;

				case right_pwm_key:
					analogWrite(WHEEL_RIGHT, input);
					break;

				default:
					input_state_key = none;
					input_state = regular;
					break;
			}

		default: //How did we get here?!
			//Go right to something sane
			input_state = regular; 
			input_state_key = none;
			break;
	}
}
