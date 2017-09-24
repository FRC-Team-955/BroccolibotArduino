#include <PID_v1.h>
#include <Encoder.h>

//Limit switch settings
//Has to be pin 2 or 3 (interrupt pins)
#define LIMIT_SWITCH 2
#define HOME_PWM -10 //How fast should we home?

//Motor controller pins
#define RPWM 5
#define LPWM 6
#define L_EN 7
#define R_EN 8

//Wheel PWM pins
#define WHEEL_FRONT 9
#define WHEEL_BACK 10

//Encoder pins
#define ENC_A 12
#define ENC_B 13

//Minimum and maximum possible PWM values
#define MIN_PWM 0
#define MAX_PWM 150

//What should set the serial line into mode rather than regular operation
#define MODE_SET -32000

//What mode is home
#define HOME_MODE 1

//What input values do we accept?
#define SAFE_BOUND_LOWER 0
#define SAFE_BOUND_UPPER 32766

int front_pwm_speed = 0;
int back_pwm_speed = 0;

//What type of input are we taking?
enum Input_state {
	regular, //Take positional input
	mode, //Take mode input
} input_state;

enum Input_state_key {
	none = 0,
	home_key = 1, //Start homing
	front_pwm_key = 2, //Change front pwm
	back_pwm_key = 3, //Change back pwm
} input_state_key;

enum Movement_state {
	idle, //Try to get to the set point
	home, //Homing
} movement_state;

double Setpoint=0, PID_In, PID_Out;
double Kp=0.1, Ki=0, Kd=0.008; //100

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

//Stop homing, set encoder position to zero, and go back to normal operation
void stop_homing () {
	if (movement_state == home) {
		multi_direction(0);
		movement_state = idle;
		quad.write(0);
	}
}

void setup() {
	Serial.begin(115200);
	Serial.setTimeout(5); //Set if serial is too slow
	quad.write(0);

	pinMode(WHEEL_FRONT, OUTPUT);
	pinMode(WHEEL_BACK, OUTPUT);
	digitalWrite(WHEEL_FRONT, LOW);
	digitalWrite(WHEEL_BACK, LOW);

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

	//pinMode(LIMIT_SWITCH, INPUT);
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
			movement_state = idle; //Go back to something sane
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
				Setpoint = input;
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

				case front_pwm_key:
					analogWrite(WHEEL_FRONT, input);
					break;

				case back_pwm_key:
					analogWrite(WHEEL_BACK, input);
					break;

				default:
					input_state_key = none;
					input_state = regular;
					break;
			}

		default: //How did we get here?!
			//Go back to something sane
			input_state = regular; 
			input_state_key = none;
			break;
	}
}
