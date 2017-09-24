#include <PID_v1.h>
#include <Encoder.h>

//Has to be 2 or 3 (interrupt pins)
#define LIMIT_SWITCH 2
#define HOME_PWM 10

#define RPWM 5
#define LPWM 6
#define L_EN 7
#define R_EN 8

#define ENC_A 12
#define ENC_B 13

#define MIN_PWM 0
#define MAX_PWM 150

#define MODE_SET -32000
#define HOME_MODE 1

#define SAFE_BOUND_LOWER 0
#define SAFE_BOUND_UPPER 32766

enum Input_state {
	regular,
	mode,
} input_state;
enum Movement_state {
	idle,
	home,
} movement_state;

double Setpoint=0, PID_In, PID_Out;
//double Kp=0.2, Ki=0, Kd=0.008; //100
double Kp=0.1, Ki=0, Kd=0.008; //100

//PID
PID pid(&PID_In, &PID_Out, &Setpoint, Kp, Ki, Kd, DIRECT);

//Quadrature encoder reader
Encoder quad(ENC_A, ENC_B);

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

	pinMode(RPWM, OUTPUT);
	pinMode(LPWM, OUTPUT);
	digitalWrite(RPWM, LOW);
	digitalWrite(LPWM, LOW);

	pinMode(R_EN, OUTPUT);
	pinMode(L_EN, OUTPUT);
	digitalWrite(R_EN, HIGH);
	digitalWrite(L_EN, HIGH);

	pinMode(ENC_A, INPUT);
	pinMode(ENC_B, INPUT);

	//pinMode(LIMIT_SWITCH, INPUT);
	pinMode(LIMIT_SWITCH, INPUT_PULLUP);


	pid.SetMode(AUTOMATIC);
	pid.SetOutputLimits(-MAX_PWM, MAX_PWM);
	pid.SetSampleTime(5);

	attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH), stop_homing, FALLING);
}

void loop() {
	switch (movement_state) {
		case idle:
			PID_In = quad.read();
			pid.Compute();
			multi_direction(PID_Out);
			break;

		case home:
			multi_direction(HOME_PWM);
			break;

		default: //How did we get here?!
			movement_state = idle;
			break;
	}	
}

void serialEvent () {
	int input = Serial.parseInt();
	if (input == MODE_SET) {
		input_state = mode;	
		return;
	}

	switch (input_state) {
		case regular:
			if (input <= SAFE_BOUND_UPPER && input >= SAFE_BOUND_LOWER) {
				Setpoint = input;
			}
			break;
		case mode:
			switch(input) {
				case HOME_MODE:
					movement_state = home;
					input_state = regular;
					break;
				default:
					break;
			}
		default: //How did we get here?!
			input_state = regular;
			break;
	}
}
