#include <PID_v1.h>
#include <Encoder.h>

#define RPWM 5
#define LPWM 6
#define L_EN 7
#define R_EN 8

#define ENC_A 12
#define ENC_B 13

#define MIN_PWM 0

double Setpoint, PID_In, PID_Out;
//double Kp=0.2, Ki=0, Kd=0.008; //100
double Kp=0.1, Ki=0, Kd=0.008; //100

//PID reader
PID pid(&PID_In, &PID_Out, &Setpoint, Kp, Ki, Kd, DIRECT);

//Quadrature encoder reader
Encoder quad(ENC_A, ENC_B);

void read_one_sec() {
	for (int i = 0; i < 1000; i++) {
		delay(1);
		quad.read();
	}
	Serial.println(quad.read());		
}

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

void setup() {
	Serial.begin(115200);
	quad.write(0);
	for (int i = 5; i < 9; i++) {
		pinMode(i, OUTPUT);
		digitalWrite(i, LOW);
	}
	digitalWrite(R_EN, HIGH);
	digitalWrite(L_EN, HIGH);
	pid.SetMode(AUTOMATIC);
	pid.SetOutputLimits(-150, 150);
	pid.SetSampleTime(1);

	Setpoint = 10000;
}
int frame = 0;
void loop() {
	PID_In = quad.read();
	pid.Compute();
	/*
	if (frame % 1000 == 0) {
		Serial.print("Encoder: ");
		Serial.print(PID_In);
		Serial.print(" Pid Output: ");
		Serial.print(PID_Out);
		Serial.println();
	}
	*/
	multi_direction(PID_Out);
	frame++;
}

//TODO: Add interrupt for limit switch
