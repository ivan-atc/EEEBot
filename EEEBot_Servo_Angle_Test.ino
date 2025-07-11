//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*  UoN EEEBot                                          *//
//*                                                      *//
//*  Servo Angle Test Code                               *// 
//********************************************************//

// use this code to centre the servo on your EEEBot
// the comments in the code are in place to help you understand the code - once understood you can take parts of the code and use it in your own solutions

#include <ESP32Servo.h>

Servo steeringServo;

int steeringAngle = 75;    // variable to store the servo position
int servoPin = 13;  // the servo is attached to IO_13 on the ESP32

void setup() {
	// allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	steeringServo.setPeriodHertz(50);    // standard 50Hz servo
	steeringServo.attach(servoPin, 1000, 2000);   // attaches the servo to the pin using the default min/max pulse widths of 1000us and 2000us
  
  steeringServo.write(steeringAngle);
}

void loop() {
  // if you wish to test different servo angles, you will need to reupload the code once the steeringAngle value has been changed
}
