//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*  UoN EEEBot                                          *//
//*                                                      *//
//*  Motor & Servo Basic Test Code                       *//
//********************************************************//

// ASSUMPTION: Channel A is LEFT, Channel B is RIGHT

// use this code to correctly assign the four pins to move the car forwards and backwards
// you first need to change the pin numbers for the four motor input 'IN' pins and two enable 'en' pins below and then 
// decide which go HIGH and LOW in each of the movements, stopMotors has been done for you
// ** marks where you need to insert the pin number or state

// feel free to modify this code to test existing or new functions

#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <math.h>

Servo steeringServo;
ESP32Encoder encoder1;



#define enA 33  // enableA command line
#define enB 25  // enableB command line

#define INa 26  // channel A direction
#define INb 27  // channel A direction
#define INc 14  // channel B direction
#define INd 12  // channel B direction
 



// setting PWM properties
const int freq = 2000;
const int ledChannela = 11;  // the ESP32 servo library uses the PWM channel 0 by default, hence the motor channels start from 1
const int ledChannelb = 12;
const int resolution = 8;

int steeringAngle = 75;    // variable to store the servo position (50 degrees being the position where wheels are aligned)
int servoPin = 13;  // the servo is attached to IO_13 on the ESP32


long oldPosition  = -999; 
float distance=0;



void setup() {
  // configure the LED PWM functionalitites and attach the GPIO to be controlled - ensure that this is done before the servo channel is attached
  // configure the LED PWM functionalitites and attach the GPIO to be controlled - ensure that this is done before the servo channel is attached

  // attach the relevant pins to each encoder
  encoder1.attachHalfQuad(34, 35);

  // set the count of both encoders to 0
  encoder1.setCount(0);

 


	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	steeringServo.setPeriodHertz(50);    // standard 50Hz servo
	steeringServo.attach(servoPin, 500, 2400);   // attaches the servo to the pin using the default min/max pulse widths of 1000us and 2000us

  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);

  // initialise serial communication
  Serial.begin(9600);
  Serial.println("ESP32 Running");  // sanity check

  //Reset the Encoder Count by default
  encoder1.clearCount();
  delay(100); //Wait before starting
}


void loop() {
  // this code rotates the steering between 0 and 180 degrees before driving both wheels forwards and backwards followed by rotating the EEEBot clockwise and anticlockwise for 3 seconds each time

  //set the speed of the motors - minimum speed of 0, maximum speed of 255 i.e. largest value for an 8-bit PWM
  int leftSpeed = 255;
  int rightSpeed = 255;

  do{ 
  
  race_dist();                            //Move forwards and make the Servo rotatrd for until the distance is superior to 10m
  goForwards();
 maintainSteering();
  motors(leftSpeed, rightSpeed);
  delay(100);
  
  } while(distance<=10);
  stopMotors();
}

void motors(int leftSpeed, int rightSpeed) {
  // set individual motor speed
  // the direction is set separately

  // constrain the values to within the allowable range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  ledcWrite(enA, leftSpeed);
  ledcWrite(enB, rightSpeed);
  delay(25);
}

//Directions setups (Only Forward and Stop Functions required for 10m challenge)
void goForwards() {
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
}

void stopMotors() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, LOW);
}


void maintainSteering() {
                                                                        //Function that change the Servo Angle whuch will be called on the loop function for adding movements to the Servo
  for (steeringAngle = 75; steeringAngle <= 93; steeringAngle += 1) {   //So that it adds some control to the vehicule rather than having only the motor wheels to do so
		steeringServo.write(steeringAngle);                                 // (Otherwise, the vehicule direction is not controlled)
                                                                        //10 degrees of contimious rotation  is an sufficient value
		delay(30);                                                          
	}

	for (steeringAngle = 93; steeringAngle >=75; steeringAngle -= 1) {   
		steeringServo.write(steeringAngle);                                 
		delay(50);                                                          
	} 
}

void race_dist() //Calculate and monitor the distance change
{
  long newPosition = encoder1.getCount();
  
  // check if encoder has moved
  if (newPosition != oldPosition) 
  {
    oldPosition = newPosition;
    //Calculate the distance moved in meters
    
    distance = (0.06*M_PI)/(44)+distance; //The encoder does count 44 spins per revolutions
    
    // output distance to the serial monitor                    
    Serial.print("Distance(m): ");
    Serial.println(distance);

     }

}





