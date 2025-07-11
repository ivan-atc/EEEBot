#include <Wire.h>
#include <ESP32Encoder.h>
#include <ESP32Servo.h>

#define I2C_SLAVE_ADRR 0x04
// L298 motor driver pin definitions
#define enA 33  // enableA command line
#define enB 25  // enableB command line
#define INa 26  // channel A direction
#define INb 27  // channel A direction
#define INc 14  // channel B direction
#define INd 12  // channel B direction#

//Define Encoders Variable
ESP32 encoder1, encoder2;

// setting PWM properties
const int freq = 2000;
const int ledChannela = 11;  // the ESP32 servo library uses the PWM channel 0 by default
const int ledChannelb = 12;
const int resolution = 8;

int16_t leftMotor_speed = 0;
int16_t rightMotor_speed = 0;
int16_t servoAngle = 0; 

int servoPin = 13;  // the servo is attached to IO_13 on the ESP32

//Make sure I2C communication is successfully done
void setup(){

   // enable the weak pull up resistors for the two encoders
	ESP32Encoder::useInternalWeakPullResistors = puType::up;

  // attach the relevant pins to each encoder
  encoder1.attachHalfQuad(34, 35);
  encoder2.attachHalfQuad(36, 39);
  cleanEncoders();
  Serial.begin()
  
  steeringServo.setPeriodHertz(50);    // standard 50Hz servo
	steeringServo.attach(servoPin, 500, 2400);   // attaches the servo to the pin using the default min/max pulse widths of 1000us and 2000us

  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);

  Wire.begin(I2C_SLAVE_ADRR); //Begin I2C Serial Communication
  Wire.onReceive(onReceive); // Event to be received
  Wire.onRequest(onRequest); //Request Event from the Master


}


void loop(){

  uint8_t leftMotor16_9 = (uint8_t)Wire.read; //Read Left Motor Values Send by the Master (in bytes)
  uint8_t leftMotor8_1 = (uint8_t)Wire.read;
  uint8_t rightMotor16_9 =(uint8_t) Wire.read;//Read Right Motor Values Send by the Master (in bytes)
  uint8_t rightMotor8_1 = (uint8_t)Wire.read;
  uint8_t servoAngle16_9 = (uint8_t)Wire.read;//Read Servo Angle Values Send by the Master (in bytes)
  uint8_t servoAngle8_1 = (uint8_t)Wire.read;

  lefttMotor_speed = (leftMotor16_9<< 8) | leftMotor_speed8_1;
  rightMotor_speed = (rightMotor16_9 << 8) | righMotor_speed8_1;
  servoAngle = (servoAngle16_9 << 8) | servoAngle8_1;
  
  goForwards();
  setSteeringAngle (servoAngle);
  runMotors (leftMotor_speed, rightMotor_speed);

}

void cleanEncoders(){
  // set the count of both encoders to 0
  encoder1.setCount(0);
  encoder2.setCount(0);
}

//Review Negative Motor Execution
void goForwards(int leftMotor_speed, int rightMotor_speed){
  digitalWrite(INa, HIGH);
  digitalWrite(INb, HIGH);
  digitalWrite(INa, HIGH);
  digitalWrite(INb, HIGH);
    
}

// function to set the steering angle
void setSteeringAngle(int steeringAngle){
  steeringServo.write(steeringAngle);
}

// function to run the motors - you may need to modify the HIGH/LOW states to get each wheel to rotate in the desired direction
void runMotors(int leftMotor_speed, int rightMotor_speed){
  // limit the speed value between -255 and 255 as the PWM value can only be between 0 and 255 - the negative is handled below
  leftMotor_speed = constrain(leftMotor_speed, -255, 255);
  rightMotor_speed = constrain(rightMotor_speed, -255, 255);
}


void onReceive(int howMany)
{
  if(howMany != 4)  // for 2 16-bit numbers, the data will be 4 bytes long - anything else is an error
  {
    emptyBuffer();
    return;
  }

}


