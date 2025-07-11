#include <ESP32Encoder.h>
#include <ESP32Servo.h>
#include <Wire.h>

Servo steeringServo;

// create two encoder variable types
ESP32Encoder encoder1;
ESP32Encoder encoder2;

// create two signed encoder count variables of 16-bit size
int16_t enc1Count = 0;
int16_t enc2Count = 0;

#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal

// L298 motor driver pin definitions
#define enA 33  // enableA command line
#define enB 25  // enableB command line
#define INa 26  // channel A direction
#define INb 27  // channel A direction
#define INc 14  // channel B direction
#define INd 12  // channel B direction

// setting PWM properties
const int freq = 2000;
const int ledChannela = 11;  // the ESP32 servo library uses the PWM channel 0 by default
const int ledChannelb = 12;
const int resolution = 8;



  int16_t leftSpeedIncr = 0;
  int16_t rightSpeedIncr = 0;
  int16_t servoAngleIncr = 0;
    int16_t leftMotor_speed = 0;
  int16_t rightMotor_speed = 0;
  int16_t servoAngle = 75;


int servoPin = 13;  // the servo is attached to IO_13 on the ESP32
void setup() {
  Serial.begin(115200); // Start Serial Monitor
  // put your setup code here, to run once:
  // attach the relevant pins to each encoder
  encoder1.attachHalfQuad(34, 35);
  encoder2.attachHalfQuad(36, 39);

  // set the count of both encoders to 0
  encoder1.setCount(0);
  encoder2.setCount(0);

  // configure the LED PWM functionalitites and attach the GPIO to be controlled - ensure that this is done before the servo channel is attached
  ledcAttachChannel(enA, freq, resolution, ledChannela);
  ledcAttachChannel(enB, freq, resolution, ledChannelb);

  // allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	steeringServo.setPeriodHertz(50);    // standard 50Hz servo
	steeringServo.attach(servoPin, 500, 2400);   // attaches the servo to the pin using the default min/max pulse widths of 1000us and 2000us

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);
  
  Wire.begin(I2C_SLAVE_ADDR);   // join i2c bus #4 - on the ESP32 the default I2C pins are 21 (SDA) and 22 (SCL)
	Wire.onReceive(onReceive);    // register an event handler for received data
}

void loop() {

if (!(Wire.begin(I2C_SLAVE_ADDR))){
  Serial.println("I2C Connection Failure");
}
else{
  Serial.println("Connection to the Master Success");
}
  runMotors(leftMotor_speed, rightMotor_speed);  



}



void onReceive(int howMany){
  if(howMany != 6){  // for 3 16-bit numbers, the data will be 6 bytes long - anything else is an error
    emptyBuffer();
    return;
  }

 //Define Default at 90 degrees
  // set up variables for the three 16-bit values

  
 // uint8_t leftSpeed_Incr16_9 = Wire.read();  // receive bits 16 to 9 of x (one byte)
 // uint8_t leftSpeed_Incr8_1 = Wire.read();   // receive bits 8 to 1 of x (one byte)
  //uint8_t rightSpeed_Incr16_9 = Wire.read(); // receive bits 16 to 9 of y (one byte)
  //uint8_t rightSpeed_Incr8_1 = Wire.read();  // receive bits 8 to 1 of y (one byte)
  //uint8_t servoAngleIncr16_9 = Wire.read();       // receive bits 16 to 9 of z (one byte)




 // leftSpeedIncr = (leftSpeed_Incr16_9 << 8) | leftSpeed_Incr8_1;    // combine the two bytes into a 16 bit number
  //rightSpeedIncr = (rightSpeed_Incr16_9 << 8) | rightSpeed_Incr8_1; // combine the two bytes into a 16 bit number
 // servoAngleIncr = (servoAngleIncr16_9 << 8) | servoAngleIncr8_1;
   uint8_t leftMotor_speed16_9 = Wire.read();  // receive bits 16 to 9 of x (one byte)
  uint8_t leftMotor_speed8_1 = Wire.read();   // receive bits 8 to 1 of x (one byte)
  uint8_t rightMotor_speed16_9 = Wire.read(); // receive bits 16 to 9 of y (one byte)
  uint8_t rightMotor_speed8_1 = Wire.read();  // receive bits 8 to 1 of y (one byte)
  uint8_t servoAngle16_9 = Wire.read();       // receive bits 16 to 9 of z (one byte)
  uint8_t servoAngle8_1 = Wire.read();        // receive bits 8 to 1 of z (one byte)


  leftMotor_speed = (leftMotor_speed16_9 << 8) | leftMotor_speed8_1;    // combine the two bytes into a 16 bit number
  rightMotor_speed = (rightMotor_speed16_9 << 8) | rightMotor_speed8_1; // combine the two bytes into a 16 bit number
  servoAngle = (servoAngle16_9 << 8) | servoAngle8_1;     
  

  setSteeringAngle(servoAngle);
   
                     // combine the two bytes into a 16 bit number

//leftMotor_speed = leftMotor_speed + leftSpeedIncr;
//rightMotor_speed = rightMotor_speed + rightSpeedIncr;
//servoAngle = servoAngle + servoAngleIncr;



  // Debug Values


}

// function to run the motors - you may need to modify the HIGH/LOW states to get each wheel to rotate in the desired direction
void runMotors(int leftMotor_speed, int rightMotor_speed){
  // limit the speed value between -255 and 255 as the PWM value can only be between 0 and 255 - the negative is handled below
  leftMotor_speed = constrain(leftMotor_speed, -255, 255);
  rightMotor_speed = constrain(rightMotor_speed, -255, 255);

  // vary the motor speeds - use the absolute value to remove the negative
  ledcWrite(enA, abs(leftMotor_speed));
  ledcWrite(enB, abs(rightMotor_speed));

  // if the speed value is negative, run the motor backwards

  if (leftMotor_speed < 0) {
    digitalWrite(INa, LOW);
    digitalWrite(INb, HIGH);
  }
  // else, run the motor forwards
  else {
    digitalWrite(INa, HIGH);
    digitalWrite(INb, LOW);    
  }

  // if the speed value is negative, run the motor backwards
  if (rightMotor_speed < 0) {
    digitalWrite(INc, LOW);
    digitalWrite(INd, HIGH);
  }
  // else run the motor forwards
  else {
    digitalWrite(INc, HIGH);
    digitalWrite(INd, LOW);    
  }
}

  void setSteeringAngle(int16_t servoAngle){
    steeringServo.write(servoAngle);
  }

// function to clear the I2C buffer
void emptyBuffer(void){
  Serial.println("Error: I2C Byte Size Mismatch");
  while(Wire.available())
  {
    Wire.read();
  }
}


