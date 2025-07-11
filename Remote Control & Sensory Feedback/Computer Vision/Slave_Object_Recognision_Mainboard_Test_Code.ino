//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*  UoN EEEBot                                          *//
//*                                                      *//
//*  EEEBot Firmware Code for the Mainboard ESP32        *//
//********************************************************//



#include <Wire.h>
#include <math.h>
#include <ESP32Encoder.h>
#include <ESP32Servo.h>

Servo steeringServo;

// create two encoder variable types
ESP32Encoder encoder1;
ESP32Encoder encoder2;

// create two signed encoder count variables of 16-bit size
int16_t enc1Count = 0;
int16_t enc2Count = 0;

float distData = 0;
  long oldPosition = -999;
  bool stoppedDueToDistance = false;


//int16_t distanceCalculation  = 


#define I2C_SLAVE_ADDR 0x22 // 4 in hexadecimal

// L298 motor driver pin definitions
#define enA 33  // enableA command line
#define enB 25  // enableB command line
#define INa 26  // channel A direction
#define INb 27  // channel A direction
#define INc 14  // channel B direction
#define INd 12  // channel B direction

#define countPerRevolution 13
#define diameterM 0.06

// setting PWM properties
const int freq = 2000;
const int ledChannela = 11;  // the ESP32 servo library uses the PWM channel 0 by default
const int ledChannelb = 12;
const int resolution = 8;
int servoPin = 13;  // the servo is attached to IO_13 on the ESP32
int reBoot = 1;
int16_t executionSignal = 0;


void setup() {
  // enable the weak pull up resistors for the two encoders
	ESP32Encoder::useInternalWeakPullResistors = puType::up;

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

  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);

  Wire.begin(I2C_SLAVE_ADDR);   // join i2c bus #4 - on the ESP32 the default I2C pins are 21 (SDA) and 22 (SCL)
  Wire.onReceive(onReceive);    // receive event
  Wire.onRequest(onRequest);    // request event
  
  Serial.begin(115200);             // start serial for the output
  Serial.println("ESP32 Running");  // sanity check
}

void loop() {
  // continuously 'get' the value from each encoder
  generateDistanceData();



  
}


// this function executes when data is requested from the master device
void onRequest(){
  // depending on the size of the encoder count value, you may need to make use of bits 32 to 17 to send larger values
  
  //Wire.write((byte)((enc1Count & 0xFF000000) >> 24)); // bits 32 to 25 of enc1Count
  //Wire.write((byte)((enc1Count & 0x00FF0000) >> 16)); // bits 24 to 17 of enc1Count
  Wire.write((byte)((enc1Count & 0x0000FF00) >> 8));    // first byte of enc1Count, containing bits 16 to 9
  Wire.write((byte)(enc1Count & 0x000000FF));           // second byte of enc1Count, containing the 8 LSB - bits 8 to 1

  //Wire.write((byte)((enc2Count & 0xFF000000) >> 24)); // bits 32 to 25 of enc2Count
  //Wire.write((byte)((enc2Count & 0x00FF0000) >> 16)); // bits 24 to 17 of enc2Count
  Wire.write((byte)((enc2Count & 0x0000FF00) >> 8));    // first byte of enc2Count, containing bits 16 to 9
  Wire.write((byte)(enc2Count & 0x000000FF));           // second byte of enc2Count, containing the 8 LSB - bits 8 to 1
}


// this function executes whenever data is received from the master device
void onReceive(int howMany){
  if(howMany != 2){  // for 3 16-bit numbers, the data will be 6 bytes long - anything else is an error
    Serial.println(howMany);
    emptyBuffer();
    return;
  }



  // set up variables for the three 16-bit values
  int16_t leftMotor_speed =0, rightMotor_speed = 0, servoAngle = 75;
  uint8_t executionSignal8_1 = Wire.read();  // receive bits 16 to 9 of x (one byte)
  uint8_t executionSignal16_9 = Wire.read();  // receive bits 16 to 9 of x (one byte)

  executionSignal = (executionSignal16_9 << 8) | executionSignal8_1;    // combine the two bytes into a 16 bit number
  Serial.print("\nExecution Signal: ");
  Serial.println(executionSignal);
  // verify that the correct values are received via the serial monitor
if(executionSignal ==0){
    leftMotor_speed = 0;
    rightMotor_speed = 0;
    servoAngle = 75;
  Serial.print("\nStop Motor");
}



Serial.print("\nExecution Signal: ");
Serial.println(executionSignal);




  if(executionSignal == 1){

    leftMotor_speed = 167;
    rightMotor_speed = 170;
    servoAngle = 75;
    Serial.print("\nGo Forward");
  }


    if(executionSignal == 3){


    servoAngle = 30;
    Serial.print("\nTurn Left");
  }

    if(executionSignal == 2){
    servoAngle = 130;
    Serial.print("\nTurn Right");
  }

  setSteeringAngle(servoAngle);
  runMotors(leftMotor_speed, rightMotor_speed);

    // verify that the correct values are received via the serial monitor
  Serial.print("Left Motor: ");
  Serial.print(leftMotor_speed);
  Serial.print("\t");
  Serial.print("Right Motor: ");
  Serial.print(rightMotor_speed);
  Serial.print("\t");
  Serial.print("Servo: ");
  Serial.println(servoAngle);
}



// function to clear the I2C buffer
void emptyBuffer(void){
  Serial.println("Error: I2C Byte Size Mismatch");
  while(Wire.available())
  {
    Wire.read();
  }
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

  if (leftMotor_speed ==  0 && rightMotor_speed ==0) {
    digitalWrite(INa, LOW);
    digitalWrite(INb, LOW);
    digitalWrite(INc, LOW);
    digitalWrite(INd, LOW);
  }

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

void generateDistanceData() //Calculate and monitor the distance change
{
  long newPosition = encoder1.getCount();
  // check if encoder has moved
  if (newPosition != oldPosition) 
  {
    oldPosition = newPosition;
    //Calculate the distance moved in meters
    
    distData += (diameterM*M_PI)/(countPerRevolution); //The encoder does count 44 spins per revolutions

    // output distance to the serial monitor                    
    Serial.print("DistanceM:");
    Serial.println(distData);

     }

}
