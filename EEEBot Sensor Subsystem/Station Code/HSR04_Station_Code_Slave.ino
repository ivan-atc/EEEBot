#include <Wire.h>
#include <ESP32Encoder.h>
#include <ESP32Servo.h>

#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal

// L298 motor driver pin definitions
#define enA 33  // enableA command line
#define enB 25  // enableB command line
#define INa 26  // channel A direction
#define INb 27  // channel A direction
#define INc 14  // channel B direction
#define INd 12  // channel B direction

ESP32Encoder leftEncoder, rightEncoder;

// setting PWM properties
const int freq = 2000;
const int ledChannel = 11;  // the ESP32 servo library uses the PWM channel 0 by default
const int resolution = 8;


const int leftSpeed = 80;
const int rightSpeed= 85;

int16_t distanceData = 0;
int16_t stopSignal = 0 ;

void setup() {
  // put your setup code here, to run once:
  // attach the relevant pins to each encoder
  leftEncoder.attachHalfQuad(34, 35);
  rightEncoder.attachHalfQuad(36, 39);

  
  // configure the LED PWM functionalitites and attach the GPIO to be controlled - ensure that this is done before the servo channel is attached
  ledcAttachChannel(enA, freq, resolution, ledChannel);

  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);

  Wire.begin(I2C_SLAVE_ADDR);   // join i2c bus #4 - on the ESP32 the default I2C pins are 21 (SDA) and 22 (SCL)
  Wire.onReceive(onReceive);    // receive event
  
  Serial.begin(115200);             // start serial for the output
  Serial.println("ESP32 Running");  // sanity check

}

void loop() {
  uint8_t distanceData16_9 = Wire.read();
  uint8_t distanceData8_1 = Wire.read();

  uint8_t stopSignal16_9 = Wire.read();
  uint8_t stopSignal8_1 = Wire.read();

  distanceData = (distanceData16_9 <<8) | distanceData8_1;
  stopSignal = (stopSignal16_9 << 8) | stopSignal8_1;



  do{
    goForwards();
    motors(leftSpeed, rightSpeed);
  }while(stopSignal == 0);

  stopMotors();

}

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

void motors(int leftSpeed, int rightSpeed) {
  // set individual motor speed
  // the direction is set separately

  // vary the motor speeds - use the absolute value to remove the negative
  ledcWrite(enA, abs(leftSpeed));
  ledcWrite(enB, abs(rightSpeed));

  // constrain the values to within the allowable range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  delay(25);
}

void onReceive(int howMany)
{
  if(howMany != 2)  // for 1 16-bit numbers, the data will be 2 bytes long - anything else is an error
  {
    emptyBuffer();
    return;
  }
}

// function to clear the I2C buffer
void emptyBuffer(void)
{
  Serial.println("Error: I2C Byte Size Mismatch");
  while(Wire.available())
  {
    Wire.read();
  }
  
}

//this function will variates the brightness of the LED based on distance information sended by the master 
void activateLED() {
    if (distanceData < 10) {
        ledcWrite(ledChannel, 255); // Maximum brightness
    } else if (distanceData < 100) {
        int brightness = map(distanceData, 10, 100, 255, 50);
        ledcWrite(ledChannel, brightness); // Scale brightness
    } else {
        ledcWrite(ledChannel, 0); // Turn LED OFF
    }
}

