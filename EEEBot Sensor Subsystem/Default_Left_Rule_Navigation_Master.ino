#include <Wire.h>
#include <HCSR04.h>
#include <Adafruit_MPU6050.h>


#define trigg 19 //Define Trigger and Echo dedicated GPIO Pins
#define echo 34



#define I2C_SLAVE_ADRR 0x04 //Define I2C adress among which communication between the master and the slave would be done
#define MEASUREMENT_SIZE 10 //Define the number of measurements data to be collected from the ultrasonic sensor for filtering output
#define SOUND_SPEED 0.034//Define Sound Speed Value in cm/s
#define LED1 13 //LED Channel Line

HCSR04 hc(echo, trigg); //initialisation of HCSR04 (trig pin , echo pin)

float distanceArray[MEASUREMENT_SIZE]; //Define Array for measurement collection during sensor operation
float gyroArray[MEASUREMENT_SIZE]; //Define Array for measurement collection during sensor operation

float obstacleDistance; //Variable for filtered distance collection
int16_t stopSignal = 0; //Definition of a 16-bit stop signal dedicated to send a signal value to the slave ESP32 in function of the calculated distance 

sensors_event_t a, g;
int16_t botRotation = 0; 


void setup() { 
  Serial.begin(9600); //Start Serial Monitor 
  Wire.begin();
  Wire.onRequest(onRequest); 
  pinMode(echo, INPUT); //Set Echo Pin as Input
  pinMode(trigg, OUTPUT);//Set Echo Pin as Input
}

void loop() {
  stopSignal = 0;
   
   Wire.beginTransmission(I2C_SLAVE_ADRR); //Begin Serial Communication at the Slave Adress 
   readPulse(); 
   obstacleDistance = filterMeasurements(distanceArray);
   botRotation = filterMeasurements(gyroArray);
  
}

/
void onRequest(){
    if(obstacleDistance>10){  //Initialise Servo Angle to default angle (i.e 70 degrees)
      botRotation = 70; //Servo Set at Parallel Position
    } 
    else  //Default Left Turn
      stopSignal =1;
      delay(1500);
      botRotation = 160;
      stopSignal = 0;
    }

    botRotation = 70;
    
    while(obstacleDistance<=10){ //Constrain the Bot to prioritize 180 rotation and right turn in case the left path is obstructed
    
    stopSignal = 1; //180 Degrees Rotation
    delay(1500);
    botRotation = -20;
    stopSignal = 0;
      
      if(obstacleDistance<=10){ //Clockwise Rotation (i.e Right Turn)
        stopSignal = 1;
        delay(1500);
        botRotation = -20;
        stopSignal = 0;
      }
    }
      Wire.write((byte)(stopSignal & 0x0000FF00 )>>8);
      Wire.write((byte)(stopSignal & 0x00FF0000));
      Wire.write((byte)(botRotation & 0x0000FF00 )>>8);
      Wire.write((byte)(botRotation & 0x00FF0000));

}
}


//External Function Dedicated for performing distance calculation between the HSR04 sensor and an obstacle based on the travel duration of an an emitted soundwave
void readPulse(){ 
  float distanceSum=0;
  for(int i=0; i<=MEASUREMENT_SIZE; i++){
    digitalWrite(trigg, HIGH); //Activates the trigger pin to emmits a soundwave
   
  delayMicroseconds(10);
    // Reads the echoPin, returns the sound wave travel time in microseconds

  digitalWrite(trigg, LOW); //Deactivate the trigger pin after having receving a sound wave value

  float duration = pulseIn(echo, HIGH); //Set the duration to be the value returned to the ECHO Pin, which would be used to calculate the distance
  //Serial.print("Pulse Duration(ms):");
  //Serial.println(duration);

  measurementsArray[i] = (duration * SOUND_SPEED)/2; //Collection of successive measurements into an array
  //Serial.print("Distance cm:");
  //Serial.println( measurementsArray[i] );

}
}

void readGyro(){ 
float lastTime=0;
  for(int i=0; i<=MEASUREMENT_SIZE; i++){
      device.getEvent(&a, &g);
      float currentTime = millis();
      float dt = (currentTime - lastTime) / 1000.0;
      lastTime = currentTime;
      botRotation += g.gyro.z*dt* (180.0 / PI); //For LED Rotation Detection, only the Yaw-Axis was necessary

  measurementsArray[i] = (duration * SOUND_SPEED)/2; //Collection of successive measurements into an array
  //Serial.print("Distance cm:");
  //Serial.println( measurementsArray[i] );

}

//External Function dedicated for filter performed measurements
int_16_t filterMeasurements(int parameter[]) {
    distanceSum = 0; // Reset sum before calculation
    for (int i = 0; i < MEASUREMENT_SIZE; i++) {
        distanceSum += measurementsArray[i];
    }
    filterDistance = distanceSum / MEASUREMENT_SIZE;
}

