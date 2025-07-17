#include <Wire.h>
#include <HCSR04.h>


#define trigg 19 //Define Trigger and Echo dedicated GPIO Pins
#define echo 34



#define I2C_SLAVE_ADRR 0x04 //Define I2C adress among which communication between the master and the slave would be done
#define MEASUREMENT_SIZE 10 //Define the number of measurements data to be collected from the ultrasonic sensor for filtering output
#define SOUND_SPEED 0.034//Define Sound Speed Value in cm/s
#define LED1 13 //LED Channel Line

HCSR04 hc(echo, trigg); //initialisation of HCSR04 (trig pin , echo pin)

float measurementsArray[MEASUREMENT_SIZE]; //Define Array for measurement collection during sensor operation
float filterDistance; //Variable for filtered distance collection
float distanceSum=0;

int16_t stopSignal = 0; //Definition of a 16-bit stop signal dedicated to send a signal value to the slave ESP32 in function of the calculated distance 

// setting PWM properties
const int freq = 2000;
const int ledChannel = 11;  // the ESP32 servo library uses the PWM channel 0 by default
const int resolution = 8;

void setup() { 
  Serial.begin(9600); //Start Serial Monitor 
  Wire.begin();
  Wire.onRequest(onRequest); 
  pinMode(echo, INPUT); //Set Echo Pin as Input
  pinMode(trigg, OUTPUT);//Set Echo Pin as Input
  pinMode(LED1, OUTPUT); //Set LED1 Pin as an Output
}


void loop() {
   Wire.beginTransmission(I2C_SLAVE_ADRR); //Begin Serial Communication at the Slave Adress 
   readPulse(); 
   filterMeasurements();
}

//External Function Dedicated for performing distance calculation between the HSR04 sensor and an obstacle based on the travel duration of an an emitted soundwave
void readPulse(){ 

  for(int i=0; i<=MEASUREMENT_SIZE; i++){
    digitalWrite(trigg, HIGH); //Activates the trigger pin to emmits a soundwave
   
  delayMicroseconds(10);
    // Reads the echoPin, returns the sound wave travel time in microseconds

  digitalWrite(trigg, LOW); //Deactivate the trigger pin after having receving a sound wave value

  float duration = pulseIn(echo, HIGH); //Set the duration to be the value returned to the ECHO Pin, which would be used to calculate the distance
  //Serial.print("Pulse Duration(ms):");
  //Serial.println(duration);

  measurementsArray[i] = ( duration * SOUND_SPEED)/2; //Collection of successive measurements into an array
  //Serial.print("Distance cm:");
  //Serial.println( measurementsArray[i] );


}
}

//External Function dedicated for filter performed measurements
void filterMeasurements() {
    distanceSum = 0; // Reset sum before calculation
    for (int i = 0; i < MEASUREMENT_SIZE; i++) {
        distanceSum += measurementsArray[i];
    }
    filterDistance = distanceSum / MEASUREMENT_SIZE;
}

void onRequest(){

     while(filterDistance>10){
      Wire.write((byte)(stopSignal & 0x0000FF00 )>>8);
      Wire.write((byte)(stopSignal & 0x00FF0000));
    }

    while(filterDistance<=10){
      stopSignal =1; 
      Wire.write((byte)(stopSignal & 0x0000FF00 )>>8);
      Wire.write((byte)(stopSignal & 0x00FF0000));
    }

    int16_t scaledDistance = (int16_t)filterDistance;
    
    Wire.write((scaledDistance >> 8) & 0xFF);  // Send the high byte
    Wire.write(scaledDistance & 0xFF);        // Send the low byte
}
