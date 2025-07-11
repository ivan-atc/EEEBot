#include <HCSR04.h>
#include <ESP32Encoder.h>
#include <math.h>

#define trigg 19//Define Trigger and Echo dedicated GPIO Pins
#define echo 34
#define LED1 13 //LED Channel Line
const int SDA_PIN = 21;
const int SCL_PIN = 22;



const int measurementSize = 10; //Define desired numbers of measurement to be perform and collected
static float sound_speed = 0.034; //Define a constant sound speed 

float measurementsArray[measurementSize]; //Define Array for measurement collection during sensor operation
float distanceSum =0;
float filterDistance=0; //Variable for filtered distance collection

HCSR04 hc(echo, trigg); //initialisation of HCSR04 (trig pin , echo pin)


void setup() //
{
  Serial.begin(9600); // Starts the serial communication
  pinMode(trigg, OUTPUT); // Sets the trigPin as an Output
  pinMode(echo, INPUT); // Sets the echoPin as an Input
  pinMode(LED1, OUTPUT); //Set LED1 Pin as an Output
}

void loop(){ //Loop function to be executed

 readPulse();
 filterMeasurements();
 activateLED();


}





//External ReadPulse Function for execution of a number of distance measurement based on the measurement size value defined on the upper code
void readPulse(){

  for(int i=0; i<=measurementSize; i++){
    digitalWrite(trigg, HIGH); //Activates the trigger pin to emmits a soundwave
   
  delayMicroseconds(10);
    // Reads the echoPin, returns the sound wave travel time in microseconds

  digitalWrite(trigg, LOW); //Deactivate the trigger pin after having receving a sound wave value

  float duration = pulseIn(echo, HIGH); //Set the duration to be the value returned to the ECHO Pin, which would be used to calculate the distance
  Serial.print("Pulse Duration(ms):");


  measurementsArray[i] = ( duration* sound_speed)/2; //Collection of repeated measurements into an array
  

}
}
//External Function for modification of the LED brightness based on the EEEBot Position calculated by the sensor 
void activateLED(){

    float oldPosition =999; //Definition of an old and new position variables
    float newPosition;
    
    for(int i = 0 ; i<1000;i++){
    newPosition = filterDistance;

    if((newPosition != oldPosition) && (newPosition > oldPosition)){
  oldPosition = newPosition;
      while(newPosition <10){
  analogWrite(LED1, 0); 
}
   if (10<newPosition<100);
   analogWrite(LED1, i);
}
  while(newPosition>100){
  analogWrite(LED1, 255); 
}
    }
}

//External function for filtering measurements values collected through the measurement array based on the average
void filterMeasurements(){
  
 for (int i=0; i<measurementSize; i++){
  float distanceSum = distanceSum + measurementsArray[i];
 }

filterDistance = distanceSum/measurementSize;


  Serial.print("Distance(cm):"); 
  Serial.println(filterDistance);
  delayMicroseconds(50000);
}


