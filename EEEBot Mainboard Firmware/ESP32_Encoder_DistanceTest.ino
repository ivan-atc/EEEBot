//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical & Electronic Engineering   *//
//*                                                      *//
//*  UoN Welcome Week                                    *//
//*  Design Task: Digital Trundle Wheel                  *//
//********************************************************//

// include encoder library
#include <ESP32Encoder.h>

ESP32Encoder myEnc;//Initialise Encoder

// change these two numbers (if needed) to enable pins connected to encoder

long oldPosition  = 0;
float distance;

void setup() 
{
  myEnc.attachHalfQuad(34, 35); //enable pins with interrupt capability
  myEnc.clearCount();//Clear Count
  
  Serial.begin(9600);   // initialise serial communication
  Serial.println("ESP32 Running"); // sanity check
}

void loop() 
{
  
  long newPosition = myEnc.getCount();
  
  // check if encoder has moved
  if (newPosition != oldPosition) 
  {
    oldPosition = newPosition;
    
  // edit the code below to calculate the distance moved, +1 increment = (diameter*pi)/encoder count per revolution
    distance = newPosition +distance;
  // ***
    
    // output distance to the serial monitor (use serial monitor to input distance)                 
    Serial.print("Encoders Counts: ");
    Serial.println(distance);
    }
}
