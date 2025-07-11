#include <Wire.h>

#define I2C_SLAVE_ADRR 0x04;

//Define Pins
#define SENSOR1 4
#define SENSOR2 35
#define SENSOR3 25
#define SENSOR4 27
#define SENSOR5 14

#define MIN_INCREAMENT 10
#define MAX_INCREAMENT 15
#define MEASUREMENT_SIZE 10

#define defaultServoAngle = 75 ;
#define defaultLeftSpeed= 85 ;
#define defaultRightSpeed = 80;


struct sensorOutput{
int16_t sensorOutput1 = 0 ; 
int16_t sensorOutput2 = 0 ;
int16_t sensorOutput3 = 0 ;
int16_t sensorOutput4 = 0;
int16_t sensorOutput5 = 0 ;
};

  struct sensorOutput rawData[];
  struct sensorOutput sumData;
  struct sensorOutput filteredOutput;

//Initialise Default EEEBot Servo and Speed Values
int_16_t ServoAngle = 75 ;
int_16_t LeftSpeed= 85 ;
int_16_t RightSpeed = 80;



void setup() {
Serial.begin();

  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);

//Read individual outputs of photodiode devices from the sensor array through exploitation of the analogRead function
//Note that in case of analogue-enabled pins of the ESP32, a 12-bit ADC conversion is performed in order for the device to read received inputs signals (with values oscillating between 0 to 4095)
}

void loop() {
  //Run Sanity Check For Line Following Algorithm Implementation
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  //checkSensorsOperation();  //Uncomment to check output values

for(int i=0; i<=measurement_size; i++){
 rawData[i].sensorOutput1 = analogRead(SENSOR1); 
 rawData[i].sensorOutput2 = analogRead(SENSOR2);
 rawData[i].sensorOutput3 = analogRead(SENSOR3);
 rawData[i].sensorOutput4 = analogRead(SENSOR4);
 rawData[i].sensorOutput5 = analogRead(SENSOR5);
}
filterMeasurements();
}

 
void onRequest(){

  }

  while(){   //Right Sensor Detect White
      ServoAngle = defaultServoAngle ;
      LeftSpeed =  defaultLeftSpeed ;
      RightSpeed = defaultLeftSpeed;
  }

     while(){   //Auxilliary Left Sensor Detects Black
     for(int i = 0; i<=-MIN_INCREAMENT; i++){
      ServoAngle = defaultServoAngle + i;
      LeftSpeed =  defaultLeftSpeed + i;
      RightSpeed = defaultLeftSpeed - i; 
    }
  }

   while(){ //Lateral Right Sensor Detect Black
   for(int i = 0; i<=MAX_INCREAMENT; i++){
      ServoAngle = defaultServoAngle + i;
      LeftSpeed =  defaultLeftSpeed + i;
      RightSpeed = defaultLeftSpeed - i; 
    }
  }

  while(){ //Left Sensor Detect White
    for(int i = 0; i<=MIN_INCREAMENT; i++){
      ServoAngle = defaultServoAngle - i;
      LeftSpeed =  defaultLeftSpeed - i;
      RightSpeed = defaultLeftSpeed + i; 
    }
  }


    while(){ //Central Sensor and Right Sensor Detect White
    for(int i = 0; i<=MAX_INCREAMENT; i++){
      ServoAngle = defaultServoAngle + i;
      LeftSpeed =  defaultLeftSpeed + i;
      RightSpeed = defaultLeftSpeed - i; 
    }
  }

 
    Wire.write((byte)((rotationAxis ) & 0x000FF00 >> 8));
    Wire.write((byte)((rotationAxis ) & 0x000FF00));

    Wire.write((byte)((rotationAxis ) & 0x000FF00 >> 8));
    Wire.write((byte)((rotationAxis) & 0x000FF00));
  
    Wire.write((byte)((defaultRightSpeed) & 0x000FF00 >> 8));
    Wire.write((byte)((defaultRightSpeed) & 0x000FF00)); 

}

void filterMeasurements(){
  for(int i=0; i<=MEASUREMENTS_SIZE; i++){
    sumData.sensorOutput1 += rawData[i].sensorOutput1;
    sumData.sensorOutput2 += rawData[i].sensorOutput2;
    sumData.sensorOutput3 += rawData[i].sensorOutput3;
    sumData.sensorOutput4 += rawData[i].sensorOutput4;
    sumData.sensorOutput5 += rawData[i].sensorOutput5;
  }
  filteredOutput.sensorOutput1 = sumData.sensorOutput1 /MEASUREMENTS_SIZE;
  filteredOutput.sensorOutput2 = sumData.sensorOutput2 /MEASUREMENTS_SIZE;
  filteredOutput.sensorOutput3 = sumData.sensorOutput3 /MEASUREMENTS_SIZE;
  filteredOutput.sensorOutput4 = sumData.sensorOutput4 /MEASUREMENTS_SIZE;
  filteredOutput.sensorOutput5 = sumData.sensorOutput5 /MEASUREMENTS_SIZE;

}

 //Test output data for debugging
void checkSensorsOperation(){
Serial.print(photoD1Output);
Serial.println("Photodiode 1 Output");
Serial.print(photoD2Output);
Serial.println("Photodiode 2 Output");
Serial.print(photoD3Output);
Serial.println("Photodiode 3 Output");
Serial.print(photoD4Output);
Serial.println("Photodiode 4 Output");
Serial.print(photoD5Output);
Serial.println("Photodiode 5 Output");

}




