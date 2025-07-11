#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


#define measurementSize 10 //Define Measuremement Size for code execution
#define LED 13 //Define the PIN of the LED
#define SCL 22
#define SDA 21
Adafruit_MPU6050 device; //Set the MPU Device

struct MPUaxis { //Declaration of a structure for definition of the MPU axis value into similar structure
float ax =0, ay=0, az=0; 
float gx=0, gy=0, gz=0;
};

sensors_event_t a, g;

const float NEGATIVE_ROTATION_THRESHOLD = -90;
const float POSITIVE_ROTATION_THRESHOLD = 90;

float filteredYaw = 0; 
float lastTime=0;
bool wasRotated = false;


struct MPUaxis summedOutput; //Declare MPU structure dedicated for making the sum of each recolted outputs, which have been stored into specific arrays
struct MPUaxis filteredOutput; //Declare an MPUaxis structure dedicated for filtering output values based which would be perform through average value


void setup(){  
Serial.begin(9600);
pinMode(LED, OUTPUT); //Set LED as an output
Wire.begin(SDA, SCL);  //Begin Serial Communication in I2C
digitalWrite(LED, LOW);

}


void loop(){

  getOutput();
  filterEvents();

  float currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  filteredYaw += filteredOutput.gz*dt* (180.0 / PI); //For LED Rotation Detection, only the Yaw-Axis was necessary
  //filteredRow += filteredOutput.gx*dt*(180.0/PI);   //For EEEBot Rotation Detection the Row and Pitch Angle are unneccesary 
  //filteredPitch += filteredOutput.gy*dt*(180.0/PI);


  if (filteredYaw >= 180) filteredYaw -= 360;
  if (filteredYaw < -180) filteredYaw += 360;

 
  if ((filteredYaw <= NEGATIVE_ROTATION_THRESHOLD || filteredYaw >= POSITIVE_ROTATION_THRESHOLD) && !wasRotated) {
    digitalWrite(LED, HIGH);
    wasRotated = true;
  } else if (filteredYaw > NEGATIVE_ROTATION_THRESHOLD && filteredYaw < POSITIVE_ROTATION_THRESHOLD && wasRotated) {
    digitalWrite(LED, LOW);
    wasRotated = false;
  }

  Serial.print("Yaw-Axis Angle:"); //Print retruned results onto the serial plotter
  Serial.println(filteredYaw);

  Serial.print("ax:"); //Print retruned results onto the serial plotter
  Serial.println(filteredOutput.ax);
  Serial.print("ay:");
  Serial.println(filteredOutput.ay);
  Serial.print("az:");
  Serial.println(filteredOutput.az);

  Serial.print("gx:");
  Serial.println(filteredOutput.gx);
  Serial.print("gy:");
  Serial.println(filteredOutput.gy);
  Serial.print("gz:");;
  Serial.println(filteredOutput.gz);
  delay(3000);
}

void getOutput(){
  for(int i=0; i<measurementSize; i++){
    device.getEvent(&a, &g);
  }
}
 
void filterEvents(){ //External Function dedicated for filtering operation
  for(int i=0; i<measurementSize; i++){  //Calculation of sum of measurement exploiting a for-loop
    //summedOutput.ax += a.acceleration.x;
    //summedOutput.ay += a.acceleration.y;
    //summedOutput.az += a.acceleration.z;
    summedOutput.gx += g.gyro.x;
    summedOutput.gy += g.gyro.y;
    summedOutput.gz += g.gyro.z;
    delay(10);
  }
   //filteredOutput.ax = summedOutput.ax/measurementSize; //Perform Filtering through average calculation of accelererometers and gyroscope components
   //filteredOutput.ay = summedOutput.ay/measurementSize;
   //filteredOutput.az = summedOutput.az/measurementSize;
   filteredOutput.gx = summedOutput.gx/measurementSize;
   filteredOutput.gy = summedOutput.gy/measurementSize;
   filteredOutput.gz = summedOutput.gz/measurementSize;
}



