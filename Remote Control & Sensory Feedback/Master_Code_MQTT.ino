#include <WiFi.h>
#include <PubSubClient.h>
#include <HCSR04.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


#define MEASUREMENTS_SIZE 10
#define MAX_CHARACTERS 50
#define ECHO 34
#define TRIGG 19
Adafruit_MPU6050 mpu;
HCSR04 ultrasonicSensor(ECHO, TRIGG); //initialisation of HCSR04 (trig pin , echo pin)

// replace the next variables with your SSID/Password combination
const char* ssid = "EEELab01";
const char* password = "EEEE1002";                

// add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "192.168.6.23";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

float obstructionDistance = 0;
float orientation = 0;
float fallRise= 0;
float gyroYoffset= 0;
float gyroZoffset= 0;

const int NEGATIVE_ROTATION_THRESHOLD = -90;
const int POSITIVE_ROTATION_THRESHOLD = 90;

float distanceMeasurements[MEASUREMENTS_SIZE];
float orientationMeasurements[MEASUREMENTS_SIZE];


float lastTime = 0; // Global variable for time tracking

void setup() {
  Serial.begin(115200);
  Wire.begin();
  setup_wifi();
  client.setServer(mqtt_server, 1883);
}

void setup_wifi() {
  delay(10);
  // we start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {

    while (!client.connected()) {
        reconnect();
    }
    client.loop();
    calibrateMPU();
    readGyro();
    readPulse();
    calibrateSensor();
   
    
    long now = millis();
    if (now - lastMsg > 5000) {
        lastMsg = now;
    }
      toString();
}

void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client")) {
            Serial.println("connected");
        } else {
            Serial.print("failed");
            delay(1000);
        }
    }
}

void toString() {
   char angleString[MAX_CHARACTERS];
   dtostrf(orientation, 1, 50, angleString);
   Serial.print("Angle: ");
   Serial.println(angleString);
   client.publish("esp32/orientation", angleString); //Publish Output to the Board

    char distanceString[MAX_CHARACTERS];
    dtostrf(obstructionDistance,1 , 50, distanceString);
    Serial.print("Obstruction:");
    Serial.println(obstructionDistance);
    client.publish("esp32/obstruction", distanceString);
    
}

void readPulse() {
    const float sound_speed = 0.034;

    for (int i = 0; i <= MEASUREMENTS_SIZE; i++) {
        digitalWrite(TRIGG, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGG, LOW);
        
        float duration = pulseIn(ECHO, HIGH);
        Serial.print(duration);
        distanceMeasurements[i] = (duration * sound_speed) / 2;
        Serial.print( distanceMeasurements[i]);
    }
}

void readGyro() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    for (int i = 0; i < MEASUREMENTS_SIZE; i++) {
        float gyroZ = g.gyro.z - gyroZoffset;
        orientationMeasurements[i] += gyroZ * dt * (180.0 / PI);
    }

    if (orientation >= 180) orientation -= 360;
    if (orientation < -180) orientation += 360;
}

//Calibrate Sensors
void calibrateSensor() {
    float dSum = 0;
    for (int i = 0; i < MEASUREMENTS_SIZE; i++) {
        dSum = distanceMeasurements[i] + dSum;
    }
    obstructionDistance = dSum / MEASUREMENTS_SIZE;
    Serial.println(obstructionDistance);

      float yawSum = 0;
      float fallRiseSum = 0;
    for (int i = 0; i < MEASUREMENTS_SIZE; i++) {
        yawSum += orientationMeasurements[i];
    }
    orientation = yawSum / MEASUREMENTS_SIZE;
    fallRise = fallRiseSum/MEASUREMENTS_SIZE;
    Serial.print("Orientation");
    Serial.println(orientation);
    Serial.print("Fall/Rise Rate");
    Serial.println(fallRise);
}

//Calibrate MPU
void calibrateMPU() {
    Serial.println("Start calibration.");
    float sumGyroZ = 0;
    float sumGyroY = 0;
    int samples = 100;

    for (int i = 0; i < samples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        sumGyroZ += g.gyro.z;
        delay(10);
    }
    gyroZoffset = sumGyroZ / samples;
}
