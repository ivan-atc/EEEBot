//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*  UoN EEEBot 2023                                     *//
//*                                                      *//
//*  ESP32 MQTT Example Code                             *//
//*                                                      *//
//*  Nat Dacombe                                         *//
//********************************************************//

// the following code is modified from https://randomnerdtutorials.com by Rui Santos

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

// replace the next variables with your SSID/Password combination
const char* ssid = "EEELab01";
const char* password = "EEEE1002";                

// add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "192.168.6.2";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

// LED Pin
const int ledPin = 4;

void setup() {
  Serial.begin(115200);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(ledPin, OUTPUT);
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

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // feel free to add more if statements to control more GPIOs with MQTT

  // if a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
}

void reconnect() {
  // loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // subscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
    
    
    // convert the value to a char array
    char tempString[8];
    float temperature = 20.2;
    dtostrf(temperature, 1, 2, tempString);
    Serial.print("Temperature: ");
    Serial.println(tempString);
    client.publish("esp32/temperature", tempString); //Publish Output to the Board
    
    // convert the value to a char array
    char humString[8];
    float humidity = 55.5;
    dtostrf(humidity, 1, 2, humString);
    Serial.print("Humidity: ");
    Serial.println(humString);
    client.publish("esp32/humidity", humString);
  }
}
