/*
Scalextric AutoCar Code for ESP8266 Board

1.0   22-04-2017  ALS   Got Board up and running
1.1   23-04-2017  ALS   Installed MQTT libraries and got pub sub working between board and python code on laptop via mqtt-dashboard broker
1.2   24-04-2017  ALS   Added potentiomer (instead of 3-axis accelerometer for now) 
1.3   24-04-2017  ALS   Added Red and Greed LED light to respond to message from broker when we are under 500 kphh / over 500 kph
1.4   04-05-2017  ALS   Added accelerometer and sending data to MQTT broker
*/


/* 
Connect Vin to the power supply, 3-5V is fine. Use the same voltage that the microcontroller logic is based off of. For most Arduinos, that is 5V
Connect GND to common power/data ground
Connect the SCL pin to the D1
Connect the SDA pin to the D2
 */

 
// Include the ESP Wifi library so we can get on to the network, as well as the MQTT library
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Include standard libraries
#include<stdlib.h>
#include<String.h>

// Include sensor library
#include "Adafruit_MMA8451.h"
#include <Adafruit_Sensor.h>

#include<Wire.h>

// Update these with values suitable for the network.

//const char* ssid = "VM6251929";
//const char* password = "wf8cfCkfPnfy";
//const char* mqtt_server = "www.mqtt-dashboard.com";

const char* ssid = "Xperia X_d5f8";
const char* password = "!1234567890";
const char* mqtt_server = "192.168.43.186";

// Accelerometer initiation
Adafruit_MMA8451 mma = Adafruit_MMA8451();

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
String msg;


int accel_value;             // Initialise accelerometer value
int GREEN_LED = 13;          // The Greed LED is connected to GPIO pin D7 (in reality Arduino pin 13 !)
int RED_LED = 12;            // The Red LED is connected to GPIO pin D6 (in reality Arduino pin 12 !)

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output - we will flash the built in LED when we get a message pushed to the board
  pinMode(GREEN_LED, OUTPUT);     // Initialize the GREEN_LED pin as an output
  
  Serial.begin(115200);
  setup_accelerometer();
  setup_wifi();
  client.setServer(mqtt_server, 1883);  /// default port for mqtt servers is 1883
  client.setCallback(callback);
}

void setup_accelerometer() {
  if (! mma.begin()) {
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("MMA8451 found!");
 

}
void setup_wifi() {
    digitalWrite(GREEN_LED, LOW);  // Turn the green LED low
    digitalWrite(RED_LED, LOW);   // Turn the red LED low

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

WiFi.begin(ssid, password);

  // Let's get connected first
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if a 1 was received as first character
  if ((char)payload[0] == '1') {
    Serial.println("Speed Up");
    digitalWrite(RED_LED, LOW);   // Turn the red LED off
    digitalWrite(GREEN_LED, HIGH);   // Turn the green LED on
  }
  if ((char)payload[0] == '0') {
    Serial.println("Slow Down");
    digitalWrite(GREEN_LED, LOW);  // Turn the green LED off
    digitalWrite(RED_LED, HIGH);   // Turn the red LED on
  }
  if ((char)payload[0] == '2') {
    Serial.println("Speed OK");
    digitalWrite(GREEN_LED, HIGH);  // Turn the green LED on
    digitalWrite(RED_LED, HIGH);   // Turn the red LED on
  }
  if ((char)payload[0] == '3') {
    Serial.println("Stopped");
    digitalWrite(GREEN_LED, LOW);  // Turn the green LED low
    digitalWrite(RED_LED, LOW);   // Turn the red LED low
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting to connect to local MQTT broker.");
    // Attempt to connect
    if (client.connect("Car")) {
      Serial.println("Connected to local MQTT Broker");
    } else {
      Serial.print("Failed to connect to local MQTT Broker: RC=");
      Serial.print(client.state());
      Serial.println("Trying again in 5 seconds");
      // Wait 5 seconds before retrying
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
  // can take this down as far as 5 milliseconds and it seems stable
  if (now - lastMsg > 1000) { 
  long now = millis();
  

  // Read the 'raw' data in 14-bit counts
  mma.read();
  Serial.print("X:\t"); Serial.print(mma.x); 
  Serial.print("\tY:\t"); Serial.print(mma.y); 
  Serial.print("\tZ:\t"); Serial.print(mma.z); 
  Serial.println();
 
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mma.getEvent(&event);
 
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("Accceleration: X: \t"); Serial.print(event.acceleration.x); Serial.print("\t");
  Serial.print("Y: \t"); Serial.print(event.acceleration.y); Serial.print("\t");
  Serial.print("Z: \t"); Serial.print(event.acceleration.z); Serial.println("\t");
  
  
  // Variables to hold x,y and z position of accelerometer
  char x[9];
  char y[9];
  char z[9];

  // Arguments to dtostrf -> initial float value, total with of result, digits post decimal point, target variable.
  dtostrf(event.acceleration.x, 8, 4, x);   
  dtostrf(event.acceleration.y, 8, 4, y);   
  dtostrf(event.acceleration.z, 8, 4, z);   

  msg="{"+ String(x) + "/" + String(y) + "/" + String(z)+ "}";
  lastMsg = now;
  char outputmessage[msg.length()+1];
  msg.toCharArray(outputmessage, msg.length()+1);
    
  Serial.println(outputmessage);
  client.publish("Scalextric/JaguarF1/Eddie_Irvine/Datapoints/",outputmessage );
  }
  
}
