#include <ESP8266WiFi.h>             
#include <FirebaseArduino.h>
#include <NTPClient.h>
#include <WiFiUdp.h>      
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MQ135.h>

#define FIREBASE_HOST "ioe-aqm-default-rtdb.firebaseio.com"      
#define FIREBASE_AUTH "SnlPoBGQZOSvp7Rf1AOQT6dnQKzbFlfql6EHeQFD"            
#define WIFI_SSID "joiiy"                                  
#define WIFI_PASSWORD "joiiy123"            
 
#define DHTPIN D2                                            // Digital pin connected to DHT11
#define DHTTYPE DHT11                                        // Initialize dht type as DHT 11
#define SEALEVELPRESSURE_HPA (1013.25)
#define PIN_MQ135 A0

MQ135 mq135_sensor(PIN_MQ135);

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// Variable to save current epoch time
int timestamp;

Adafruit_BME280 bmp;
float temperature, humidity, pressure, altitude;

void setup() 
{
  Serial.begin(115200);                                        //reads dht sensor data 
               
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);                                  
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
 
  Serial.println();
  Serial.print("Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());                               // prints local IP address
  Firebase.begin(FIREBASE_HOST);                                // connect to the firebase
  
//  bmp.begin(0x76);
 
}
 
void loop() 
{
//  temperature = bmp.readTemperature();
//  pressure = bmp.readPressure() / 100.0F;
//  altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  float ppm = mq135_sensor.getPPM();
   
//  Serial.print("temperature: ");  
//  Serial.print(temperature);
//  Serial.print("°C ");
//  Serial.print("%  pressure: ");  
//  Serial.print(pressure);
//  Serial.print("%  altitude: ");  
//  Serial.println(altitude);
  Serial.print("%  ppm: ");  
  Serial.println(ppm);
 
  timestamp = getTime();

//  Firebase.setFloat("/" + String(timestamp) + "/temperature", temperature);
//  Firebase.setFloat("/" + String(timestamp) + "/pressure", pressure);
//  Firebase.setFloat("/" + String(timestamp) + "/altitude", altitude);
  Firebase.setFloat("/" + String(timestamp) + "/ppm", ppm);
  
  if (Firebase.failed()) 
  {
    Serial.print("pushing /logs failed:");
    Serial.println(Firebase.error()); 
    return;
  }

  delay(20000);
}

// Function that gets current epoch time
unsigned long getTime() {
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  return now;
}
