#include <ESP8266WiFi.h>             
#include <FirebaseArduino.h>
#include <NTPClient.h>
#include <WiFiUdp.h>      
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MQ135.h>
#include <SFE_BMP180.h>
#include "constants.h"
            
#define WIFI_SSID "joiiy"                                  
#define WIFI_PASSWORD "joiiy123"            

#define ALTITUDE 1655.0 // Altitude of SparkFun's HQ in Boulder, CO. in meters
#define PIN_MQ135 A0

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
MQ135 mq135Sensor(PIN_MQ135);
SFE_BMP180 bmp;

// Variable to save current epoch time
int timestamp;
float temperature, absolutePressure, relativePressure, altitude, ppm;
char status;
double T,P,p0,a;

void setup() 
{
  Serial.begin(115200); 
  initBMP180();
  initWiFi();
  Firebase.begin(FIREBASE_HOST);                                // connect to the firebase
}
 
void loop() 
{
  
  ppm = mq135Sensor.getPPM();

  Serial.println();
  Serial.print("provided altitude: ");
  Serial.print(ALTITUDE,0);
  Serial.print(" meters, ");
  Serial.print(ALTITUDE*3.28084,0);
  Serial.println(" feet");

  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = bmp.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = bmp.getTemperature(T);
    
    if (status != 0)
    {
      // Print out the measurement:
      Serial.print("temperature: ");
      Serial.print(T,2);
      Serial.print(" deg C, ");
      Serial.print((9.0/5.0)*T+32.0,2);
      Serial.println(" deg F");
      temperature = T;
      
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = bmp.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = bmp.getPressure(P,T);
        if (status != 0)
        {
          // Print out the measurement:
          Serial.print("absolute pressure: ");
          Serial.print(P,2);
          Serial.print(" mb, ");
          Serial.print(P*0.0295333727,2);
          Serial.println(" inHg");
          absolutePressure = P;

          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb

          p0 = bmp.sealevel(P,ALTITUDE); // we're at 1655 meters (Boulder, CO)
          Serial.print("relative (sea-level) pressure: ");
          Serial.print(p0,2);
          Serial.print(" mb, ");
          Serial.print(p0*0.0295333727,2);
          Serial.println(" inHg");
          relativePressure = p0;

          // On the other hand, if you want to determine your altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
          // Result: a = altitude in m.

          a = bmp.altitude(P,p0);
          Serial.print("computed altitude: ");
          Serial.print(a,0);
          Serial.print(" meters, ");
          Serial.print(a*3.28084,0);
          Serial.println(" feet");
          altitude = a;
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");

  Serial.print("%  ppm: ");  
  Serial.println(ppm);
 
  timestamp = getTime();
  
  Firebase.setFloat("/" + String(timestamp) + "/temperature", temperature);
  Firebase.setFloat("/" + String(timestamp) + "/absolutePressure", absolutePressure);
  Firebase.setFloat("/" + String(timestamp) + "/relativePressure", relativePressure);
  Firebase.setFloat("/" + String(timestamp) + "/altitude", altitude);
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



void initBMP180() {
  // Initialize the sensor (it is important to get calibration values stored on the device).
  while(!bmp.begin()) {
    Serial.println("BMP180 init fail\n\n");
  }
  Serial.println("BMP180 init success");
}



void initWiFi() {
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
}
