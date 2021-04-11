/*
 *  This sketch sends data via HTTP GET requests to data.sparkfun.com service.
 *
 *  You need to get streamId and privateKey at data.sparkfun.com and paste them
 *  below. Or just customize this script to talk to other HTTP servers.
 *
 */

#include <WiFi.h>

const char* ssid     = "hotspot";
const char* password = "abcdefgh";
bool isConnect = true;
unsigned long Time;

void setup()
{
    Serial.begin(115200);
    delay(10);

    // We start by connecting to a WiFi network

    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);
    Time = millis();
    while (!WiFi.isConnected()) {
        if(millis() - Time > 10000){
          Serial.println("Time out!");
          isConnect = false;
          break;
        }
    }
    if(isConnect){
      Serial.println("");
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      Serial.println("MAC address: ");
      Serial.println(WiFi.macAddress());
    }
    Serial.println("The program has been run out");
}

void loop()
{
    
}
