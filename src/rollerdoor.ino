#include <elapsedMillis.h>
#include <Particle.h>
#include "application.h"
#include <stdarg.h>

int doorUpPin = D0; // Input from door up hall effect sensor
int doorDownPin = D1; // Input from door down hall effect sensor
int boardLed = D7; // This is the LED that is already on your device.
unsigned int doorStatus;
bool ledStatus;
int doorUp;
int doorDown;
int rssi = 0;
byte bssid[6];
String sBssid = "";

const unsigned int unknown = 0;
const unsigned int open = 1;
const unsigned int opening = 2;
const unsigned int closed = 3;
const unsigned int closing = 4;

STARTUP( WiFi.selectAntenna(ANT_AUTO));

// Setup doorStatus indicate LED update timer
void updateDoorMovingStatusLed()
{
    if((doorStatus==opening)|(doorStatus==closing))
    {
        ledStatus = !ledStatus;
        digitalWrite(boardLed,ledStatus);
    }
    else if (doorStatus == open)
        digitalWrite(boardLed,HIGH);
    else if (doorStatus == closed)
        digitalWrite(boardLed,LOW);
}



// Setup doorStatus indicate LED update timer
void updateDoorUnknownStatusLed()
{
    if((doorStatus==unknown))
        {
            ledStatus = !ledStatus;
            digitalWrite(boardLed,ledStatus);
        }
}

Timer moving_timer(300, updateDoorMovingStatusLed);
Timer unknown_timer(75, updateDoorUnknownStatusLed);


void setup() {
    doorStatus = unknown;
    // Particle.variable("doorStatus",doorStatus);
    Particle.publish("doorStatus","unknown",60,PRIVATE);
    Particle.publish("doorEvent","127",60,PRIVATE);

    Particle.variable("rssi", rssi);
    Particle.variable("bssid", sBssid);

    pinMode(doorUp,INPUT); // Our LED pin is output (lighting up the LED)
    pinMode(boardLed,OUTPUT); // Our on-board LED is output as well
    pinMode(doorDown,INPUT);  // Our photoresistor pin is input (reading the photoresistor)

    Serial.begin();
    digitalWrite(boardLed,LOW); // Turn off board LED
    ledStatus = LOW;
    // WiFi.off();
    WiFi.selectAntenna(ANT_INTERNAL);
    // WiFi.on();
    // while(!WiFi.ready())
    // {
    //   Serial.println("Waiting for WiFi Internal");
    //   Particle.process();
      delay(1000);
    // }
    rssi = WiFi.RSSI();
    Particle.publish("rssiInternal",String(rssi),60,PRIVATE);
    // WiFi.off();
    WiFi.selectAntenna(ANT_EXTERNAL);
    // WiFi.on();
    // while(!WiFi.ready())
    // {
    //   Serial.println("Waiting for WiFi External");
    //   Particle.process();
      delay(1000);
    // }
    rssi = WiFi.RSSI();
    WiFi.BSSID(bssid); 
    Particle.publish("rssiExternal",String(rssi),60,PRIVATE);
    sBssid = String::format("%02X:%02X:%02X:%02X:%02X:%02X", bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
    Particle.publish("BSSID",sBssid,60,PRIVATE);
}

// Now for the loop.

void loop()
{
    if((doorStatus == unknown) & !unknown_timer.isActive())
        unknown_timer.start();
    else if((doorStatus != unknown) & unknown_timer.isActive())
        unknown_timer.stop();

    doorUp = !digitalRead(doorUpPin);
    doorDown = !digitalRead(doorDownPin);

    if (doorUp & doorDown)
    {
            if (doorStatus != unknown)
            {
                Particle.publish("doorStatus","unknown",60,PRIVATE);
                Particle.publish("doorEvent","127",60,PRIVATE);
                Serial.println("Housten, we have a problem");
                moving_timer.stop();
                unknown_timer.start();
            }
            doorStatus = unknown;
    }
    else if (doorUp)
        {
            if ((doorStatus==opening) | (doorStatus==unknown) | (doorStatus==closing))
            {
                Particle.publish("doorStatus","open",60,PRIVATE);
                Particle.publish("doorEvent","255",60,PRIVATE);
                Serial.println("Door now open");
                moving_timer.stop();
                digitalWrite(boardLed,HIGH);
            }
            doorStatus=open;
        }
        else if(doorStatus==open)
        {// was open
            Particle.publish("doorStatus","closing",60,PRIVATE);
            Particle.publish("doorEvent","85",60,PRIVATE);
            doorStatus=closing;
            moving_timer.start();
            Serial.println("Door now closing");
        }
    else if (doorDown)
        {
            if ((doorStatus==closing) | (doorStatus == unknown) | (doorStatus == opening))
            {
                Particle.publish("doorStatus","closed",60,PRIVATE);
                Particle.publish("doorEvent","0",60,PRIVATE);
                Serial.println("Door now closed");
                moving_timer.stop();
                digitalWrite(boardLed,LOW);
            }
            doorStatus=closed;
        }
        else if(doorStatus==closed)
            { // was closed
                Particle.publish("doorStatus","opening",60,PRIVATE);
                Particle.publish("doorEvent","170",60,PRIVATE);
                doorStatus=opening;
                moving_timer.start();
                Serial.println("Door now opening");
        }
        rssi=WiFi.RSSI();
}
