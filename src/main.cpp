#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_BNO055.h>

#include <utility/imumaths.h>
#include <SPI.h>
#include "Wire.h"
#include "Adafruit_Sensor.h"


/*
    General
*/

unsigned long allTime = 0; // keep track of how long the UAV is in the air for
unsigned long tStart = 0; // keep track of loop time


/* 
    Debug library stuff
*/
#define HOST_NAME "remotedebug"
#define USE_MDNS true
#include <WiFi.h>
#ifdef USE_MDNS
    #include <DNSServer.h>
    #include "ESPmDNS.h"
#endif

#include "RemoteDebug.h"

RemoteDebug Debug;

// SSID and password
const char* ssid = "DESKTOP-2J7JM8Q 6736";
const char* password = "gentnet69";

/*
    Servo setup
*/

Servo dihedral_servo;
int dihedral_pin = 25;
int dihedral_pos = 0;

Servo sweep_servo;
int sweep_pin = 13;
int sweep_pos = 0;

/*
    IMU setup
*/

// Set the delay between fresh samples
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


// Time

uint32_t mLastTime = 0;
uint32_t mTimeSeconds = 0;

/*
    functions
*/
char result[8];
char* convert(float var) {
    return dtostrf(var, 6, 2, result);
}


void setup() {
    Serial.begin(230400);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Initialise the sensor
    if(!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }

    bno.setExtCrystalUse(true);

    /* 
        Servo
    */
    dihedral_servo.attach(dihedral_pin);

    // WiFi connection

    WiFi.begin(ssid, password);
    Serial.println("");

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }

    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Register host name in WiFi and mDNS

    String hostNameWifi = HOST_NAME;
    hostNameWifi.concat(".local");

    #ifdef USE_MDNS  // Use the MDNS ?

        if (MDNS.begin(HOST_NAME)) {
            Serial.print("* MDNS responder started. Hostname -> ");
            Serial.println(HOST_NAME);
        }

        MDNS.addService("telnet", "tcp", 23);

    #endif

	// Initialize RemoteDebug

	Debug.begin(HOST_NAME); // Initialize the WiFi server

    Debug.setResetCmdEnabled(true); // Enable the reset command
	Debug.showProfiler(false); // Profiler (Good to measure times, to optimize codes)
	Debug.showColors(false); // Colors

    // End off setup

    Serial.print("WiFI connected. IP address: ");
    Serial.println(WiFi.localIP());

    
    
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(200);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    allTime = micros();

}


void loop() {
    tStart = micros();

    sensors_event_t orientation_data, accelerometer_data;
    bno.getEvent(&orientation_data, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&accelerometer_data, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    

    unsigned long current_time = micros() - allTime;

	debugI("%lu    %f    %f    %f    %f    %f    %f", 
        current_time, 
        orientation_data.orientation.x,
        orientation_data.orientation.y, 
        orientation_data.orientation.z, 
        accelerometer_data.acceleration.x, 
        accelerometer_data.acceleration.y, 
        accelerometer_data.acceleration.z
    );

    Debug.handle();

    while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000)) {
        //poll until the next sample is ready
        yield();
        //???
    }
}
