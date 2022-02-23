#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_BNO055.h>
#include <FastLED.h>

#include <utility/imumaths.h>
#include <SPI.h>
#include "Wire.h"
#include "Adafruit_Sensor.h"


/*
    General
*/
// How many leds in your strip?
#define NUM_LEDS 1
#define DATA_PIN 5
#define CLOCK_PIN 13
CRGB leds[NUM_LEDS];
int led_selector = 0;
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

void delayy(unsigned long ms) {
    uint32_t start = micros();
    debugA("waiting");
    Debug.handle();
    while (ms > 0) {
        while ( ms > 0 && (micros() - start) >= 1000) {
            ms--;
            start += 1000;
        }
    }
}


void setup() {

    Serial.begin(230400);

    // RBG LED SETUP
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
    leds[0] = CRGB::Chocolate;
    FastLED.show();

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


    leds[0] = CRGB::AntiqueWhite;
    FastLED.show();

    const byte buttonPin = 27;
    pinMode(buttonPin, INPUT_PULLUP);
    while (digitalRead(buttonPin) == HIGH){
        delayy(500);
    }

    for(int i = 0; i<=5; i++){
        leds[0] = CRGB::Green;
        FastLED.show();
        delayy(500);
        leds[0] = CRGB::DarkRed;
        FastLED.show();
        delayy(500);
    }
    leds[0].r = 0; leds[0].g = 0; leds[0].b = 250;

    allTime = micros();
    debugW("Launch");
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

    leds[0].r++;
    if(leds[0].r == 254){
        leds[0].r = 0;
    }
    FastLED.show();

    while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000)) {
        //poll until the next sample is ready
        yield();
        //???
    }
}
