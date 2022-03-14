#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_BNO055.h>
#include <FastLED.h>
#include <Dynamixel.h>
#include <RemoteDebug.h>

#include <utility/imumaths.h>
#include <SPI.h>
#include "Wire.h"
#include "Adafruit_Sensor.h"
#include <WiFi.h>
#include <DNSServer.h>
#include "ESPmDNS.h"

#include <utils.h>

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
unsigned long tStart = 0;  // keep track of loop time
const byte buttonPin = 27;

/*
    Debug library stuff
*/
#define HOST_NAME "remotedebug"

RemoteDebug Debug;

// SSID and password
const char *ssid = "DESKTOP-2J7JM8Q 6736";
const char *password = "gentnet69";

/*
    Servo setup
*/
#define DYNAMIXEL_SERIAL Serial2 // change as you want

const uint8_t SERVO_DIHEDRAL_ID = 2;
const uint8_t SERVO_SWEEP_ID = 3;
const uint8_t SERVO_ELEVATOR_ID = 1;
const uint8_t PIN_RTS = 36;
const uint16_t DYNAMIXEL_BAUDRATE = 57600;

Dynamixel dxl(PIN_RTS); // create instance with RTS pin

int dxl_dihedral_goal_position[2];
int dxl_sweep_goal_position[2];
int dxl_elevator_goal_position[2];

/*
    IMU setup
*/

// Set the delay between fresh samples
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Time

uint32_t mLastTime = 0;
uint32_t mTimeSeconds = 0;


void setup()
{
    Serial.begin(230400);

    // RBG LED SETUP
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS); // GRB ordering is assumed
    leds[0] = CRGB::Chocolate;
    FastLED.show();

    // Initialise the IMU
    if (!bno.begin())
        Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    bno.setExtCrystalUse(true);

    /*
        Servo
    */
    DYNAMIXEL_SERIAL.begin(DYNAMIXEL_BAUDRATE);
    dxl.attach(DYNAMIXEL_SERIAL, DYNAMIXEL_BAUDRATE);
    dxl.addModel<DxlModel::X>(SERVO_DIHEDRAL_ID);
    dxl.addModel<DxlModel::X>(SERVO_SWEEP_ID);
    //dxl.addModel<DxlModel::X>(SERVO_ELEVATOR_ID);
    
    dxl.torqueEnable(SERVO_DIHEDRAL_ID, false);
    dxl.torqueEnable(SERVO_SWEEP_ID, false);
    //dxl.torqueEnable(SERVO_ELEVATOR_ID, false);

    dxl.torqueEnable(SERVO_DIHEDRAL_ID, true);

    // WiFi connection

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
        delay(500); Serial.print(".");
    Serial.print("Connected to "); Serial.println(ssid);
    Serial.print("IP address: "); Serial.println(WiFi.localIP());

    // Register host name in WiFi and mDNS
    String hostNameWifi = HOST_NAME;
    hostNameWifi.concat(".local");
    if (MDNS.begin(HOST_NAME))
        Serial.print("* MDNS responder started. Hostname -> "); Serial.println(HOST_NAME);
    MDNS.addService("telnet", "tcp", 23);

    // Initialize RemoteDebug
    Debug.begin(HOST_NAME); // Initialize the WiFi server
    Debug.setResetCmdEnabled(true); // Enable the reset command
    Debug.showProfiler(false);      // Profiler (Good to measure times, to optimize codes)
    Debug.showColors(false);        // Colors

    // End off setup

    Serial.print("WiFI connected. IP address: "); Serial.println(WiFi.localIP());

    leds[0] = CRGB::AntiqueWhite;
    FastLED.show();

    pinMode(buttonPin, INPUT_PULLUP);
    while (digitalRead(buttonPin) == HIGH)
        delayy(500, Debug);

    for (int i = 0; i <= 5; i++)
    {
        leds[0] = CRGB::Green;
        FastLED.show();
        delayy(500, Debug);
        leds[0] = CRGB::DarkRed;
        FastLED.show();
        delayy(500, Debug);
    }
    leds[0].r = 0;
    leds[0].g = 0;
    leds[0].b = 250;

    allTime = micros();
    debugW("Launch");
}

bool dir = true; // CW or CCW

void loop()
{
    tStart = micros();

    dxl.goalPosition(SERVO_DIHEDRAL_ID, dxl_dihedral_goal_position[(size_t)dir]); // move to position

    delay(3000);

    Serial.print("current pos = ");
    Serial.println(dxl.presentPosition(SERVO_DIHEDRAL_ID)); // get current position

    dir = !dir; // reverse direction

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
           accelerometer_data.acceleration.z);

    Debug.handle();

    leds[0].r++;
    if (leds[0].r == 254)
    {
        leds[0].r = 0;
    }
    FastLED.show();

    while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000))
    {
        // poll until the next sample is ready
        yield();
        //???
    }
}
