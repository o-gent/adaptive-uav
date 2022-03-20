#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_BNO055.h>
#include <FastLED.h>
#include <Dynamixel2Arduino.h>
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
#define NUM_LEDS 1
#define DATA_PIN 5

CRGB leds[NUM_LEDS];
const byte buttonPin = 27;

// Time
unsigned long allTime = 0; // keep track of how long the UAV is in the air for
unsigned long tStart = 0;  // keep track of loop time
uint32_t mLastTime = 0;
uint32_t mTimeSeconds = 0;

/*
    Debug library stuff
*/
#define HOST_NAME "remotedebug"
RemoteDebug Debug;
// SSID and password to local network
const char *ssid = "DESKTOP-2J7JM8Q 6736";
const char *password = "MjTNaC$VB4SA";

/*
    Servo setup
*/
#define DXL_SERIAL Serial2
const uint8_t DXL_DIR_PIN = A4; // DYNAMIXEL Shield DIR PIN
const uint8_t DXL_ID = 2;
const float DXL_PROTOCOL_VERSION = 2.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

/*
    IMU setup
*/
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; // Set the delay between fresh samples
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/**
 * check we are actually getting IMU data
 */
void imu_check()
{
    sensors_event_t accelerometer_data;
    bno.getEvent(&accelerometer_data, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    if (accelerometer_data.acceleration.y == 0)
    {
        Serial.println("Accelerometer reading 0");
        debugI("Accelerometer reading 0");
        FastLED.showColor(CRGB::Red);
        delayy(1000, Debug);
    }
}

/**
 * poll button state change
 */
void wait_for_button_press()
{
    pinMode(buttonPin, INPUT_PULLUP);
    while (digitalRead(buttonPin) == HIGH)
        delayy(500, Debug);
}

/**
 * flash the LEDs for a set time
 */
void countdown(int amount)
{
    for (int i = 0; i <= amount; i++)
    {
        FastLED.showColor(CRGB::Green);
        delayy(500, Debug);
        FastLED.showColor(CRGB::Yellow);
        delayy(500, Debug);
    }
    leds[0].r = 0;
    leds[0].g = 0;
    leds[0].b = 250;
}

/**
 * Initialise the WiFi connection to ground station and setup the "Debug" library
 * Won't proceed until wifi connection established
 */
void wifi_telem_setup()
{
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
        delay(500);
    Serial.print(".");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Register host name in WiFi and mDNS
    String hostNameWifi = HOST_NAME;
    hostNameWifi.concat(".local");
    if (MDNS.begin(HOST_NAME))
        Serial.print("* MDNS responder started. Hostname -> ");
    Serial.println(HOST_NAME);
    MDNS.addService("telnet", "tcp", 23);

    // Initialize RemoteDebug
    Debug.begin(HOST_NAME);         // Initialize the WiFi server
    Debug.setResetCmdEnabled(true); // Enable the reset command
    Debug.showProfiler(false);      // Profiler (Good to measure times, to optimize codes)
    Debug.showColors(false);        // Colors

    Serial.print("WiFI connected. IP address: ");
    Serial.println(WiFi.localIP());
}

/**
 * start and arm servos
 */
void servo_setup()
{
    dxl.begin(57600);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    
    dxl.torqueOff(DXL_ID);
    dxl.setOperatingMode(DXL_ID, OP_POSITION);

    dxl.torqueOn(DXL_ID);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 0); // Use 0 for Max speed
}

/**
 * start the IMU
 */
void imu_start()
{
    if (!bno.begin())
    {
        Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        debugI("IMU not detected");
        FastLED.showColor(CRGB::Red);
    }
    bno.setExtCrystalUse(true);
}

/**
 * configure
 *  - WiFi
 *  - IMU
 *  - Servos
 */
void setup()
{
    Serial.begin(230400);

    // RBG LED SETUP
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS); // GRB ordering is assumed
    FastLED.showColor(CRGB::Chocolate);

    wifi_telem_setup();

    imu_start();

    servo_setup();
    
    FastLED.showColor(CRGB::AntiqueWhite);

    wait_for_button_press();

    countdown(5);

    imu_check();

    allTime = micros();
    debugW("Launch");
}

/**
 * Send new data (IMU, Servo)
 * calaculate / command servo positions
 */
void loop()
{
    tStart = micros();
 
    // telemetry send
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

    while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000))
    {
        // vary the LED colour so we know the loop is still running
        leds[0].r++;
        if (leds[0].r == 254)
            leds[0].r = 0;
        FastLED.show();
    }
}
