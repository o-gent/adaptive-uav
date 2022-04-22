#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_ISM330DHCX.h>
#include <FastLED.h>
#include <Dynamixel2Arduino.h>
#include <RemoteDebug.h>
#include "SPIFFS.h"

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
bool check = true;
float buffer[17500];
unsigned int buffer_index = 0;

// Time
unsigned long allTime = 0; // keep track of how long the UAV is in the air for
unsigned long tStart = 0;  // keep track of loop time
uint32_t mLastTime = 0;
uint32_t mTimeSeconds = 0;
unsigned long current_time = 0;

TaskHandle_t Task1;
TaskHandle_t Task2;

/*
    Debug library stuff
*/
#define HOST_NAME "remotedebug"
RemoteDebug Debug;
// SSID and password to local network
const char *ssid = "DESKTOP-2J7JM8Q 6736";
const char *password = "MjTNaC$VB4SA";
//WiFiClient raw_telem;

/*
    Servo setup
*/
#define DXL_SERIAL Serial2
const uint8_t DXL_DIR_PIN = A4; // DYNAMIXEL Shield DIR PIN
const uint8_t DIHEDRAL_ID = 2;
const uint8_t SWEEP_ID = 3;
const uint8_t ELEVATOR_ID = 4;
const float DXL_PROTOCOL_VERSION = 2.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;
float elevator_position = 0;

/*
    IMU setup
*/
#define SAMPLERATE_DELAY_US 3000 // Set the delay between fresh samples
Adafruit_ISM330DHCX ism330dhcx;
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

/**
 * check we are actually getting IMU data
 */
void imu_check()
{
    ism330dhcx.getEvent(&accel, &gyro, &temp);
    if (accel.acceleration.y == 0)
    {
        Serial.println("Accelerometer reading 0");
        debugI("Accelerometer reading 0");
        FastLED.showColor(CRGB::Red);
        while(1)
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
    Debug.begin(HOST_NAME);
    Debug.setResetCmdEnabled(true);
    Debug.showProfiler(false);
    Debug.showColors(false);

    Serial.print("WiFI connected. IP address: ");
    Serial.println(WiFi.localIP());

    //raw_telem = Debug.getTelnetClient();
}

/**
 * start and arm servos
 */
void servo_setup()
{
    dxl.begin(57600);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    
    for(int ID = 2; ID<=4; ID++)
    {
        dxl.torqueOff(ID);
        dxl.setOperatingMode(ID, OP_POSITION);
        dxl.writeControlTableItem(PROFILE_VELOCITY, ID, 0); // Use 0 for Max speed
        dxl.torqueOn(ID);
    }
}

/**
 * start the IMU
 */
void imu_start()
{
    if (!ism330dhcx.begin_I2C())
    {
        Serial.println("no IMU detected");
        debugI("no IMU detected");
        FastLED.showColor(CRGB::Red);
        while(1)
            delayy(1000, Debug);
    }
    ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
    ism330dhcx.setGyroRange(ISM330DHCX_GYRO_RANGE_4000_DPS);
    ism330dhcx.setAccelDataRate(LSM6DS_RATE_416_HZ);
    ism330dhcx.setGyroDataRate(LSM6DS_RATE_416_HZ);
}

/**
 * actuate the servos at the required time
 */ 
void actuate(void * pvParameters){
    if(check){
        if(current_time > 500000){
            dxl.setGoalPosition(ELEVATOR_ID, 35, UNIT_DEGREE);
            dxl.setGoalPosition(DIHEDRAL_ID, 30, UNIT_DEGREE);
            check = false;
        }
        delayMicroseconds(1);
    }
    else {
        // task finished
        delayMicroseconds(100000);
    }
}

void dxl_data(void * pvParameters){
    delayMicroseconds(1000);
    elevator_position = dxl.getPresentPosition(ELEVATOR_ID, UNIT_DEGREE);
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

    dxl.setGoalPosition(ELEVATOR_ID, 65, UNIT_DEGREE);
    dxl.setGoalPosition(DIHEDRAL_ID, 0, UNIT_DEGREE);
    dxl.setGoalPosition(SWEEP_ID, 0, UNIT_DEGREE);

    wait_for_button_press();

    countdown(5);

    imu_check();

    xTaskCreatePinnedToCore(
        actuate,        /* Task function. */
        "actuate",      /* name of task. */
        10000,          /* Stack size of task */
        NULL,           /* parameter of the task */
        1,              /* priority of the task */
        &Task1,         /* Task handle to keep track of created task */
        0               /* pin task to core 0 */
    );
    
    xTaskCreatePinnedToCore(
        dxl_data,        /* Task function. */
        "dxl_data",      /* name of task. */
        10000,          /* Stack size of task */
        NULL,           /* parameter of the task */
        1,              /* priority of the task */
        &Task2,         /* Task handle to keep track of created task */
        0               /* pin task to core 0 */
    );

    // printf( "Task Name\tStatus\tPrio\tHWM\tTask\tAffinity\n");
    // char stats_buffer[1024];
    // vTaskList(stats_buffer);
    // printf("%s\n", stats_buffer);


    allTime = micros();
    debugW("Launch");
    Debug.handle();
}

/**
 * Send new data (IMU, Servo)
 * calaculate / command servo positions
 */
void loop()
{
    tStart = micros();
    ism330dhcx.getEvent(&accel, &gyro, &temp);
    current_time = micros() - allTime;
    
    buffer[buffer_index] = current_time;
    buffer[buffer_index+1] = gyro.gyro.x;
    buffer[buffer_index+2] = gyro.gyro.y;
    buffer[buffer_index+3] = gyro.gyro.z;
    buffer[buffer_index+4] = accel.acceleration.x;
    buffer[buffer_index+5] = accel.acceleration.y;
    buffer[buffer_index+6] = accel.acceleration.z;
    buffer[buffer_index+7] = elevator_position;
    buffer_index = buffer_index + 8;
    
    if ((micros() - tStart) < SAMPLERATE_DELAY_US)
    {

        if(current_time > 3000000){
            unsigned int buffer_index_end = buffer_index;
            buffer_index = 0;

            while(buffer_index <= buffer_index_end){
                debugI("%lu\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
                    (unsigned long)buffer[buffer_index],
                    buffer[buffer_index+1],
                    buffer[buffer_index+2],
                    buffer[buffer_index+3],
                    buffer[buffer_index+4],
                    buffer[buffer_index+5],
                    buffer[buffer_index+6],
                    buffer[buffer_index+7]
                );
                buffer_index = buffer_index + 8;
            }

            while(1)
                debugI("finished");Debug.handle();delayy(1000, Debug);
        }
    }

    while((micros() - tStart) < SAMPLERATE_DELAY_US)
        delayMicroseconds(1);
}
