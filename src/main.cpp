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

TaskHandle_t Task1;
TaskHandle_t Task2;

File file;


// Time
unsigned long allTime = 0; // keep track of how long the UAV is in the air for
unsigned long tStart = 0;  // keep track of loop time
uint32_t mLastTime = 0;
uint32_t mTimeSeconds = 0;
volatile unsigned long current_time = 0;

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
volatile float elevator_position;

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
 * Core 0 loop for data storing
 */
void record(void * pvParameters){
    
    
    while(1){

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
    buffer[buffer_index+7] = 0;
    buffer_index = buffer_index + 8;

    if(current_time > 3000000){

        unsigned int buffer_index_end = buffer_index;
        buffer_index = 0;

        while(buffer_index <= buffer_index_end){
            file.printf("%lu\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
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
        
        file.close();

        delay(1000);

        ESP.restart();
    }

    while((micros() - tStart) < SAMPLERATE_DELAY_US)
        delayMicroseconds(1);
    }
}


/**
 * Core 1 loop for actuating servos
 */
void actuate(void * pvParameters){
    while(1){
        if(check){
            if(current_time > 800000){
                dxl.setGoalPosition(ELEVATOR_ID, 95, UNIT_DEGREE);
                dxl.setGoalPosition(DIHEDRAL_ID, 30, UNIT_DEGREE);
            }
            if(current_time > 9000000){
                dxl.setGoalPosition(DIHEDRAL_ID, 0, UNIT_DEGREE);
                dxl.setGoalPosition(SWEEP_ID, 250, UNIT_DEGREE);
                check = false;
            }
        }
        //elevator_position = dxl.getPresentPosition(ELEVATOR_ID, UNIT_DEGREE);
        vTaskDelay(10);
    }
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
    FastLED.showColor(CRGB::ForestGreen);

    servo_setup();

    dxl.setGoalPosition(ELEVATOR_ID, 65, UNIT_DEGREE);
    dxl.setGoalPosition(DIHEDRAL_ID, 250, UNIT_DEGREE);
    dxl.setGoalPosition(SWEEP_ID, 0, UNIT_DEGREE);

    wifi_telem_setup();

    imu_start();

    FastLED.showColor(CRGB::AntiqueWhite);


    if(!SPIFFS.begin(true)){
      debugW("An Error has occurred while mounting SPIFFS");
    }

    wait_for_button_press();

    file = SPIFFS.open("/data.txt", FILE_READ);
    while(file.available()){
        String line = file.readStringUntil('\n');
        Debug.println(line);
    }
    Debug.handle();
    file.close();

    FastLED.showColor(CRGB::Amethyst);

    delay(1000);

    wait_for_button_press();

    file = SPIFFS.open("/data.txt", FILE_WRITE);

    countdown(5);

    imu_check();

    xTaskCreatePinnedToCore(
        record,        /* Task function. */
        "record",      /* name of task. */
        10000,          /* Stack size of task */
        NULL,           /* parameter of the task */
        1,              /* priority of the task */
        &Task1,         /* Task handle to keep track of created task */
        0               /* pin task to core 0 */
    );

    xTaskCreatePinnedToCore(
        actuate,        /* Task function. */
        "actuate",      /* name of task. */
        10000,          /* Stack size of task */
        NULL,           /* parameter of the task */
        2,              /* priority of the task */
        &Task2,         /* Task handle to keep track of created task */
        1               /* pin task to core 0 */
    );

    debugW("Launch");
    Debug.handle();
    
    Debug.disconnect();
    WiFi.disconnect();
    allTime = micros();
}

/**
 * Send new data (IMU, Servo)
 * calaculate / command servo positions
 */
void loop()
{
    vTaskSuspend(NULL);
}