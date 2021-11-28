#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_BNO055.h>
#include <WebSerial.h>

// dependancies
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

/* 
    Webserial setup
*/

AsyncWebServer server(80);
const char* ssid = "DESKTOP-2J7JM8Q 6736";    // Your WiFi SSID
const char* password = "gentnet69"; 

/*
    Servo setup
*/

Servo dihedral_servo;
int dihedral_pin = 13;
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
//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2

/*
    functions
*/

/* Message callback of WebSerial */
void recvMsg(uint8_t *data, size_t len) {
    WebSerial.println("Received Data...");
    String d = "";
    for(int i=0; i < len; i++) {
        d += char(data[i]);
    }
    WebSerial.println(d);
}

/*
    Main Loop
*/

void setup() {
    /*
        General
    */
    Serial.begin(9600);

    /*
        IMU
    */
    // Initialise the sensor
    if(!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        WebSerial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }

    bno.setExtCrystalUse(true);

    /* 
        Servo
    */
    dihedral_servo.attach(dihedral_pin);

    /*
        Webserial
    */
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.printf("WiFi Failed!\n");
        return;
    }
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    // WebSerial is accessible at "<IP Address>/webserial" in browser
    WebSerial.begin(&server);
    WebSerial.msgCallback(recvMsg);
    server.begin();
}


void loop() {
    unsigned long tStart = micros();

    // Get IMU data
    sensors_event_t event;
    bno.getEvent(&event);

    WebSerial.println((float)event.orientation.x);
    WebSerial.println((float)event.orientation.y);
    WebSerial.println((float)event.orientation.z);

    sensors_event_t orientationData , accelData, gyroData;
    
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    WebSerial.println("hello");

    while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000)) {
        //poll until the next sample is ready
    }
}