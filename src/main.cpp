#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_BNO055.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// dependancies
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <SPI.h>

/*
    General
*/

unsigned long allTime = 0; // keep track of how long the UAV is in the air for
unsigned long tStart = 0; // keep track of loop time

/* 
    Webserial setup
*/

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const char* ssid = "DESKTOP-2J7JM8Q 6736";    // Your WiFi SSID
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
//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2

/*
    functions
*/
char result[8];
char* convert(float var) {
    return dtostrf(var, 6, 2, result);
}

char* sensor_data(char * data) {
    // Get IMU data
    sensors_event_t event;
    bno.getEvent(&event);

    //client->text(convert((float)event.orientation.x));
    //client->text(convert((float)event.orientation.y));
    //client->text(convert((float)event.orientation.z));

    // sensors_event_t orientationData , accelData, gyroData;
    
    // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    // bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    // bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
 
    if(type == WS_EVT_CONNECT){

        Serial.println("Websocket client connection received");
        client->text("Hello from ESP32 Server");
    
    } else if(type == WS_EVT_DISCONNECT){
        Serial.println("Client disconnected");
    
    } else if(type == WS_EVT_DATA){

        // do something based on the input

        switch(data[0]){
            case 'd':
                // send data
            case 'f':
                // fire the catapult
                allTime = micros();
                Serial.println("catapult");
        }
    }
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
        Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }

    bno.setExtCrystalUse(true);

    /* 
        Servo
    */
    dihedral_servo.attach(dihedral_pin);

    /*
        Web
    */
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi..");
    }
    
    Serial.println(WiFi.localIP());

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    
    server.begin();
}


void loop() {
    Serial.println("HELOO");

    tStart = micros();

    dihedral_pos += 1;
    dihedral_servo.writeMicroseconds(dihedral_pos);
    if (dihedral_pos == 2500) {
        dihedral_pos = 1000;
    }

    sensor_data();
    
    while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000)) {
        //poll until the next sample is ready
    }
}