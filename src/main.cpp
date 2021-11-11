#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

AsyncWebServer server(80);
const char* ssid = "SKY938BB";    // Your WiFi SSID
const char* password = "FNLBCPFYBY";  // yes i realise this is public :) Your WiFi Password

/* Message callback of WebSerial */
void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);
}

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);

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
    // put your main code here, to run repeatedly:
    // WebSerial.println("Hello!");
    sleep(1);
    WebSerial.println("hello");

}