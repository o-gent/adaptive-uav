
#include <Dynamixel2Arduino.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include "Wire.h"
#include "Adafruit_Sensor.h"
#include <WiFi.h>
#include <DNSServer.h>
#include "ESPmDNS.h"

#define DXL_SERIAL Serial2
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = A4; // DYNAMIXEL Shield DIR PIN

const uint8_t DXL_ID = 2;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup()
{
  DEBUG_SERIAL.begin(230400);
  // put your setup code here, to run once:
  DEBUG_SERIAL.println("yo");
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  Serial.println(dxl.ping(DXL_ID));

  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 0);
}

void loop()
{
  // put your main code here, to run repeatedly:

  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value.
  // Set Goal Position in RAW value
  dxl.setGoalPosition(DXL_ID, 512);

  int i_present_position = 0;
  float f_present_position = 0.0;

  while (abs(512 - i_present_position) > 10)
  {
    f_present_position = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
    i_present_position = dxl.getPresentPosition(DXL_ID);
    DEBUG_SERIAL.print("Present_Position(raw) : ");
    DEBUG_SERIAL.println(i_present_position);
  }
  delay(500);

  // Set Goal Position in DEGREE value
  dxl.setGoalPosition(DXL_ID, 5.7, UNIT_DEGREE);

  while (abs(5.7 - f_present_position) > 2.0)
  {
    f_present_position = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
    i_present_position = dxl.getPresentPosition(DXL_ID);
    DEBUG_SERIAL.print("Present_Position(raw) : ");
    DEBUG_SERIAL.println(i_present_position);
  }
  delay(500);
}