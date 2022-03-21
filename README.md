# Adaptive UAV

ESP32 program to control/log a micro UAV as part of my Aerospace MEng individual project.

## Purpose
- sync actions and data with catapult launch 
- Actuate servos at precise points during flight
- Potentially control pitch
- record/send IMU data -> ultimately to get velocities etc

## components used
- board: https://wiki.dfrobot.com/FireBeetle_Board_ESP32_E_SKU_DFR0654
- IMU: https://www.adafruit.com/product/2472

## library requirements

Main Arduino libraries (PlatformIO)
- IMU: https://github.com/adafruit/Adafruit_BNO055
- Remote Debug: https://github.com/JoaoLopesF/RemoteDebug
- Dynamixel library: https://github.com/ROBOTIS-GIT/Dynamixel2Arduino

Python
- Catapult library https://github.com/o-gent/uav-launcher
