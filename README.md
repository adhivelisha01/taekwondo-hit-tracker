# Wireless Taekwondo Hit-Tracking System

A wearable system that detects and reports strikes in taekwondo. IMU
tracker nodes worn by the player stream impact data to a central
controller over ESP-NOW, which displays the hits.

## How it works
- Multiple ESP32 tracker nodes read motion data from IMUs (accelerometer/gyro) over I2C
- Each node detects a hit and sends the event wirelessly over ESP-NOW
- A central ESP32 controller receives the events and shows them on a display

## Hardware
- ESP32 (tracker nodes + central controller)
- IMU sensors (MPU-6050 / LSM6-series)
- Battery power for the wearable nodes

## My contribution
This project was built together with a senior. My role was on the
software and debugging side:
- Error correction and code rectification across the tracker/controller firmware
- Diagnosing the intermittent loss of wireless connection — the main
  reliability problem — and identifying it as a power issue: current
  spikes from the radio were starving the ESP32 on weak batteries
- Using AI tools to speed up debugging and narrow down the root cause

## Status
Working prototype.
