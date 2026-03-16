# RTOS-Based Smart Parking Gate System on ESP32

## Overview
This project is a real-time smart parking gate system built using **ESP32** and **FreeRTOS**.  
It detects vehicles using an ultrasonic sensor and automatically opens or closes a servo-controlled barrier.  
The system includes emergency override functionality and traffic light signaling.

##  Demo Video

This video demonstrates the RTOS-based smart parking gate system running on ESP32.

https://github.com/mariemh25/rtos-smart-parking-gate-esp32/blob/main/video5891117627278891907.mp4

## Features
- Vehicle detection using ultrasonic sensor
- Automatic gate control using servo motor
- Emergency override button using interrupt
- Traffic light LEDs (red/green)
- FreeRTOS multitasking system
- Task communication using queues
- Synchronization using semaphores and mutexes

## Hardware
- ESP32
- Ultrasonic Sensor
- Servo Motor
- Push Button
- Red LED
- Green LED

## RTOS Tasks
- Vehicle Detection Task
- Authorization Task
- Gate Control Task
- Emergency Task
- Traffic Light Task

## Simulation
The system was first tested using the **Wokwi simulation platform** before implementing it on real ESP32 hardware.
https://wokwi.com/projects/449791207883021313

## Technologies
- ESP32
- Arduino IDE
- FreeRTOS
- Embedded C/C++

## Author
Mariem H
