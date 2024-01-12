# ✨ ADAS Project

Welcome to our NTI Graduation Project repository 🖐🏻! We are thrilled and proud to present our "Advanced Driver Assistance System" (ADAS), an innovative solution designed to enhance the driving experience and improve safety on the road.

## ⭐Introduction

Our ADAS project aims to revolutionize the way we interact with vehicles, bringing advanced technologies and intelligent features to the forefront of modern driving. With a focus on user-friendly controls, and real-time monitoring.

## ⭐Features and Functionalities

### Three Different Modes of Operation:

- Normal Mode: Control the car's speed and direction using a mobile application while having the option to switch between different modes.
- Adaptive Cruise Control Mode: The car automatically adjusts its speed based on the distance from the object in front of it.
- Smart Driving Mode: The car intelligently slows down when approaching an object, and an ultrasonic sensor scans the environment to select the optimal path.

### Advanced Safety Features:

- Lane Departure Warning (LDW): Alerts the driver when the vehicle deviates from its lane.
- Lane Keep Assist (LKA): Automatically helps the driver to stay within the lane.
- Blind Spot Detection: Warns the driver about vehicles in the blind spots.
- Rain Detection: Detects rain and adapts the car's behavior accordingly.
- Rear Lights System: Enhances visibility and safety while driving in different conditions.

### Mobile Application:

Control various aspects of the car, including speed, direction, and mode selection, through a user-friendly mobile application.

### GUI Dashboard:

A graphical user interface (GUI) represents the car's dashboard, providing real-time information such as distance, speed, and system status.

## ⭐Technologies and Components Used

To bring this project to life, we utilized a combination of cutting-edge technologies and carefully selected components:

- STM32F401RCT6 microcontroller (Cortex M4 ARM-based microprocessor)
- Ultrasonic sensors
- Raspberry Pi (4B)
- Servo motors
- DC motors & Motor Driver
- Rain Sensor

## ⭐Development Tools

For development, we used the STM Cube IDE, a powerful integrated development environment for STM32 microcontrollers.

The system's tasks and scheduling are managed using FreeRTOS.

## ⭐System Architecture

### How Our System Works:

- Communication between the ARM microcontroller and the Raspberry Pi is established using the UART communication protocol.
- The STM32F401RCT6 microcontroller utilizes various peripherals such as a Timer (PWM mode & ICU mode), GPIO, Systick, RCC, NVIC, and USART.
- All system features are controlled through the mobile application.
- The GUI displays real-time information such as distance and speed and other visuals that alert the driver.

## ⭐Project video
you can visualize our system in action by watching this video:

[![Watch the video](https://img.youtube.com/vi/RgO6FYgJRLc/hqdefault.jpg)](https://www.youtube.com/embed/RgO6FYgJRLc)

## ⭐Our Team

we want to express our gratitude to the incredible team behind this endeavor. Your dedication, passion, and collaboration have been instrumental in bringing ADAS to life. Together, we have overcome challenges, shared knowledge, and achieved milestones that we can all be proud of 💪. Thank you for your hard work and commitment to excellence.

Now, let's acknowledge the remarkable individuals who made this project possible :

#### Kirollos Nashaat
#### Abdelrahman Zad
#### Sarah Saeed
#### Salma Sewilam
#### Emad Hamdy
#### Mohammed Maged
#### Hassan Saeed
#### Mohamed Hatem

Thank you for being part of this incredible journey.

