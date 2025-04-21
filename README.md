# 🚦 Real-Time Railway Crossing Controller using STM32F446 & FreeRTOS

A real-time embedded system project that simulates an intelligent railway crossing controller using **STM32F446VET6 MCU** and **FreeRTOS**. It detects train arrival using a sensor, operates gates using GPIO-controlled relays or motors, and logs events through UART — all in a modular RTOS-based architecture.

---

## 📌 Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [RTOS Task Breakdown](#rtos-task-breakdown)
- [Hardware Requirements](#hardware-requirements)
- [Software Stack](#software-stack)
- [Project Structure](#project-structure)
- [Building & Flashing](#building--flashing)
- [Future Improvements](#future-improvements)
- [License] MIT
- [Author] ARAVIND PV
- [Date] 14/04/2025

---

## 🔍 Overview

The project simulates a safety-critical railway crossing gate controller, composed of real-time tasks under **FreeRTOS**. It handles:

- Periodic sensor checking (train detection)
- Timed gate control logic (relay/motor)
- RTC-based timestamping
- Logging and watchdog health monitoring

---


## RTOS Task Breakdown
-Task	Priority	Description
    Sensor Task	High	Triggered every 50ms by TIM2. Detects train and timestamps
    Control Task	High	Manages LED & barrier control via GPIO
    Log Task	Low	Handles logging for diagnostics via UART
    Watchdog	Medium	Ensures Sensor Task runs on time
## Hardware Requirements
- STM32F446VET6 board

- IR or Ultrasonic Sensor (Train detection)

- RTC module 

- Relay or Motor driver 

- LEDs (Red, Green).

- UART interface for logging

- Power source, breadboard, jumper wires

## Software Stack
- STM32CubeIDE

- STM32 HAL Drivers

- FreeRTOS (CMSIS-RTOS v2 interface)

- C language (Embedded C with CMSIS)

## Project Structure

    /Real-Time Railway crossing controller simulation
    ├── Core/
    │   ├── Inc/
    │   │   ├── main.h
    │   │   └── ...
    │   ├── Src/
    │   │   ├── main.c
    │   │   ├── freertos.c
    │   │   └── ...
    ├── Drivers/
    │   └── STM32F4xx_HAL_Driver/
    ├── Middlewares/
    │   └── FreeRTOS/
    ├── STM32F446VET6.ioc
    ├── README.md
    └── DesignReport.pdf
## Building & Flashing

- Open STM32F446VET6.ioc in STM32CubeIDE

- Click Project > Generate Code

- Build the project (Ctrl+B)

- Connect your STM32 board via ST-Link or USB

- Flash the binary (Ctrl+F11)

- Use serial monitor (e.g., PuTTY, TeraTerm) at 115200 baud to view logs


## License
- MIT License — feel free to use, share, or modify this project. See LICENSE for details.