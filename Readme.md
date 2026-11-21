# Object Handling Rover with Robotic Arm

A semi-autonomous mobile rover designed for object detection, obstacle avoidance, and lightweight object manipulation (up to ~200 g) using a dual-controller architecture (**Raspberry Pi Zero 2 W** + **STM32 Blue Pill**). Developed as part of **21EIC301P – Embedded System Design** at SRM Institute of Science & Technology.

## System Architecture

- **Perception & Decision Layer (Raspberry Pi Zero 2 W + OV5647 Camera):** Captures live video frames and uses **OpenCV** (with optional **TensorFlow Lite** support) to detect target objects and send single-character UART commands (`F`, `B`, `L`, `R`, `S`, `P`) to the STM32.
- **Control & Actuation Layer (STM32 Blue Pill):** Runs embedded C (STM32CubeIDE / HAL) to drive **4 BO-geared DC motors** through an **L293D** motor driver, monitor distance via **HC-SR04** ultrasonic sensor(s) for collision avoidance, and control a **4-DOF robotic arm** (base, shoulder, elbow, gripper) using PWM signals.
- **Power Management:** Rechargeable lithium-ion battery pack with voltage regulation supplying 5 V to the Raspberry Pi and 3.3 V to the STM32 (~40 minutes continuous operation in testing).

## Command Protocol (UART)

| Command | Action |
| :--- | :--- |
| `F` | Move Forward |
| `B` | Move Backward |
| `L` | Turn Left |
| `R` | Turn Right |
| `S` | Stop Motors |
| `P` | Execute Pick-and-Place Sequence |

## Hardware Summary

- **Compute:** Raspberry Pi Zero 2 W, STM32 Blue Pill (STM32F103)
- **Vision & Sensors:** OV5647 Camera Module, HC-SR04 Ultrasonic Sensor(s)
- **Actuators & Drivers:** 4x BO-Geared DC Motors, L293D Motor Driver, 4x Servo Motors (4-DOF Arm)

## Authors & Guidance

- **Team:** Srija P, Hisham B
- **Guide:** Dr. J. Sam Jebakumar, Dept. of Electronics and Instrumentation Engineering, SRMIST
