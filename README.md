# Object Detection and Sorting: A Braccio Arm Project

This repository contains the full project report and Arduino source code for an autonomous robotic system developed for the **UCL Engineering Foundation Year**. The project demonstrates the integration of sensing, actuation, and decision-making to create a system that can detect, retrieve, and sort objects based on their colour.

## Table of Contents
- [Overview](#overview)
- [How It Works](#how-it-works)
- [Hardware](#hardware)
- [Software & Libraries](#software--libraries)
- [Custom Modifications](#custom-modifications)
- [Repository Contents](#repository-contents)
- [Contributors](#contributors)

## Overview

The goal of this project was to augment a TinkerKit Braccio robotic arm to perform an automated sorting task. The final system is capable of scanning its environment, locating a coloured block, picking it up, identifying its colour, and placing it into the correct corresponding bin.

This was achieved by integrating ultrasonic and colour sensors with the arm, and developing control software that uses an inverse kinematics library for precise path planning and movement.

## How It Works

The operational logic follows a simple, repeatable loop:

1.  **Find:** The HC-SR04 ultrasonic sensor, mounted on the arm's base, performs a 180-degree radar-like sweep to find the nearest object within a 25cm range.
2.  **Locate:** Once an object is detected, its angle and distance are converted into (x, y) coordinates.
3.  **Grab:** The system uses an inverse kinematics library to translate the (x, y) coordinates into the precise servo angles needed to move the end-effector and pick up the object.
4.  **Scan Colour:** The arm moves the object over a TCS3200 colour sensor to identify it as Red, Green, or Blue.
5.  **Sort:** Based on the colour detected, the arm moves to the predefined coordinates of the correct bin and drops the object.
6.  **Reset:** The arm returns to its home position and the loop begins again.

![Program Flow Chart](https://i.imgur.com/gKj3tQd.png)

## Hardware

| Component              | Function                               |
| ---------------------- | -------------------------------------- |
| **Arduino UNO**        | Microcontroller / Brain of the system  |
| **TinkerKit Braccio**  | 6-axis robotic arm                     |
| **HC-SR04 Sensor**     | Ultrasonic distance sensing            |
| **TCS3200 Sensor**     | Colour identification                  |

A full wiring schematic can be found in the project report.

## Software & Libraries

The control logic is written in C++ for the Arduino platform.

-   **`<Braccio.h>`**: The official library for controlling the Braccio arm's servos.
-   **`<InverseK.h>`**: A third-party library used to solve inverse kinematics, allowing for coordinate-based movement. [Link to Library](https://github.com/cgxeiji/CGx-InverseK).
-   **`<Servo.h>`**: Standard Arduino library for servo control.

## Custom Modifications

To improve the system's reliability and performance, several physical enhancements were made:

-   **Custom Base:** A laser-cut wooden base was designed to securely house the Arduino, breadboard, and wiring, while also providing a stable platform for the arm.
-   **3D-Printed Sensor Bracket:** A custom bracket was designed and 3D-printed to securely mount the HC-SR04 sensor to the arm's rotating base.
-   **Enhanced Gripper:** A rubber grip was added to the end-effector's claws to dramatically improve its ability to reliably grasp and hold the acrylic blocks.

## Repository Contents

-   `Braccio_Arm_Object_Detection_and_Sorting.pdf`: The complete project report, including system design, implementation details, evaluation, and conclusions.
-   `BraccioController.ino`: The complete, commented Arduino source code for the project.

## Contributors

* Jason Mitchell
* William Stephen
* Darmekan Tharmapalan
* Erik Spasov