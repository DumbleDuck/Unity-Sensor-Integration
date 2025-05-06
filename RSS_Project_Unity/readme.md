
# Robotic Sensor Systems Project

## Overview

This project is designed to help students learn more about different types of sensors and their applications by integrating Unity 
with a sensors system to control a predeveloped open world game environment. The game includes a robot character, and a house that 
has two rooms and three different doors. 

### Project Structure

#### Unity Scene

Contains a house with several rooms, each with one door. Each door has its respective derived script attached.

## Setup Instructions

### Unity Setup

#### 1. Open the Project in Unity

- Launch Unity Hub.
- Click on "Open" and select the project directory.

#### 2. Review the Scene

- Open the main scene (Playground 1.unity).
- You will see a house with several rooms, each with one door. Each door should have a derived script attached (Door1, Door2, Door3).

## Tasks

### Scripts

The following are the scripts that will be modified by the students.

#### Door Scripts

Located in the `Assets` folder:

- **Door.cs**: The base script containing common door functionalities (should be left unchanged).
- **Door1.cs**: Derived script for the first door.
- **Door2.cs**: Derived script for the second door.
- **Door3.cs**: Derived script for the third door.
- **TemperatureDisplay.cs**: Script for temperature display.

#### Third Person Controller Script

Located under `Assets/UnityTechnologies/StarterAssets/ThirdPersonController/Scripts`:

- **ThirdPersonController.cs**: Character and camera movement script.

### ESP32 Integration

- Implement the ESP32 setup and programming to send appropriate signals to Unity to trigger door operations.
- Modify each derived door script (Door1, Door2, Door3) to respond to unique triggers from the ESP32 microcontroller instead of the "E" key.
- Modify temperature display script to display realtime temperature.
- Modify movement script to control character using ESP32.
