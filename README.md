# EV3 Robot Line Follower Project

## Overview

This project implements a line-following and obstacle-avoiding robot using the LEGO EV3 platform and the LeJOS Java library.  
The robot uses a color sensor to detect and follow a line, and an ultrasonic sensor to detect and avoid obstacles dynamically.  
The system is multi-threaded and features a robust recovery mechanism for line loss.

## Features

- **PID-controlled line following**: Uses a PID controller for smooth and accurate line tracking.
- **Obstacle detection and avoidance**: Detects obstacles and executes intelligent avoidance maneuvers.
- **Multi-threaded operation**: Separate threads for line following and obstacle detection.
- **Recovery system**: Smart recovery strategies if the robot loses track of the line.
- **Dynamic edge switching**: Switches between inner and outer edges for better recovery after avoidance.

## Hardware Requirements

- LEGO EV3 Brick
- 2 Large Motors (connected to ports A and B)
- EV3 Color Sensor (connected to port S1)
- EV3 Ultrasonic Sensor (connected to port S2)

## Project Structure

```
EV3LineFollower/
│
├── bin/                  # Compiled .class files
│   ├── robot/
│       ├── Constants.class
│       ├── LineFollower.class
│       ├── LineSensorTask.class
│       ├── ObstacleAvoidTask.class
│       └── RobotController.class
│
├── lib/
│   └── ev3classes.jar    # LeJOS EV3 library
│
├── src/
│   └── robot/            # Java source files
│       ├── Constants.java
│       ├── LineFollower.java
│       ├── LineSensorTask.java
│       ├── ObstacleAvoidTask.java
│       └── RobotController.java
│
├── build.properties      # Build configuration
├── build.xml             # Ant build script
├── compile.bat           # Compilation script
├── package.bat           # JAR packaging script
└── linefollower.jar      # Output JAR file (generated after packaging)
```

## Source Code Components

### Constants.java

Defines all configuration constants used throughout the project, including:

- PID controller parameters (`Kp`, `Ki`, `Kd`)
- Speed settings for different operations
- Sensor threshold values
- Timing parameters for turns, delays, and forward movement during obstacle avoidance

### LineFollower.java

The main class that initializes sensors, creates task threads, and manages the robot's application lifecycle:

- Initializes the color and ultrasonic sensors
- Creates and starts separate threads for line following and obstacle avoidance
- Manages safe shutdown when the program ends

### LineSensorTask.java

Implements the line-following logic using PID control:

- Reads light intensity from the color sensor
- Calculates motor power adjustments with a PID algorithm
- Implements a recovery strategy when the robot loses the line
- Displays real-time debug information on the EV3 LCD display

### ObstacleAvoidTask.java

Handles obstacle detection and avoidance:

- Continuously checks ultrasonic sensor distance
- When an obstacle is detected, temporarily halts line following
- Executes a pre-programmed avoidance routine: turns, curves, and forward movement
- Helps the robot find and return to the line post-avoidance

### RobotController.java

Provides shared methods and state management for the robot:

- Manages motor speed, stop/start controls
- Coordinates between line-following and obstacle-avoidance actions
- Controls switching between inner/outer edge following modes
- Handles emergency stop triggered by the EV3 ESCAPE button

## How It Works

1. The robot follows a black line using the color sensor to detect brightness levels.
2. A PID controller adjusts the left and right motor speeds based on sensor feedback to maintain line tracking.
3. If the robot strays too far and loses the line, a recovery strategy attempts to re-locate it.
4. If an obstacle appears within a predefined distance, the obstacle avoidance routine temporarily pauses line following.
5. After successfully avoiding the obstacle, the robot resumes normal line-following behavior.

## PID Controller

The PID controller continuously adjusts the robot's steering based on three calculated terms:

- **Proportional (P)**: Corrects based on the present error.
- **Integral (I)**: Corrects based on accumulated past errors.
- **Derivative (D)**: Predicts and corrects based on future trends (rate of error change).

These parameters are customizable in `Constants.java` to fine-tune the robot’s responsiveness for different environments.

## Building and Running

1. Install the LeJOS EV3 development environment and Java SDK.
2. Connect to your EV3 brick via USB, Wi-Fi, or Bluetooth.
3. Open the project folder (`EV3LineFollower/`) in Visual Studio Code.
4. Run `compile.bat` to compile all Java files.
5. Run `package.bat` to create the `linefollower.jar`.
6. Transfer `linefollower.jar` to your EV3 brick (via FTP or SD card).
7. On the EV3, execute:

```bash
jrun -jar linefollower.jar
```

## Terminating the Program

Press the **ESCAPE** button on the EV3 brick at any time to safely stop all motors and terminate the program.

## Customization

You can easily customize robot behavior by modifying values in `Constants.java`:

- Adjust PID parameters (`Kp`, `Ki`, `Kd`) for sharper or smoother line following.
- Tune speed settings for different track conditions.
- Modify thresholds to handle various line contrasts or lighting conditions.
- Adjust obstacle avoidance parameters for different obstacle sizes or layouts.
