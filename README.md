# EV3 Robot Line Follower Project

## Overview

This project implements a line-following robot using the LEGO EV3 platform and LeJOS Java library. The robot uses color sensors to detect and follow a line while also implementing obstacle avoidance capabilities using an ultrasonic sensor.

## Features

- **PID-controlled line following**: Uses a PID (Proportional-Integral-Derivative) controller to smoothly follow a line
- **Obstacle detection and avoidance**: Detects obstacles in the robot's path and executes a predefined avoidance routine
- **Multi-threaded operation**: Separate threads handle line following and obstacle detection
- **Recovery system**: Implements a recovery strategy when the robot loses track of the line

## Hardware Requirements

- LEGO EV3 Brick
- 2 EV3 Motors (connected to ports A and B)
- EV3 Color Sensor (connected to port S1)
- EV3 Ultrasonic Sensor (connected to port S2)

## Project Structure

```
EV3_PROJECT/
│
├── bin/                  # Compiled .class files
│   └── robot/
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
│   ├── robot/            # Java source files
│   │   ├── Constants.java
│   │   ├── LineFollower.java
│   │   ├── LineSensorTask.java
│   │   ├── ObstacleAvoidTask.java
│   │   └── RobotController.java
│   │
│   ├── compile.bat       # Compilation script
│   └── package.bat       # Jar packaging script
│
├── linefollower.jar      # Output jar (generated after packaging)
│
├── build.properties      # Build configuration
└── build.xml            # Ant build script
```

## Source Code Components

### Constants.java

Defines all configuration constants used throughout the project, including:
- PID controller parameters (Kp, Ki, Kd)
- Speed settings for different operations
- Sensor threshold values
- Timing parameters for various operations

### LineFollower.java

The main class that initializes sensors, creates task threads, and manages the application lifecycle. It:
- Sets up the color sensor and ultrasonic sensor
- Creates and starts separate threads for line following and obstacle avoidance
- Handles cleanup when the program terminates

### LineSensorTask.java

Implements the line following logic using PID control:
- Reads the color sensor values to detect the line
- Calculates PID adjustments to keep the robot on the line
- Implements a recovery system when the robot strays too far from the line
- Displays status information on the EV3 LCD display

### ObstacleAvoidTask.java

Handles obstacle detection and avoidance:
- Continuously monitors the ultrasonic sensor for obstacles
- When an obstacle is detected, executes a predefined avoidance routine
- The avoidance routine follows a pattern of turns and movements to navigate around obstacles and return to the line

### RobotController.java

Provides a shared interface for controlling the robot's motors and tracking the robot's state:
- Manages the running state of the robot
- Controls motor speeds and directions
- Coordinates between line following and obstacle avoidance tasks
- Provides safe stopping functionality

## How It Works

1. The robot follows a line using the color sensor to detect the contrast between the line and the surface
2. A PID controller calculates adjustments to the motor speeds to keep the robot centered on the line
3. If the robot strays too far from the line, a recovery system attempts to find the line again
4. If an obstacle is detected by the ultrasonic sensor, the line following is paused, and the robot executes an avoidance maneuver
5. After avoiding the obstacle, the robot returns to line following mode

## PID Controller

The PID controller uses three parameters to maintain accurate line following:
- **Proportional**: Responds proportionally to the current error
- **Integral**: Accounts for accumulated error over time
- **Derivative**: Responds to the rate of change of error

These parameters can be adjusted in the Constants.java file to optimize performance for different line types and surfaces.

## Building and Running

1. Ensure the LeJOS EV3 environment is installed on your computer
2. Connect to your EV3 brick
3. Run the compile.bat script to compile the Java files
4. Run the package.bat script to create the JAR file
5. Transfer the JAR file to your EV3 brick
6. Run the program on the EV3 brick

## Terminating the Program

Press the ESCAPE button on the EV3 brick to stop the program safely.

## Customization

You can customize the robot's behavior by modifying the constants in Constants.java:
- Adjust PID parameters for better line following
- Modify threshold values for different line types
- Change speed settings for faster/slower operation
- Tune the obstacle avoidance routine timing
