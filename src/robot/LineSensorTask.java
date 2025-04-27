// // LineSensorTask thread: Continuously monitors the color sensor and controls motor speeds using PID-based line following
package robot;

import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

// LineSensorTask class: Handles line-following logic in a separate thread by implementing Runnable
public class LineSensorTask implements Runnable { 
    private final EV3ColorSensor colorSensor;
    private int recoveryAttempts = 0;
    private float error = 0;
    private float lastError = 0;
    private float integral = 0;
    
    // Constructor: initialize LineSensorTask with a color sensor
    public LineSensorTask(EV3ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
    }
    
    @Override
    public void run() {
        SampleProvider light = colorSensor.getRedMode(); // Interface used to read sensor data in LeJOS, Gets the red light intensity mode of the color sensor
        float[] sample = new float[light.sampleSize()]; // A container to store the color sensor readings
        
        while (RobotController.isRunning()) {
            if (RobotController.isPerformingAvoidance()) { // Pause PID while obstacle avoidance is active, otherwise continue
                Delay.msDelay(Constants.controlLoopDelay); // Avoid conflict, give time and efficient loop
                continue;   // Skip the rest of the code in the current loop
            }
            // Get sensor reading
            light.fetchSample(sample, 0);
            float value = sample[0];

            // Recovery system - Triggered when light value is too far from expected range
            if (value < Constants.outerThreshold || value > Constants.innerThreshold) {
                recoveryAttempts++;

                boolean isRecoveringOut = (value < Constants.outerThreshold); // too much into the black (true/false)

                // Display recovery direction
                LCD.drawString("Recovering " + (isRecoveringOut ? "OUT" : "IN"), 0, 1);
                LCD.drawString("Edge: " + (RobotController.isOuterEdgeFollowing() ? "OUTER" : "INNER"), 0, 2);

                // Every 3rd attempt, move forward instead of turning
                if (recoveryAttempts % 3 == 0) {
                    // Forward burst to break spinning
                    RobotController.setMotorSpeeds(Constants.baseSpeed, Constants.baseSpeed);
                } else {
                    if (RobotController.isOuterEdgeFollowing()) { // Robot thinks the black track is on its left side
                        // Black on left
                        if (isRecoveringOut) {
                            // Too far left (outside line) - turn right to get back on track
                            // outerThreshold means the robot has gone beyond the ideal line toward the black area (left of the road).
                            RobotController.setMotorSpeeds(200, 50); // veer right
                        } else {
                            // Too far right (inside line) - turn left to get back on track
                            // innerThreshold means the robot has moved toward the white area (right of the road).
                            RobotController.setMotorSpeeds(50, 200); // veer left
                        }
                    } else { //Robot thinks the black track is on its right side
                        // Black on right (inner edge)
                        if (isRecoveringOut) {
                            RobotController.setMotorSpeeds(50, 200); // veer left
                        } else {
                            RobotController.setMotorSpeeds(200, 50); // veer right
                        }
                    }
                }

                Delay.msDelay(Constants.recoveryDelay);
                integral = 0;
                lastError = 0;
                continue;
            } else {
                // Reset counter when properly aligned
                recoveryAttempts = 0;
            }

            // Normal PID line following

            // Adjust PID error calculation based on edge following mode
            if (RobotController.isOuterEdgeFollowing()) {
                error = Constants.target - value; // Outer edge (black on left)
            } else {
                error = value - Constants.target; // Inner edge (black on right) → inverted
            }
            integral += error;  // Accumulates error over time
            float derivative = error - lastError; // Measures how fast the error is changing
            float turn = Constants.Kp * error + Constants.Ki * integral + Constants.Kd * derivative;

            float leftSpeed, rightSpeed;

            // Adjust motor response based on edge being followed
            if (RobotController.isOuterEdgeFollowing()) {
                // Black on left → curve left when error increases
                leftSpeed = Math.max(50, Constants.baseSpeed + turn);    // Left wheel moves more → robot curves left.
                rightSpeed = Math.max(50, Constants.baseSpeed - turn);
            } else {
                // Black on right → curve right when error increases
                leftSpeed = Math.max(50, Constants.baseSpeed - turn);    // Right wheel moves more → robot curves right.
                rightSpeed = Math.max(50, Constants.baseSpeed + turn);
            }
            
            RobotController.setMotorSpeeds(leftSpeed, rightSpeed);

            LCD.clear();
            LCD.drawString("Light: " + (int)(value * 100) + "%", 0, 0);
            LCD.drawString("Turn: " + (int)turn, 0, 1);

            lastError = error;
            Delay.msDelay(Constants.controlLoopDelay); // Avoid conflict, give time and efficient loop
        }
    }
}