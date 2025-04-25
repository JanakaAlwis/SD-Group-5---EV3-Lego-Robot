package robot;

import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineSensorTask implements Runnable {
    private final EV3ColorSensor colorSensor;
    private int recoveryAttempts = 0;
    private float error = 0;
    private float lastError = 0;
    private float integral = 0;
    
    public LineSensorTask(EV3ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
    }
    
    @Override
    public void run() {
        SampleProvider light = colorSensor.getRedMode();
        float[] sample = new float[light.sampleSize()];
        
        while (RobotController.isRunning()) {
            if (RobotController.isPerformingAvoidance()) {
                Delay.msDelay(Constants.controlLoopDelay);
                continue;
            }
            
            light.fetchSample(sample, 0);
            float value = sample[0];

            if (value < Constants.outerThreshold || value > Constants.innerThreshold) {
                recoveryAttempts++;

                boolean isRecoveringOut = (value < Constants.outerThreshold);

                LCD.drawString("Recovering " + (isRecoveringOut ? "OUT" : "IN"), 0, 1);
                LCD.drawString("Edge: " + (RobotController.isOuterEdgeFollowing() ? "OUTER" : "INNER"), 0, 2);

                if (recoveryAttempts % 3 == 0) {
                    RobotController.setMotorSpeeds(Constants.baseSpeed, Constants.baseSpeed);
                } else {
                    if (RobotController.isOuterEdgeFollowing()) {
                        // Black on left
                        if (isRecoveringOut) {
                            RobotController.setMotorSpeeds(200, 50); // veer left
                        } else {
                            RobotController.setMotorSpeeds(50, 200); // veer right
                        }
                    } else {
                        // Black on right (inner edge)
                        if (isRecoveringOut) {
                            RobotController.setMotorSpeeds(50, 200); // veer right
                        } else {
                            RobotController.setMotorSpeeds(200, 50); // veer left
                        }
                    }
                }

                Delay.msDelay(Constants.recoveryDelay);
                integral = 0;
                lastError = 0;
                continue;
            } else {
                recoveryAttempts = 0;
            }


            // Adjust PID error calculation based on edge following mode
            if (RobotController.isOuterEdgeFollowing()) {
                error = Constants.target - value; // Outer edge (black on left)
            } else {
                error = value - Constants.target; // Inner edge (black on right) → inverted
            }
            integral += error;
            float derivative = error - lastError;
            float turn = Constants.Kp * error + Constants.Ki * integral + Constants.Kd * derivative;

            float leftSpeed, rightSpeed;

            // Adjust motor response based on edge being followed
            if (RobotController.isOuterEdgeFollowing()) {
                // Black on left → curve left when error increases
                leftSpeed = Math.max(50, Constants.baseSpeed + turn);    // +turn to go left
                rightSpeed = Math.max(50, Constants.baseSpeed - turn);
            } else {
                // Black on right → curve right when error increases
                leftSpeed = Math.max(50, Constants.baseSpeed - turn);    // -turn to go right
                rightSpeed = Math.max(50, Constants.baseSpeed + turn);
            }
            
            RobotController.setMotorSpeeds(leftSpeed, rightSpeed);

            LCD.clear();
            LCD.drawString("Light: " + (int)(value * 100) + "%", 0, 0);
            LCD.drawString("Turn: " + (int)turn, 0, 1);

            lastError = error;
            Delay.msDelay(Constants.controlLoopDelay);
        }
    }
}