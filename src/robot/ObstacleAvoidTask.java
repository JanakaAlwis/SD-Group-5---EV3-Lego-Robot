// ObstacleAvoidTask thread: Monitors ultrasonic sensor to detect obstacles and triggers avoidance behavior
package robot;

import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.motor.Motor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

// ObstacleAvoidTask class: Handles obstacle detection and avoidance in a separate thread by implementing Runnable
public class ObstacleAvoidTask implements Runnable {
    private final EV3UltrasonicSensor usSensor;
    private final EV3ColorSensor colorSensor;

    // Constructor: Initialize ObstacleAvoidTask with ultrasonic and color sensors
    public ObstacleAvoidTask(EV3UltrasonicSensor usSensor, EV3ColorSensor colorSensor) {
        this.usSensor = usSensor;
        this.colorSensor = colorSensor;
    }

    @Override
    public void run() {
        SampleProvider distanceProvider = usSensor.getDistanceMode(); // Interface to fetch distance readings
        float[] distanceSample = new float[distanceProvider.sampleSize()]; // Container for ultrasonic sensor data

        while (RobotController.isRunning()) {
            distanceProvider.fetchSample(distanceSample, 0);
            float distance = distanceSample[0]; // Distance in meters

            if (distance <= Constants.CRITICAL_RANGE) {
                // Critical obstacle detected
                RobotController.setPerformingAvoidance(true);
                RobotController.stopMotors();
                LCD.clear();
                LCD.drawString("Obstacle!", 0, 1);

                // Perform avoidance maneuver
                performAvoidance(distanceProvider);

                RobotController.setPerformingAvoidance(false);
            } else if (distance <= Constants.BUFFER_RANGE) {
                // Obstacle detected but not critical, slow down
                RobotController.setPerformingAvoidance(false);
                RobotController.setMotorSpeeds(Constants.AVOIDANCE_SLOW_SPEED, Constants.AVOIDANCE_SLOW_SPEED);
            } else {
                // No obstacle nearby, continue normal operation
                RobotController.setPerformingAvoidance(false);
            }

            Delay.msDelay(Constants.controlLoopDelay); // Control loop timing
        }
    }

    // performAvoidance(): Executes a 3-step maneuver to avoid obstacle and return to line
    private void performAvoidance(SampleProvider distanceProvider) {
        float[] distSample = new float[distanceProvider.sampleSize()];

        // 1. Pivot right until obstacle is cleared
        LCD.drawString("Turning Right", 0, 2);
        Motor.A.setSpeed(Constants.AVOIDANCE_TURN_SPEED_RIGHT); // Left motor turn speed
        Motor.B.setSpeed(Constants.AVOIDANCE_TURN_SPEED_LEFT);  // Right motor turn speed
        Motor.A.forward(); // Start left motor moving forward → and the program immediately continues
        Motor.B.backward(); // Start right motor moving backward → and the program immediately continues

        //Motors keep turning while(true) and keep checking distance
        while (true) {
            distanceProvider.fetchSample(distSample, 0);
            float dist = distSample[0];
            if (dist > Constants.BUFFER_RANGE) { 
                break; // Obstacle cleared (If obstacle is gone → break the loop)
            }
            Delay.msDelay(Constants.controlLoopDelay);
        }

        Motor.A.stop(true);
        Motor.B.stop();
        Delay.msDelay(100); // Small pause after turn

        // 2. Move Forward a Bit to Pass Obstacle
        LCD.drawString("Going Forward", 0, 3);
        int calibratedForwardTimeMs = Constants.AVOIDANCE_FORWARD_MS;
        Motor.A.setSpeed(Constants.forwardSpeed);
        Motor.B.setSpeed(Constants.forwardSpeed);
        Motor.A.forward();
        Motor.B.forward();
        Delay.msDelay(calibratedForwardTimeMs); // Forward time duration

        Motor.A.stop(true);
        Motor.B.stop();
        Delay.msDelay(100); // Small pause after forward move

        // 3. Search and rejoin the line
        // 3.1. Temporarily switch to inner (right) edge following
        RobotController.setOuterEdgeFollowing(false);

        // 3.2. Curve left slowly while searching for the line
        LCD.drawString("Finding Line", 0, 4);
        SampleProvider colorProvider = colorSensor.getRedMode(); // Interface used to read sensor data in LeJOS
        float[] colorSample = new float[colorProvider.sampleSize()]; // A container to store the color sensor readings

        int leftCurveSpeed = (int)(Constants.AVOIDANCE_TURN_SPEED_RIGHT * 0.999);
        int rightCurveSpeed = Constants.AVOIDANCE_TURN_SPEED_RIGHT;
        Motor.A.setSpeed(leftCurveSpeed);
        Motor.B.setSpeed(rightCurveSpeed);
        Motor.A.forward(); //Start left motor moving forward → and the program immediately continues
        Motor.B.forward(); // Start right motor moving forward → and the program immediately continues

        // while robot slowly curves to the left, keeps scanning using the color sensor:
        while (true) {
            colorProvider.fetchSample(colorSample, 0);
            float lightValue = colorSample[0];
            if (lightValue < Constants.innerThreshold + 0.02f) {  // 0.02f extra tolerance to detect the black line faster.
                break;
            }
            Delay.msDelay(Constants.COLOR_SAMPLE_INTERVAL); // delay for checking color sensor
        }

        Motor.A.stop(true);
        Motor.B.stop();
        Delay.msDelay(50); // Small pause after detecting line

        // 3.3. Switch back to original outer (left) edge following
        RobotController.setOuterEdgeFollowing(true);
    }
}