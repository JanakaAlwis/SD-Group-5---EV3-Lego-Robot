package robot;

import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.motor.Motor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class ObstacleAvoidTask implements Runnable {
    private final EV3UltrasonicSensor usSensor;
    private final EV3ColorSensor colorSensor;

    public ObstacleAvoidTask(EV3UltrasonicSensor usSensor, EV3ColorSensor colorSensor) {
        this.usSensor = usSensor;
        this.colorSensor = colorSensor;
    }

    @Override
    public void run() {
        SampleProvider distanceProvider = usSensor.getDistanceMode();
        float[] distanceSample = new float[distanceProvider.sampleSize()];

        while (RobotController.isRunning()) {
            distanceProvider.fetchSample(distanceSample, 0);
            float distance = distanceSample[0]; // in meters

            if (distance <= Constants.CRITICAL_RANGE) {
                RobotController.setPerformingAvoidance(true);
                RobotController.stopMotors();
                LCD.clear();
                LCD.drawString("Obstacle!", 0, 1);

                performAvoidance(distanceProvider);

                RobotController.setPerformingAvoidance(false);
            } else if (distance <= Constants.BUFFER_RANGE) {
                RobotController.setPerformingAvoidance(false);
                RobotController.setMotorSpeeds(Constants.AVOIDANCE_SLOW_SPEED, Constants.AVOIDANCE_SLOW_SPEED);
            } else {
                RobotController.setPerformingAvoidance(false);
            }

            Delay.msDelay(Constants.controlLoopDelay);
        }
    }

    private void performAvoidance(SampleProvider distanceProvider) {
        float[] distSample = new float[distanceProvider.sampleSize()];

        // 1. Pivot Right Turn Until Obstacle is Cleared
        LCD.drawString("Turning Right", 0, 2);
        Motor.A.setSpeed(Constants.AVOIDANCE_TURN_SPEED_RIGHT); // Left motor forward
        Motor.B.setSpeed(Constants.AVOIDANCE_TURN_SPEED_LEFT);  // Right motor backward
        Motor.A.forward();
        Motor.B.backward();

        while (true) {
            distanceProvider.fetchSample(distSample, 0);
            float dist = distSample[0];
            if (dist > Constants.BUFFER_RANGE) {
                break;
            }
            Delay.msDelay(Constants.controlLoopDelay);
        }

        Motor.A.stop(true);
        Motor.B.stop();
        Delay.msDelay(100);

        // 2. Move Forward a Bit to Pass Obstacle
        LCD.drawString("Going Forward", 0, 3);
        int calibratedForwardTimeMs = Constants.AVOIDANCE_FORWARD_MS;
        Motor.A.setSpeed(Constants.forwardSpeed);
        Motor.B.setSpeed(Constants.forwardSpeed);
        Motor.A.forward();
        Motor.B.forward();
        Delay.msDelay(calibratedForwardTimeMs);
        Motor.A.stop(true);
        Motor.B.stop();
        Delay.msDelay(100);

        // 3. Temporarily switch to inner (right) edge following
        RobotController.setOuterEdgeFollowing(false);

        // 4. Curve left gently to find the line
        LCD.drawString("Finding Line", 0, 4);
        SampleProvider colorProvider = colorSensor.getRedMode();
        float[] colorSample = new float[colorProvider.sampleSize()];

        int leftCurveSpeed = (int)(Constants.AVOIDANCE_TURN_SPEED_RIGHT * 0.98);
        int rightCurveSpeed = Constants.AVOIDANCE_TURN_SPEED_RIGHT;
        Motor.A.setSpeed(leftCurveSpeed);
        Motor.B.setSpeed(rightCurveSpeed);
        Motor.A.forward();
        Motor.B.forward();

        while (true) {
            colorProvider.fetchSample(colorSample, 0);
            float lightValue = colorSample[0];
            if (lightValue < Constants.innerThreshold + 0.02f) {
                break;
            }
            Delay.msDelay(Constants.COLOR_SAMPLE_INTERVAL);
        }

        Motor.A.stop(true);
        Motor.B.stop();
        Delay.msDelay(50);

        // 5. Switch back to original left edge following
        RobotController.setOuterEdgeFollowing(true);
    }
}
