package robot;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollower {

    public static void main(String[] args) {
        // Sensors
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
        EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S2);

        SampleProvider light = colorSensor.getRedMode();
        float[] sample = new float[light.sampleSize()];
        SampleProvider distanceProvider = usSensor.getDistanceMode();
        float[] distanceSample = new float[distanceProvider.sampleSize()];

        // PID Constants
        float Kp = 250f;
        float Ki = 5f;
        float Kd = 800f;

        // PID Variables
        float error, lastError = 0, integral = 0, derivative;
        float baseSpeed = 150;
        float target = 0.2f;

        while (!Button.ESCAPE.isDown()) {
            // Check distance
            distanceProvider.fetchSample(distanceSample, 0);
            float distance = distanceSample[0];

            // If obstacle detected, stop and perform avoidance
            if (distance < 0.05f) {
                Motor.A.stop(true);
                Motor.B.stop();
                LCD.clear();
                LCD.drawString("Obstacle!", 0, 1);

                performAvoidance();

                // Reset PID after maneuver
                integral = 0;
                lastError = 0;
                continue;
            }

            // PID Line Following
            light.fetchSample(sample, 0);
            float value = sample[0];

            error = target - value;
            integral += error;
            derivative = error - lastError;
            float turn = Kp * error + Ki * integral + Kd * derivative;

            float leftSpeed = baseSpeed + turn;
            float rightSpeed = baseSpeed - turn;

            // Clamp speed/ avoid exceed
            leftSpeed = Math.max(0, Math.min(400, leftSpeed));
            rightSpeed = Math.max(0, Math.min(400, rightSpeed));

            // Apply speeds
            Motor.A.setSpeed((int) leftSpeed);
            Motor.B.setSpeed((int) rightSpeed);
            Motor.A.forward();
            Motor.B.forward();

            LCD.clear();
            LCD.drawString("Light: " + (int)(value * 100) + "%", 0, 0);
            LCD.drawString("Turn: " + (int)turn, 0, 1);

            lastError = error;
            Delay.msDelay(20);
        }

        Motor.A.stop();
        Motor.B.stop();
        colorSensor.close();
        usSensor.close();
    }

    private static void performAvoidance() {
        int turnSpeed = 200;
        int forwardSpeed = 350;
    
        // Approx timings (tuned for 90-degree turns and 1 robot-length forward)
        int turnDuration = 700;     // ~90 degree turn
        int forwardDuration = 1000;  // Move forward 1 robot length
    
        // Right Turn 90°
        Motor.A.setSpeed(turnSpeed);
        Motor.B.setSpeed(turnSpeed);
        Motor.A.forward(); // Left motor forward
        Motor.B.backward(); // Right motor backward
        Delay.msDelay(turnDuration);
        Motor.A.stop(true);
        Motor.B.stop();
        Delay.msDelay(100);
    
        // Forward
        Motor.A.setSpeed(forwardSpeed);
        Motor.B.setSpeed(forwardSpeed);
        Motor.A.forward();
        Motor.B.forward();
        Delay.msDelay(forwardDuration);
        Motor.A.stop(true);
        Motor.B.stop();
        Delay.msDelay(100);
    
        // Left Turn 90°
        Motor.A.setSpeed(turnSpeed);
        Motor.B.setSpeed(turnSpeed);
        Motor.A.backward(); // Left motor backward
        Motor.B.forward();  // Right motor forward
        Delay.msDelay(turnDuration);
        Motor.A.stop(true);
        Motor.B.stop();
        Delay.msDelay(100);
    
        // Forward
        Motor.A.setSpeed(forwardSpeed);
        Motor.B.setSpeed(forwardSpeed);
        Motor.A.forward();
        Motor.B.forward();
        Delay.msDelay(forwardDuration);
        Motor.A.stop(true);
        Motor.B.stop();
        Delay.msDelay(100);
    
        // Left Turn 90°
        Motor.A.setSpeed(turnSpeed);
        Motor.B.setSpeed(turnSpeed);
        Motor.A.backward(); // Left motor backward
        Motor.B.forward();  // Right motor forward
        Delay.msDelay(turnDuration);
        Motor.A.stop(true);
        Motor.B.stop();
        Delay.msDelay(100);
    
        // Forward
        Motor.A.setSpeed(forwardSpeed);
        Motor.B.setSpeed(forwardSpeed);
        Motor.A.forward();
        Motor.B.forward();
        Delay.msDelay(forwardDuration);
        Motor.A.stop(true);
        Motor.B.stop();
        Delay.msDelay(100);
    
        // Right Turn 90° to rejoin line
        Motor.A.setSpeed(turnSpeed);
        Motor.B.setSpeed(turnSpeed);
        Motor.A.forward();  // Left motor forward
        Motor.B.backward(); // Right motor backward
        Delay.msDelay(turnDuration);
        Motor.A.stop(true);
        Motor.B.stop();
        Delay.msDelay(100);
    }
    
}
