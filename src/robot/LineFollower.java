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
        // Initialize sensors
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
        EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S2);

        SampleProvider light = colorSensor.getRedMode();
        float[] sample = new float[light.sampleSize()];
        SampleProvider distanceProvider = usSensor.getDistanceMode();
        float[] distanceSample = new float[distanceProvider.sampleSize()];

        // PID Constants (your proven values)
        float Kp = 250f;
        float Ki = 5f;
        float Kd = 800f;

        // Control Variables
        float error, lastError = 0, integral = 0, derivative;
        float baseSpeed = 150;
        float target = 0.2f;
        
        // Recovery thresholds
        float outerThreshold = 0.05f;    // Too far outside line
        float innerThreshold = 0.35f;    // Too far inside line
        int recoveryAttempts = 0;        // Counter for recovery attempts

        while (!Button.ESCAPE.isDown()) {
            // Obstacle detection
            distanceProvider.fetchSample(distanceSample, 0);
            float distance = distanceSample[0];
            if (distance < 0.05f) {
                Motor.A.stop(true);
                Motor.B.stop();
                LCD.clear();
                LCD.drawString("Obstacle!", 0, 1);
                performAvoidance();
                integral = 0;
                lastError = 0;
                recoveryAttempts = 0;
                continue;
            }

            // Get sensor reading
            light.fetchSample(sample, 0);
            float value = sample[0];

            // Enhanced recovery system
            if (value < outerThreshold || value > innerThreshold) {
                recoveryAttempts++;
                
                // Display recovery direction
                LCD.drawString("Recovering " + (value < outerThreshold ? "OUT" : "IN"), 0, 1);
                
                // Every 3rd attempt, move forward instead of turning
                if (recoveryAttempts % 3 == 0) {
                    // Forward burst to break spinning
                    Motor.A.setSpeed(baseSpeed);
                    Motor.B.setSpeed(baseSpeed);
                } else {
                    // Normal corrective turn
                    if (value < outerThreshold) {
                        // Too far outside - turn left
                        Motor.A.setSpeed(200);
                        Motor.B.setSpeed(50);
                    } else {
                        // Too far inside - turn right
                        Motor.A.setSpeed(50);
                        Motor.B.setSpeed(200);
                    }
                }
                
                Motor.A.forward();
                Motor.B.forward();
                Delay.msDelay(100);
                
                // Reset PID terms
                integral = 0;
                lastError = 0;
                continue;
            } else {
                // Reset counter when properly aligned
                recoveryAttempts = 0;
            }

            // Normal PID line following
            error = target - value;
            integral += error;
            derivative = error - lastError;
            float turn = Kp * error + Ki * integral + Kd * derivative;

            float leftSpeed = Math.max(50, baseSpeed + turn);
            float rightSpeed = Math.max(50, baseSpeed - turn);
            
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

    // Original avoidance routine
    private static void performAvoidance() {
        int turnSpeed = 200;
        int forwardSpeed = 350;
        int turnDuration = 700;
        int forwardDuration = 1000;

        // Right Turn 90°
        Motor.A.setSpeed(turnSpeed);
        Motor.B.setSpeed(turnSpeed);
        Motor.A.forward();
        Motor.B.backward();
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
        Motor.A.backward();
        Motor.B.forward();
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
        Motor.A.backward();
        Motor.B.forward();
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
        Motor.A.forward();
        Motor.B.backward();
        Delay.msDelay(turnDuration);
        Motor.A.stop(true);
        Motor.B.stop();
        Delay.msDelay(100);
    }
}