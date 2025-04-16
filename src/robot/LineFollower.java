package robot;

import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollower {

    public static void main(String[] args) {
        // Sensor setup
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
        SampleProvider light = colorSensor.getRedMode();
        float[] sample = new float[light.sampleSize()];

        // PID Constants (Tune these values for best performance)
        float Kp = 500f;  // Proportional
        float Ki = 0f;    // Integral (can start with 0)
        float Kd = 1000f; // Derivative

        // PID variables
        float error, lastError = 0;
        float integral = 0, derivative;
        float baseSpeed = 200; // base motor speed
        float target = 0.2f;   // target light value (edge between black and white)

        // Main loop
        while (!Button.ESCAPE.isDown()) {
            light.fetchSample(sample, 0);
            float value = sample[0];

            // Calculate PID terms
            error = target - value;
            integral += error;
            derivative = error - lastError;
            float turn = Kp * error + Ki * integral + Kd * derivative;

            // Calculate motor speeds
            float leftSpeed = baseSpeed + turn;
            float rightSpeed = baseSpeed - turn;

            // Clamp speeds to valid range (0–900 for EV3)
            leftSpeed = Math.max(0, Math.min(900, leftSpeed));
            rightSpeed = Math.max(0, Math.min(900, rightSpeed));

            // Set motor speeds
            Motor.A.setSpeed((int) leftSpeed);
            Motor.B.setSpeed((int) rightSpeed);
            Motor.A.forward();
            Motor.B.forward();

            // Debug output
            LCD.clear();
            LCD.drawString("Light: " + (int)(value * 100) + "%", 0, 0);
            LCD.drawString("Turn: " + (int)turn, 0, 1);

            // Prepare for next loop
            lastError = error;

            Delay.msDelay(30);  // PID loop delay
        }

        // Stop motors and cleanup
        Motor.A.stop();
        Motor.B.stop();
        colorSensor.close();
    }
}
