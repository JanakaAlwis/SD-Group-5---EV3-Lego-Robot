package robot;

import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;

public class TestMotors3 {
    public static void main(String[] args) {

        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S2);
        SampleProvider distance = ultrasonicSensor.getDistanceMode();
        float[] sample = new float[distance.sampleSize()];

        // Set speed and move straight for 10 seconds
        Motor.A.setSpeed(300); // Speed in degrees/sec
        Motor.B.setSpeed(300);
        Motor.A.forward();
        Motor.B.forward();

        while (true) {
            // Get the curRent distnce reading from the US sensor
            distance.fetchSample(sample, 0);
            float distance1 = sample[0];
            
            // Display the distance on the LCD screen
            LCD.clear();
            LCD.drawString("Dist: " + sample[0] + " meters", 0, 0);

            if (distance1 <= 0.05) {
                Motor.A.stop(true);
                Motor.B.stop(true);
                break;
            }
        }
        ultrasonicSensor.close();

               // Perform a 360-degree turn
               LCD.clear();
               LCD.drawString("Turning 360", 0, 0);
               Motor.A.rotate(360, true);  // Rotate forward
               Motor.B.rotate(-360);       // Rotate backward
               
               // Display halfway message
               LCD.clear();
               LCD.drawString("Reached halfway", 0, 0);
               LCD.drawString("Returning to base", 0, 1);
               Delay.msDelay(3000);
                      
               // Move back to start position
               Motor.A.setSpeed(400); // Corrected: Replacing setPower with setSpeed
               Motor.B.setSpeed(400);
               Motor.A.forward();
               Motor.B.forward();
               Delay.msDelay(5000); // Move for 10 seconds
               
               Motor.A.stop(true);
               Motor.B.stop(true);
               
               // Stop and display message
               LCD.clear();
               LCD.drawString("Motors stopped.", 0, 1);
               
               // Wait for button press to exit
               Button.waitForAnyPress();
    }
}