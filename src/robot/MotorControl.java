//package robot;

import lejos.hardware.motor.Motor;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;

public class MotorControl {

    public static void main(String[] args) {
        
        // Set the motors to rotate
        Motor.A.forward();                          // Start motor A
        Motor.B.forward();                          // Start motor B
        
        LCD.clear();
        LCD.drawString("Spinning motors.", 0, 0);

        Motor.A.setSpeed(300);                      // Motor speed: 200 degrees/sec
        Motor.B.setSpeed(300);              

        Delay.msDelay(5000);                        // inserting 5 sec delay
        
        Motor.A.stop(true);
        Motor.B.stop(true);

        LCD.clear();
        LCD.drawString("Turning back", 0, 0);
        Motor.A.rotate(400, true);  // Rotate motor A forward
        Motor.B.rotate(-400);       // Rotate motor B backward

        Motor.A.forward();                          
        Motor.B.forward();

        LCD.clear();
        LCD.drawString("Spinning motors.", 0, 0);

        Motor.A.setSpeed(300);                      // Motor speed: 200 degrees/sec
        Motor.B.setSpeed(300);              

        Delay.msDelay(3000);                        // inserting 5 sec delay
        
        Motor.A.stop(true);
        Motor.B.stop(true);
        // LCD.clear();
        LCD.drawString("Motors stopped.", 0, 1);
        
        Button.waitForAnyPress();
    }
}
