package robot;

import lejos.hardware.motor.Motor;
import lejos.hardware.Button;

public class RobotController {
    // Preserving your control flow variables
    private static volatile boolean running = true;
    private static volatile boolean performingAvoidance = false;
    private static boolean outerEdgeFollowing = true;
    
    // Motor control with your original speed variables
    private static volatile float leftSpeed = 0;
    private static volatile float rightSpeed = 0;
    
    public static boolean isRunning() {
        return running && !Button.ESCAPE.isDown();
    }
    
    public static void stop() {
        running = false;
        Motor.A.stop();
        Motor.B.stop();
    }
    
    public static void setPerformingAvoidance(boolean avoiding) {
        performingAvoidance = avoiding;
    }
    
    public static boolean isPerformingAvoidance() {
        return performingAvoidance;
    }

    public static boolean isOuterEdgeFollowing() {
        return outerEdgeFollowing;
    }

    public static void setOuterEdgeFollowing(boolean isOuter) {
        outerEdgeFollowing = isOuter;
    }

    
    public static void setMotorSpeeds(float left, float right) {
        if (!performingAvoidance) {
            leftSpeed = left;
            rightSpeed = right;
            Motor.A.setSpeed((int) leftSpeed);
            Motor.B.setSpeed((int) rightSpeed);
            Motor.A.forward();
            Motor.B.forward();
        }
    }
    
    public static void stopMotors() {
        Motor.A.stop(true);
        Motor.B.stop();
    }
}