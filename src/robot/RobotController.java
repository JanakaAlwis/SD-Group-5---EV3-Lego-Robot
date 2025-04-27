// This class is the central control unit - manages the state of the robot, controls motor speeds, 
// and handles safety and thread coordination between the line-following and obstacle-avoidance threads.

package robot;

import lejos.hardware.motor.Motor;
import lejos.hardware.Button;

public class RobotController {
    // Preserving control flow variables (Shared Across Threads)
    private static volatile boolean running = true; // volatile - if one thread updates a variable, other threads see the updated value immediately
    private static volatile boolean performingAvoidance = false;
    private static boolean outerEdgeFollowing = true;
    
    // Motor control speed variables
    private static volatile float leftSpeed = 0;
    private static volatile float rightSpeed = 0;
    
    // Running State
    public static boolean isRunning() {
        return running && !Button.ESCAPE.isDown();
    }
    
    // getter(check) - Obstacle Avoidance Status
    public static boolean isPerformingAvoidance() {
        return performingAvoidance;
    }

    // setter(update) - Obstacle Avoidance Status
    public static void setPerformingAvoidance(boolean avoiding) {
        performingAvoidance = avoiding;
    }

    // outerEdgeFollowing = true → robot follows outer edge (black on left)
    // outerEdgeFollowing = false → robot follows inner edge (black on right)

    //  getter(check) - Check if robot is following outer edge (outer - black / inner - white)
    public static boolean isOuterEdgeFollowing() {
        return outerEdgeFollowing;
    }

    // setter(update) - Set whether robot is following outer edge
    public static void setOuterEdgeFollowing(boolean isOuter) {
        outerEdgeFollowing = isOuter;
    }

    // Motor Speed Control - Only sets motor speeds if the robot is not performing obstacle avoidance
    public static void setMotorSpeeds(float left, float right) {
        if (!performingAvoidance) { // shared flag - If true, the method does nothing
            leftSpeed = left;
            rightSpeed = right;
            Motor.A.setSpeed((int) leftSpeed);
            Motor.B.setSpeed((int) rightSpeed);
            Motor.A.forward();
            Motor.B.forward();
        }
    }
    
    // Immediate Motor Stop
    public static void stopMotors() {
        Motor.A.stop(true);
        Motor.B.stop();
    }

    // Stops the robot completely by setting running to false and stopping both motors.
    public static void stop() {
        running = false;
        Motor.A.stop();
        Motor.B.stop();
    }
}