package robot;

public class Constants {
    // Variables Define
    public static final float Kp = 250f;
    public static final float Ki = 5f;
    public static final float Kd = 800f;
    
    public static final float baseSpeed = 200;
    public static final float target = 0.2f;
    public static final float outerThreshold = 0.05f;
    public static final float innerThreshold = 0.35f;
    
    public static final float obstacleDistance = 0.05f;
    public static final int turnSpeed = 200;
    public static final int forwardSpeed = 350;
    public static final int turnDuration = 700;
    public static final int forwardDuration = 1000;
    
    public static final int controlLoopDelay = 10;
    public static final int recoveryDelay = 100;

    public static final float DETECTION_RANGE = 0.40f;   // 40 cm
    public static final float BUFFER_RANGE = 0.30f;      // 30 cm
    public static final float CRITICAL_RANGE = 0.15f;    // 15 cm
    public static final float ROBOT_WIDTH = 0.14f;       // 14 cm
    public static final float ROBOT_LENGTH = 0.27f;      // 27 cm
    public static final int AVOIDANCE_SLOW_SPEED = 200;
    public static final int AVOIDANCE_TURN_SPEED_LEFT = 100;
    public static final int AVOIDANCE_TURN_SPEED_RIGHT = 200;
    public static final int COLOR_SAMPLE_INTERVAL = 15;  // ms delay for checking color sensor

    // Defined a method or constant to estimate robot speed in m/ms
    public static float forwardSpeedInMperMs() {
        // Estimate forward speed of robot in meters per millisecond based on the speed settings
        return 0.00012f; 
    }

    public static final int AVOIDANCE_FORWARD_MS = 1000; // Tuned this lower (~1000ms or less)
}