package robot;

// Centralized configuration for easy tuning and clear code
// Utility class that holds static final values (Accessible from any class and value cannot be changed after initialization)
public class Constants {
    // PID tuning parameters - PID Constants (Tested values included based on surface)
    public static final float Kp = 250f; // Proportional gain - Controls the reaction to the current error (how much we correct immediately)
    public static final float Ki = 5f; // Integral gain - Corrects accumulated past errors over time (helps eliminate steady drift)
    public static final float Kd = 800f; // Derivative gain - Predicts future error based on change rate (helps dampen overshoot)
    
    // Base and target values for line following:
    public static final float baseSpeed = 200; // default speed
    public static final float target = 0.2f; // 0.1 - Black, 1.0 - White, 0.2 - Closer to Black (edge between black line and white surface)
    public static final float outerThreshold = 0.05f; // Too far outside line (0.2 - 0.15) - on Black
    public static final float innerThreshold = 0.35f; // Too far inside line (0.2 + 0.15) - on  White
    
    // Control delays:
    public static final int controlLoopDelay = 10; // It controls the normal "heartbeat" of the robot's brain.
    public static final int recoveryDelay = 100;   // Gives the robot time to finish its movement
    public static final int COLOR_SAMPLE_INTERVAL = 15;  // ms delay for checking color sensor

    // Ultrasonic sensor ranges:
    public static final float DETECTION_RANGE = 0.40f;   // ultrasonic sensor detects an object within 40 cm
    public static final float BUFFER_RANGE = 0.30f;      // ultrasonic sensor keep buffer range 30 cm
    public static final float CRITICAL_RANGE = 0.15f;    // ultrasonic sensor react to object closer than 15 cm

    // Robot physical dimensions:
    public static final float ROBOT_WIDTH = 0.14f;       // robot width 14 cm
    public static final float ROBOT_LENGTH = 0.27f;      // robot length 27 cm

    // Avoidance speeds and times:
    public static final int forwardSpeed = 350; // Move forward speed to past the obstacle
    public static final int AVOIDANCE_SLOW_SPEED = 180; // Obstacle detected, slow down
    public static final int AVOIDANCE_TURN_SPEED_LEFT = 100;
    public static final int AVOIDANCE_TURN_SPEED_RIGHT = 200;
    public static final int AVOIDANCE_FORWARD_MS = 1000; // Tuned this lower (~1000ms or less)
}