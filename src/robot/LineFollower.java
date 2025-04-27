package robot;

//import lejos.hardware.Button;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class LineFollower {
    public static void main(String[] args) {
        // Initialize sensors
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1); // Creates a color sensor object on port S1
        EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S2); // Creates an ultrasonic sensor object on port S2
        
        // Initialize tasks
        // LineSensorTask - Handles PID line following and recovery logic using the color sensor
        // ObstacleAvoidTask - Monitors for obstacles using the ultrasonic sensor and performs avoidance
        LineSensorTask lineSensorTask = new LineSensorTask(colorSensor);
        ObstacleAvoidTask obstacleAvoidTask = new ObstacleAvoidTask(usSensor,colorSensor);
        
        // Create and start threads (running both tasks in parallel using two separate threads)
        Thread lineThread = new Thread(lineSensorTask);
        Thread obstacleThread = new Thread(obstacleAvoidTask);
        
        lineThread.start();
        obstacleThread.start();
        
        // Wait for threads to finish
        try {
            lineThread.join(); // ensures the main thread waits for the two threads to finish
            obstacleThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace(); // to see exactly where the error happened
        } finally {
            // Closes the sensors properly to free hardware resources
            colorSensor.close();
            usSensor.close();
            RobotController.stop();
        }
    }
}