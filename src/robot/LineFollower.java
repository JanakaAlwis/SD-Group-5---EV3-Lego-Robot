package robot;

//import lejos.hardware.Button;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class LineFollower {
    public static void main(String[] args) {
        // Initialize sensors
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
        EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S2);
        
        // Create tasks
        LineSensorTask lineSensorTask = new LineSensorTask(colorSensor);
        ObstacleAvoidTask obstacleAvoidTask = new ObstacleAvoidTask(usSensor,colorSensor);
        
        // Create and start threads
        Thread lineThread = new Thread(lineSensorTask);
        Thread obstacleThread = new Thread(obstacleAvoidTask);
        
        lineThread.start();
        obstacleThread.start();
        
        // Wait for threads to finish
        try {
            lineThread.join();
            obstacleThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        } finally {
            // Clean up
            colorSensor.close();
            usSensor.close();
            RobotController.stop();
        }
    }
}