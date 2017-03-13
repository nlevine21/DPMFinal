package Localization;



import Main.MainProgram;
import Navigation.Navigator;
import Odometry.Odometer;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class LightLocalizer {
	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;	
	public Navigator nav;
	
	private final static double SENSOR_DIST = 3.5;
		
	public LightLocalizer(Odometer odo, SampleProvider colorSensor, float[] colorData) {
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		nav = new Navigator(this.odo);  // Initialize navigation object
	}
	
	public void doLocalization() {
		double XDistance;
	
		// Set robot wheel speeds and move forward until a black line.
		nav.setSpeeds(200, 200);
		
		
			while (true) {
		colorSensor.fetchSample(colorData,0);		// Get color sample.
		if ( (colorData[0]*1000) < 200) {			// Check for black lines.
			Sound.beep();
			nav.setSpeeds(0, 0);					// Stop motors. 
			XDistance = odo.getX();					// Record the data travelled.
			nav.goForward(-XDistance);				// Go back to the initial position.
			break;
		}
	}
		turnLeft();									// Rotate 90 degrees counterclockwise.
		nav.setSpeeds(200, 200);					// Move forward along positive- Y
		
		
		
		while (true) {
			colorSensor.fetchSample(colorData,0);
			if ( (colorData[0]*1000) < 200) {		// Check for black lines. 
				Sound.beep();
				nav.goForward(SENSOR_DIST);			// Go a distance Sensor_Dist after the line.
				turnRight();						// Turn 90 degrees clockwise.
				nav.setSpeeds(0, 0);				// Stop the motors.
				XDistance += SENSOR_DIST;			
				nav.goForward(XDistance);			// Travel towards (0,0).
				break;
			}
		}
		


		
		odo.setPosition(new double[]{0, 0,0}, new boolean[] {true,true,true});
		
		
			}
	
	// turn 90 degrees clockwise
	private void turnRight() {
		MainProgram.leftMotor.setSpeed(USLocalizer.ROTATION_SPEED);
		MainProgram.rightMotor.setSpeed(USLocalizer.ROTATION_SPEED);

		MainProgram.leftMotor.rotate(convertAngle(MainProgram.WHEEL_RADIUS, MainProgram.TRACK, 90.0), true);
		MainProgram.rightMotor.rotate(-convertAngle(MainProgram.WHEEL_RADIUS, MainProgram.TRACK, 90.0), false);
	}
	
	// turn 90 degrees clockwise
	private void turnLeft() {
		MainProgram.leftMotor.setSpeed(USLocalizer.ROTATION_SPEED);
		MainProgram.rightMotor.setSpeed(USLocalizer.ROTATION_SPEED);

		MainProgram.leftMotor.rotate(-convertAngle(MainProgram.WHEEL_RADIUS, MainProgram.TRACK, 90.0), true);
		MainProgram.rightMotor.rotate(convertAngle(MainProgram.WHEEL_RADIUS, MainProgram.TRACK, 90.0), false);
	}
	
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	}