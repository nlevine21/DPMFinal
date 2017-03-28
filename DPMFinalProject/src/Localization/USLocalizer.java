package Localization;

import Navigation.Navigator;
import Navigation.Navigator;
import Odometry.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
/**
 * The Localization of the Robot
 * 
 * @author Noah Levine, Steven Cangul
 * @version 1.0
 * @since 2017-02-07  	
 */
public class USLocalizer {
	public static int ROTATION_SPEED = 180;
	
	private static final int WALL_DIST	= 30;  // WALL_DIST is the distance needed to detect a wall
	private Odometer odo;
	private SensorModes usSensor;
	private SampleProvider usValue;
	private float[] usData;
	public Navigator nav;
	
	/**
	 * Constructor for the Ultrasonic Localization tool
	 * 
	 * @param odo The odometer of the Robot
	 * @param usSensor The front US sensor of the Robot
	 * @param nav The navigator of the robot
	 * 
	 * @return USLocalizer The Localization object	
	 */
	public USLocalizer(Odometer odo,  EV3UltrasonicSensor usSensor, Navigator nav) {
		this.odo = odo;
		this.usSensor = usSensor;
		this.nav = nav;
		this.usValue = usSensor.getMode("Distance");			
		this.usData = new float[usValue.sampleSize()];		
	}
	
	
	/**
	 * The method which performs the localization
	 * 
	 * 
	 * 
	 */
	public void doLocalization() {
		double angleA, angleB; 
		double actualAng = 0; 
		
			//Have the robot turn right until it no longer sees a wall
			//Thus, the falling edge technique can always be implemented 
			while(seesWall()){
				rightTurn();
			}
			
			
			delay();
			
			//Continue turning the robot right until it sees a wall
			while (!seesWall()){	
				rightTurn();
			}
			
			//Save the angle at which the robot sees the first wall
			angleA = odo.getAng(); 
			nav.setSpeeds(0, 0);
			delay();
		
			//Turn the robot left until it no longer sees the wall
			while(seesWall()){
				leftTurn();
			}
			
			delay(); 
			
			//Turn the robot left until it sees the wall
			while (!seesWall()){
				leftTurn();
			}
			
			//Save the angle at which the robot sees the second wall
			angleB = odo.getAng(); 
			nav.setSpeeds(0, 0); 
			delay();
			
			//Calculate the actual Angle using the calcHeading method 
			actualAng = calcHeading(angleA,angleB) + odo.getAng(); 
			
			//Set the odometer's position to (0,0,acutalAngle)
			odo.setPosition(new double [] {0.0, 0.0, actualAng}, new boolean [] {true, true, true});
			
			//Turn the robot to face angle zero
			nav.turnTo(0, true); 
			
		
	}
	
	// filter the data 
	/**
	 * Method which filters the data from the US sensor
	 * 
	 *  
	 * 	
	 */
	private float getFilteredData() { 
		usSensor.fetchSample(usData, 0);
		float distance = usData[0] * 100;
		
		if (distance >= 50){
			distance = 50;
		}
		return distance;
	}
	
	// rotate left
	/**
	 * Method which causes the robot to rotate to the left until interrupted
	 * 
	 * 
	 * 	
	 */
	private  void leftTurn(){ 
		nav.setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);
	}

	// rotate right
	/**
	 * Method which causes the robot to rotate to the right until interrupted
	 * 
	 * 
	 * 	
	 */
	private  void rightTurn(){ 
		nav.setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);
	}
	
	//Method returning whether the robot sees the wall
	/**
	 * Method which returns whether the robot sees the wall
	 * 
	 * 
	 * 	
	 */
	private boolean seesWall() {
		return getFilteredData() <= WALL_DIST;
	}
	
	//Delay method to prevent bad data from sensor
	/**
	 * Method which delays the thread for 1 second for  synchronization
	 * 
	 * 
	 * 	
	 */
	private void delay() {
		Delay.msDelay(1000);
	}
	
	// calculation of heading as shown in the tutorial
	/**
	 * Method which calculates the initial heading of the robot
	 * 
	 * @param angleA The angle at which the robot saw the wall for the first time
	 * @param angleB The angle at which the robot saw the wall for the second time
	 * @return double
	 */
	private static double calcHeading(double angleA, double angleB) { 
		
		double heading;
		
		if (angleA < angleB){
			heading = 45 - (angleA + angleB) /2 ;
		}else {
			heading = 225 - (angleA + angleB) /2 ;	
		}
		return heading;
	}
}