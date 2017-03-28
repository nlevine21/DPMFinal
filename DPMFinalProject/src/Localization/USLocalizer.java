package Localization;

import Navigation.Navigator;
import Navigation.Navigator;
import Odometry.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class USLocalizer {
	public static float ROTATION_SPEED = 180;
	
	private static final int WALL_DIST	= 30;  // WALL_DIST is the distance needed to detect a wall
	private Odometer odo;
	private SensorModes usSensor;
	private SampleProvider usValue;
	private float[] usData;
	public Navigator nav;
	
	public USLocalizer(Odometer odo,  EV3UltrasonicSensor usSensor, Navigator nav) {
		this.odo = odo;
		this.usSensor = usSensor;
		this.nav = nav;
		this.usValue = usSensor.getMode("Distance");			
		this.usData = new float[usValue.sampleSize()];		
	}
	
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
	private float getFilteredData() { 
		usSensor.fetchSample(usData, 0);
		float distance = usData[0] * 100;
		
		if (distance >= 50){
			distance = 50;
		}
		return distance;
	}
	
	// rotate left
	private  void leftTurn(){ 
		nav.setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);
	}

	// rotate right
	private  void rightTurn(){ 
		nav.setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);
	}
	
	//Method returning whether the robot sees the wall
	private boolean seesWall() {
		return getFilteredData() <= WALL_DIST;
	}
	
	//Delay method to prevent bad data from sensor
	private void delay() {
		Delay.msDelay(1000);
	}
	
	// calculation of heading as shown in the tutorial
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