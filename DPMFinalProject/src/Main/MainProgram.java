package Main;

import java.io.File;

import Launching.Launcher;
import Odometry.LCDInfo;
import Localization.USLocalizer;
import Navigation.Navigator;
import Odometry.Odometer;
import Odometry.OdometryCorrection;
import Wifi.WiFiData;
import lejos.hardware.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.utility.Delay;

/**
 * The Main Thread Program of the Robot
 * 
 * @author Noah Levine
 * @version 4.0
 * @since 2017-03-27  	
 */
public class MainProgram {

	// Static Resources:
	// Left motor connected to output A
	// Right motor connected to output D
	// Ultrasonic sensor port connected to input S1
	// Color sensor port connected to input S2
	
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor topMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	public static final EV3LargeRegulatedMotor topMotor2 = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	
	
	private static final Port LusPort = LocalEV3.get().getPort("S1");
	private static final Port MusPort = LocalEV3.get().getPort("S2");
	private static final EV3ColorSensor leftLightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
	private static final EV3ColorSensor rightLightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
	//private static final EV3UltrasonicSensor leftUsSensor = new EV3UltrasonicSensor(LusPort);
	private static final EV3UltrasonicSensor middleUsSensor = new EV3UltrasonicSensor(MusPort);

	

	private static final int [] targetPosition = {5,6};
	private static final double TILE_LENGTH = 30.48;
	
	// Constants
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 14.65;
	
	public static boolean demo = true;

	
	/**
	 * Main Thread Method. Manages the sequence of events
	 * 
	 * @param args Unused
	 * 
	 *    	
	 */
	public static void main(String[] args) {
		
		
	/*
		WiFiData data = new WiFiData();
		if (data.offense || data.defense) {
			Sound.beep();
		}
	*/
		

		
		
		
		
		Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
		LCDInfo lcd = new LCDInfo(odo);
		Navigator nav = new Navigator(odo, middleUsSensor);
		
		int corner = 0;
		OdometryCorrection odoCorrect = new OdometryCorrection(odo, leftLightSensor, rightLightSensor, nav, corner);

	
		 // perform the ultrasonic localization
		  USLocalizer usl = new USLocalizer(odo, middleUsSensor, nav); 
		  usl.doLocalization();
		
	  
		  //start odometry correction
		
		odoCorrect.start();
		topMotor.stop(); topMotor2.stop();
		
		nav.delay();
		nav.travelTo(40, 40);
		nav.travelTo(61, 40);
		nav.travelTo(61, 31+7);
		nav.turnTo(262, true);
		
		topMotor.setAcceleration(100); 
		topMotor2.setAcceleration(100);
		
		topMotor.setSpeed(100);
		topMotor2.setSpeed(100);
		
		topMotor.rotate(155, true); topMotor2.rotate(155, false);
		

		nav.reverseToDispenser();

		nav.setSpeeds(0, 0);
		
		Sound.beep();
		
		try {
			Thread.sleep(4000);
		} catch (Exception e) {
			
		}
	
	
		  
		  
		 /* 
		  
		  if (demo) {
			  
			  double xPosition = targetPosition[0]*TILE_LENGTH;
			  double yPosition = targetPosition[1]*TILE_LENGTH - lineDist*TILE_LENGTH - 0.5*TILE_LENGTH;
			
			  nav.travelTo(xPosition - 10, yPosition);
			  
			  odoCorrect.interrupt();
			  
			  nav.delay();
			  nav.travelTo(xPosition, yPosition);
			  nav.turnTo(90,true);
			  
			
		
			  Launcher launch = new Launcher(topMotor, topMotor2, lineDist);
			  
			  launch.launchBall();
		  }
		  */
	}
	
	/**
	 * This method makes the robot travel to the dispenser
	 * 
	 * @param bx The X coordinate of the dispenser (in Tile Units)
	 * @param by The Y coordinate of the dispenser (in Tile Units)
	 * 			
	 *     	
	 */
	private void goToDispenser(int bx, int by) {
		
	}
		


	

}