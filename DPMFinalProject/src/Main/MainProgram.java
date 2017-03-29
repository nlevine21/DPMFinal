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
	

	private static final Port MusPort = LocalEV3.get().getPort("S2");
	private static final EV3ColorSensor leftLightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
	private static final EV3ColorSensor rightLightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
	private static final EV3ColorSensor backLightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	private static final EV3UltrasonicSensor middleUsSensor = new EV3UltrasonicSensor(MusPort);
	
	private static float[] backColors = {0,0,0}, backColorsIniti = {0,0,0};

	

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
		

		
		
		backLightSensor.getRGBMode().fetchSample(backColorsIniti, 0);

		
		Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
		LCDInfo lcd = new LCDInfo(odo);
		Navigator nav = new Navigator(odo, middleUsSensor);
		
		int corner = 0;
		OdometryCorrection odoCorrect = new OdometryCorrection(odo, leftLightSensor, rightLightSensor, nav, corner);

	
		 // perform the ultrasonic localization
		  USLocalizer usl = new USLocalizer(odo, middleUsSensor, nav); 
		  usl.doLocalization();
		  
		  double[] position = {-15,-15,0}; boolean[] update = {true,true,true};
		  odo.setPosition(position, update);
		  
		  //start odometry correction
		
		odoCorrect.start();
		topMotor.stop(); topMotor2.stop();
		
		
		

		int bx = 11; int by = 1;
		goToDispenser(bx, by, -1, 11, nav);
		



		
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
	
	
	private static void dispenserLocalize(Navigator nav) {
		
		while (!isLineOnBack()) {
			nav.setSpeeds(-30, 30);
		}
		
	}
	
	private static boolean isLineOnBack() {
		backLightSensor.getRGBMode().fetchSample(backColors, 0);
		if (backColors[0] < (backColorsIniti[0] - 0.07) && backColors[1] < backColorsIniti[1] && backColors[2] < (backColorsIniti[2] - 0.004)) {
			return true;
		} else {
			return false;
		}
	}
	
	/**
	 * This method makes the robot travel to the dispenser
	 * 
	 * @param bx The X coordinate of the dispenser (in Tile Units)
	 * @param by The Y coordinate of the dispenser (in Tile Units)
	 * 			
	 *     	
	 */
	private static void goToDispenser(int bx, int by, int min, int max, Navigator nav) {
		
		double xCm = bx*TILE_LENGTH;
		double yCm = by*TILE_LENGTH;
		
		if (bx == max) {
			xCm -= TILE_LENGTH;
			nav.travelTo(xCm - 15, yCm - 15);
			nav.travelTo(xCm - 15, yCm);
			nav.travelTo(xCm - 22, yCm);
			nav.turnTo(150, true);
		}
		else if (bx == min) {
			xCm += TILE_LENGTH;
			nav.travelTo(xCm + 15, yCm - 15);
			nav.travelTo(xCm + 15, yCm);
			nav.travelTo(xCm + 22, yCm);
			nav.turnTo(330, true);
		}
		else if (by == max) {
			yCm -= TILE_LENGTH;
			nav.travelTo(xCm - 15, yCm - 15);
			nav.travelTo(xCm, yCm - 15);
			nav.travelTo(xCm, yCm - 22);
			nav.turnTo(240, true);
		}
		else if (by == min) {
			yCm += TILE_LENGTH;
			nav.travelTo(xCm - 15, yCm + 15);
			nav.travelTo(xCm, yCm + 15);
			nav.travelTo(xCm, yCm + 22);
			nav.turnTo(60, true);
		}
		
		
		
		topMotor.setAcceleration(100); 
		topMotor2.setAcceleration(100);
		
		topMotor.setSpeed(100);
		topMotor2.setSpeed(100);
		
		topMotor.rotate(155, true); topMotor2.rotate(155, false);

		
		dispenserLocalize(nav);
		
		nav.setSpeeds(0, 0);
		nav.reverseToDispenser();

	}
		


	

}