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

	

	private static final int [] target= {10,2};
	public static final double TILE_LENGTH = 30.48;
	
	// Constants
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 14.65;
	
	
	public static boolean demo = true;

	private static int dispenserMin = -1, dispenserMax = 11;
	
	/**
	 * Main Thread Method. Manages the sequence of events
	 * 
	 * @param args Unused
	 * 
	 *    	
	 */
	public static void main(String[] args) {
		
		

		WiFiData data = new WiFiData();
		if (data.offense || data.defense) {
			Sound.beep();
		}

		

		backLightSensor.getRGBMode().fetchSample(backColorsIniti, 0);

		
		Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
		LCDInfo lcd = new LCDInfo(odo);
		Navigator nav = new Navigator(odo, middleUsSensor);
		
		 // perform the ultrasonic localization
		 USLocalizer usl = new USLocalizer(odo, middleUsSensor, nav); 
		 usl.doLocalization();
		  
		  //start odometry correction
		OdometryCorrection odoCorrect = new OdometryCorrection(odo, leftLightSensor, rightLightSensor, nav, data.corner);
		odoCorrect.start();
		
		topMotor.stop(); topMotor2.stop();
	
	
		if (data.offense) {
			if (data.corner == 3) {
				nav.travelTo(10*TILE_LENGTH - 15, 15.24, true);
			}
			if (data.corner == 4) {
				nav.travelTo(15.24, 15.24, true);
			}
			
			if (data.orientation.equals("S")) {
				data.by = -1;
			}
			
			if (data.orientation.equals("W")) {
				data.bx = -1;
			}
			
			if (data.orientation.equals("E")) {
				data.bx = 11;
			}
			
			while (true) {
				goToDispenser(data.bx, data.by, dispenserMin, dispenserMax, nav);
				goToLaunchPoint(target[0], target[1], data.d1, nav);
				Launcher launch = new Launcher(topMotor, topMotor2, data.d1);
				launch.launchBall();
			}
			
		}
		else if (data.defense) {
			nav.travelTo(target[0]*TILE_LENGTH, (target[1]-1)*TILE_LENGTH - data.w2*TILE_LENGTH, false);
		}
		
		


		
	
	}
	
	
	private static void dispenserLocalize(Navigator nav) {
		
		while (!isLineOnBack()) {
			nav.setSpeeds(-30, 30);
		}
		
		nav.setSpeeds(0, 0);
		
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
		
		boolean xFirst; 
		
		if (bx == max) {
			
			xFirst = true;
			
			xCm -= TILE_LENGTH;
			nav.travelTo(xCm - 10, yCm - 15, xFirst);
			nav.travelTo(xCm - 10, yCm, xFirst);
			nav.travelTo(xCm - 19, yCm, xFirst);
			nav.turnTo(150, xFirst);
		}
		else if (bx == min) {
			
			xFirst = true;
			
			xCm += TILE_LENGTH;
			nav.travelTo(xCm + 10, yCm - 15, xFirst);
			nav.travelTo(xCm + 10, yCm, xFirst);
			nav.travelTo(xCm + 19, yCm, xFirst);
			nav.turnTo(330, xFirst);
		}
		else if (by == min) {
			
			xFirst = false;
			
			yCm += TILE_LENGTH;
			nav.travelTo(xCm - 15, yCm + 10, xFirst);
			nav.travelTo(xCm, yCm + 10, xFirst);
			nav.travelTo(xCm, yCm + 19, xFirst);
			nav.turnTo(60, xFirst);
		}
		
		nav.setSpeeds(0, 0);
		
		topMotor.setAcceleration(100); 
		topMotor2.setAcceleration(100);
		
		topMotor.setSpeed(100);
		topMotor2.setSpeed(100);
		
		topMotor.rotate(160, true); topMotor2.rotate(160, false);
		topMotor.flt(true); topMotor2.flt(true);
		nav.delay();
		nav.delay();
		
		topMotor.rotate(-25, true); topMotor2.rotate(-25, false);

		nav.setSpeeds(0,0);
		
		dispenserLocalize(nav);
		
		nav.setSpeeds(0, 0);
		
		nav.reverseToDispenser();
		nav.setSpeeds(0, 0);		
		topMotor.stop(); topMotor2.stop();
		Sound.beep();
		delay();
		
		
		
	
	}
	
	private static void goToLaunchPoint (int targetX, int targetY, int distance, Navigator nav) {
		
		nav.turnOffSensor = true;
		
		double targetXCm = targetX * TILE_LENGTH;
		double targetYCm = targetY * TILE_LENGTH;
		double distanceCm = distance * TILE_LENGTH;
	
		nav.travelTo((targetXCm - distanceCm) + 15, targetYCm + 15, false);
		nav.travelTo((targetXCm - distanceCm) + 15, targetYCm - 15, false);
		nav.travelTo((targetXCm - distanceCm) - 8 , targetYCm, true);
		nav.turnTo(330, true);
		dispenserLocalize(nav);
		
	}
	
	public static void delay() {
		Delay.msDelay(5000);
	}
	
		


	

}