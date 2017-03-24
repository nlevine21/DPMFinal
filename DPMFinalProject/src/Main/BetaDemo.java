package Main;

import Launching.Launcher;
import Odometry.LCDInfo;
import Localization.USLocalizer;
import Localization.USLocalizer2;
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

public class BetaDemo {

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
	private static final Port RusPort = LocalEV3.get().getPort("S3");
	private static final EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
	private static final EV3UltrasonicSensor leftUsSensor = new EV3UltrasonicSensor(LusPort);
	private static final EV3UltrasonicSensor middleUsSensor = new EV3UltrasonicSensor(MusPort);
	private static final EV3UltrasonicSensor rightUsSensor = new EV3UltrasonicSensor(RusPort);
	

	private static final int [] targetPosition = {5,6};
	private static final double TILE_LENGTH = 30.48;
	
	// Constants
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 14.75;
	
	public static boolean demo = true;

	public static void main(String[] args) {
		
	
		WiFiData data = new WiFiData();
		if (data.offense || data.defense) {
			Sound.beep();
		}
		
		
		int lineDist = data.d1;
		
		
		
		
		Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
		LCDInfo lcd = new LCDInfo(odo);
		OdometryCorrection odoCorrect = new OdometryCorrection(odo, lightSensor);

		Navigator nav = new Navigator(odo, leftUsSensor, middleUsSensor, rightUsSensor);
		
		 // perform the ultrasonic localization
		
		  USLocalizer usl = new USLocalizer(odo, middleUsSensor, nav); 
		  usl.doLocalization();
		  Sound.beep();
		
		  
		  odoCorrect.start();
		  USLocalizer2 usl2 = new USLocalizer2(odo, nav, rightUsSensor);
		  usl2.run();

		  if (lineDist == 3) {
			  lineDist = 4;
		  }
		  
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
	}
		


	

}