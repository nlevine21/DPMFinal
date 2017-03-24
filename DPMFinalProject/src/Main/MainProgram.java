package Main;

import Launching.Launcher;
import Odometry.LCDInfo;
import Localization.LightLocalizer;
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
	private static final Port RusPort = LocalEV3.get().getPort("S3");
	private static final Port colorPort = LocalEV3.get().getPort("S4");
	private static final EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
	private static final EV3UltrasonicSensor leftUsSensor = new EV3UltrasonicSensor(LusPort);
	private static final EV3UltrasonicSensor middleUsSensor = new EV3UltrasonicSensor(MusPort);
	private static final EV3UltrasonicSensor rightUsSensor = new EV3UltrasonicSensor(RusPort);

	private static int [] targetPosition = {5,6};
	
	// Constants
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 15;
	
	public static boolean demo = true;

	public static void main(String[] args) {
		
		WiFiData data = new WiFiData();
		if (data.offense || data.defense) {
			Sound.beep();
		}
		
		

		
		Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
		LCDInfo lcd = new LCDInfo(odo);
		OdometryCorrection odoCorrect = new OdometryCorrection(odo, lightSensor);

		Navigator nav = new Navigator(odo, leftUsSensor, middleUsSensor, rightUsSensor);
		
		 // perform the ultrasonic localization
		
		  USLocalizer usl = new USLocalizer(odo, middleUsSensor, nav); 
		  usl.doLocalization();
		  Sound.beep();
		  
		  odoCorrect.start();

		  if (data.d1 == 3) {
			  data.d1 = 4;
		  }
		  
		  if (demo) {
			  nav.travelTo(targetPosition[0]*30.48 - 10, targetPosition[1]*30.48 - data.d1*30.48 - 0.5*30.48);
			  odoCorrect.interrupt();
			  
			  nav.travelTo(152.4, 45.72);
			  nav.turnTo(90,true);
			  
			 			  
			  Launcher launch = new Launcher(topMotor, topMotor2, 4);
			  
			  launch.launchBall();
		  }
	}
		


	



}