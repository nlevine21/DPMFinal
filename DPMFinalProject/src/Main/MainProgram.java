package Main;


import Localization.LCDInfo;
import Localization.LightLocalizer;
import Localization.USLocalizer;
import Navigation.Navigator;
import lejos.hardware.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;


public class MainProgram {

	// Static Resources:
	// Left motor connected to output A
	// Right motor connected to output D
	// Ultrasonic sensor port connected to input S1
	// Color sensor port connected to input S2
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port LusPort = LocalEV3.get().getPort("S1");
	private static final Port MusPort = LocalEV3.get().getPort("S2");
	private static final Port RusPort = LocalEV3.get().getPort("S3");
	private static final Port colorPort = LocalEV3.get().getPort("S4");		

	
	// Constants
		public static final double WHEEL_RADIUS = 2.1;
		public static final double TRACK = 14.6;
	
	
	public static void main(String[] args) {
		 
		EV3UltrasonicSensor leftUsSensor = new EV3UltrasonicSensor(LusPort);
		EV3UltrasonicSensor middleUsSensor = new EV3UltrasonicSensor(MusPort);
		EV3UltrasonicSensor rightUsSensor = new EV3UltrasonicSensor(RusPort);
				
		/*
		//Setup ultrasonic sensor
		@SuppressWarnings("resource")							    	// Because we don't bother to close this resource
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		SampleProvider usValue = usSensor.getMode("Distance");			// colorValue provides samples from this instance
		float[] usData = new float[usValue.sampleSize()];				// colorData is the buffer in which data are returned
		*/
		
		
		//Setup color sensor
		SensorModes colorSensor = new EV3ColorSensor(colorPort);
		SampleProvider colorValue = colorSensor.getMode("Red");			// colorValue provides samples from this instance
		float[] colorData = new float[colorValue.sampleSize()];			// colorData is the buffer in which data are returned
		
		
		// setup the odometer and display
		Odometry.Odometer odo = new Odometry.Odometer(leftMotor, rightMotor, 30, true);
		LCDInfo lcd = new LCDInfo(odo);
		
		Navigator nav = new Navigator(odo, leftUsSensor, middleUsSensor, rightUsSensor);
		
		/*
		// perform the ultrasonic localization
		USLocalizer usl = new USLocalizer(odo, usValue, usData,nav);
		usl.doLocalization();
		
		// perform the light sensor localization
		LightLocalizer lsl = new LightLocalizer(odo, colorValue, colorData,nav);
		lsl.doLocalization(); */
		
		nav.travelTo(30,30);
		
		//If a button is pressed, terminate the program
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}

}