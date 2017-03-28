package Navigation;

import Main.MainProgram;
import Odometry.Odometer;
import Odometry.Odometer.Direction;
import lejos.hardware.Sound;
/*


 * File: Navigation.java
 * Written by: Sean Lawlor
 * ECSE 211 - Design Principles and Methods, Head TA
 * Fall 2011
 * Ported to EV3 by: Francois Ouellet Delorme
 * Fall 2015
 * 
 * Movement control class (turnTo, travelTo, flt, localize)
 */
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.utility.Delay;

/**
 * The Navigator of the Robot
 * 
 * @author Noah Levine, Rowan Kennedy, Ryan Servera 
 * @version 4.0
 * @since 2017-03-27  	
 */
public class Navigator {
	public final static int FAST = 150;
	static final int SLOW = 100, SLOW_LINE = 50;
	static final int ACCELERATION = 4000;
	final static int FRONT_DETECTABLE_DISTANCE = 20, SIDE_DETECTABLE_DISTANCE = 40;
	final static double DEG_ERR = 3.0, CM_ERR = .5;
	private Odometer odometer;
//	private Direction currentDirection1;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private EV3UltrasonicSensor middleUsSensor;
	
	private boolean headingCorrect, setToSlow;
	
	private double leftDistance, rightDistance;
	
	private final double LIGHT_SENSOR_DIST = 9.9;
	
	public int lineCount;
	
//	private static final double xshoot = 152.4, yshoot = 45.72; // point to
																// shoot from
	/**
	 * Constructor for the Navigation tool
	 * 
	 * @param odo The odometer of the Robot
	 * @param leftUsSensor The left US sensor of the Robot
	 * @param middleUsSensor The middle US sensor of the robot
	 * 
	 * @return Navigator The Navigation object	
	 */
	public Navigator(Odometer odo, EV3UltrasonicSensor middleUsSensor) {
		this.lineCount = 0;
		
		this.odometer = odo;
		this.middleUsSensor = middleUsSensor;

		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];

		// set acceleration
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
		
		this.headingCorrect = false;
	}



	/*
	 * Called if an object appears in front of it and navigates around it. Uses
	 * 90 degree turns needs to take coordinates it's traveling too
	 */
	
	/**
	 * Avoiding method which is after an obstacle is detected by
	 * the middle US sensor
	 * 
	 * 
	 * 
	 */
	public void avoid() {

		
		delay();
		turnRight();
		
		
		while (true) {
			boolean blocked = false;
			if (isObstacle(middleUsSensor, SIDE_DETECTABLE_DISTANCE)) {
				blocked = true;
			}
			
			if (!blocked) {
				delay();
				travelForward();
				this.headingCorrect = false;
				return;
			}
			else {
				turnLeft();
				turnLeft();
			}
		}
		

	}


	/*
	 * robot turns 90 degrees to the left
	 */
	/**
	 * Turns the robot 90 degrees to the left
	 * 
	 * 
	 * 
	 */
	private void turnLeft() {
		// turning more than 90 deg, needs to be fixed
		float deg = convertAngle(MainProgram.WHEEL_RADIUS, MainProgram.TRACK, 90);
		// System.out.println("left");
		// System.out.println(deg);
		leftMotor.setSpeed(SLOW);
		rightMotor.setSpeed(SLOW);
		leftMotor.rotate((int) -deg, true);
		rightMotor.rotate((int) deg, false);
		leftMotor.stop();
		rightMotor.stop();
	}

	/*
	 * robot turns 90 degrees to the right
	 */
	/**
	 * Turns the robot 90 degrees to the right
	 * 
	 * 
	 * 
	 */
	private void turnRight() {
		// turning more than 90 deg, needs to be fixed
		float deg = convertAngle(MainProgram.WHEEL_RADIUS, MainProgram.TRACK, 90);
		leftMotor.setSpeed(SLOW);
		rightMotor.setSpeed(SLOW);
		// System.out.println("right");
		// System.out.println(deg);
		//leftMotor.forward();   THESE 2 SHOULDNT BE HERE. THIS MIGHT BE WHY IT WAS SUDDENLY SPEEDING UP
		//rightMotor.backward();
		leftMotor.rotate((int) deg, true);
		rightMotor.rotate((int) -deg, false);
		leftMotor.stop();
		rightMotor.stop();
	}



	/*
	 * robot goes forward one square (30cm)
	 */
	/**
	 * Tells the Robot to travel 33cm forward
	 * 
	 * 
	 * 
	 */
	private void travelForward() {
		this.headingCorrect = false;
		
		float deg = convertDistance(MainProgram.WHEEL_RADIUS, 25);
		leftMotor.setSpeed(FAST);
		rightMotor.setSpeed(FAST);
		// System.out.println("forward");
		// System.out.println(deg);
		//leftMotor.forward();		SAME THING AS BEFORE. SHOULDNT BE HERE
		//rightMotor.forward();
		
		
		leftMotor.rotate((int) deg, true);
		rightMotor.rotate((int) deg, false);
	}
	
	/**
	 * Reverses the Robot 13cm. (Called to reverse to the ball
	 * dispenser)
	 * 
	 * 
	 * 
	 */
	public void reverseToDispenser() {
		float deg = convertDistance(MainProgram.WHEEL_RADIUS, 15);
		leftMotor.setSpeed(FAST);
		rightMotor.setSpeed(FAST);
		// System.out.println("forward");
		// System.out.println(deg);
		//leftMotor.forward();		SAME THING AS BEFORE. SHOULDNT BE HERE
		//rightMotor.forward();
		
		
		leftMotor.rotate((int) -deg, true);
		rightMotor.rotate((int) -deg, false);
	}

	/*
	 * checks if US sensor can detect an object in adjacent tile
	 */
	/**
	 *	Checks whether a US Sensor sees an obstacle
	 * 
	 * @param usSensor The sensor to be checked
	 * @param distanceWall The minimum distance at which an object would be detected 
	 * 
	 * @return boolean Whether the sensor sees an object
	 * 
	 */
	public boolean isObstacle(EV3UltrasonicSensor usSensor, int distanceWall) {
		boolean isObstacle = false;
		float[] distance = { 0 };
		usSensor.fetchSample(distance, 0);
		distance[0] = distance[0] * 100;
		// System.out.print(" d: ");
		// System.out.println(distance[0]);
		if (distance[0] < distanceWall) {
			isObstacle = true;
		}
		return isObstacle;
	}

	/*
	 * Functions to set the motor speeds jointly
	 */
	/**
	 * Sets the speeds of the driving motors
	 * 
	 * @param lSpd The speed to set the left motor
	 * @param rSpd The speed to set the right motor
	 * 
	 * 
	 */
	public void setSpeeds(int lSpd, int rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}

	/*
	 * Float the two motors jointly
	 */
	
	/**
	 * Method which floats the driving motors
	 * 
	 * 
	 * 
	 */
	public void setFloat() {
		this.leftMotor.stop();
		this.rightMotor.stop();
		this.leftMotor.flt();		//THESE WERE BOTH SET TO TRUE BEFOre
		this.rightMotor.flt();
	}

	/*
	 * TravelTo function which takes as arguments the x and y position in cm
	 * Will travel to designated position, while constantly updating it's
	 * heading
	 */
	/**
	 * Method which allows the robot to travel to an (x,y) 
	 * position through the minimum angle
	 * 
	 * @param x The requested x coordinate (in cm)
	 * @param y The requested y coordinate (in cm)
	 * 
	 */
	public void travelToMinAngle(double x, double y) {
		double minAng;
		while (Math.abs(x - odometer.getX()) > CM_ERR || Math.abs(y - odometer.getY()) > CM_ERR) {
			minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);
			if (minAng < 0)
				minAng += 360.0;
			this.turnTo(minAng, false);
			this.setSpeeds(SLOW, SLOW);
		}
		this.setSpeeds(0, 0);		// Why do we have this here?
	}

	/*
	 * travels to tile given in x and y coordinates
	 */
	/**
	 * Method which allows the robot to travel to an (x,y) 
	 * position by first travelling in x and then in y
	 * 
	 * @param x The requested x coordinate (in cm)
	 * @param y The requested y coordinate (in cm)
	 * 
	 */
	public void travelTo(double x, double y) {

		double currentX = odometer.getX();
		double currentY = odometer.getY();

		if (!(Math.abs(x - currentX) < CM_ERR)) {

			if ((x - currentX) >= 0) {
				

				
				turnTo(0, true);
				//this.lineCount = 0;
				
				while ((x - odometer.getX()) > CM_ERR) {
					
					if (headingCorrect) {
						this.setSpeeds(0, 0);
						headingCorrection(leftDistance, rightDistance);
						headingCorrect = false;
					}
					
					if (setToSlow) {
						this.setSpeeds(SLOW_LINE, SLOW_LINE);
					}else {
						this.setSpeeds(FAST, FAST);
					}

					
					
					if (isObstacle(middleUsSensor, FRONT_DETECTABLE_DISTANCE)) {
						this.setSpeeds(0, 0);
						delay();
						avoid();
						delay();
						travelTo(x, y);
						return;
					}

				}
						
				this.setSpeeds(0, 0);
				



			} else {
				
				
				turnTo(180, true);
			//	this.lineCount = 0;
				
				while ((odometer.getX() - x) > CM_ERR) {
					
					if (headingCorrect) {
						this.setSpeeds(0, 0);
						headingCorrection(leftDistance, rightDistance);
						headingCorrect = false;
					}
					
					

					if (setToSlow) {
						this.setSpeeds(SLOW_LINE, SLOW_LINE);
					}else {
						this.setSpeeds(FAST, FAST);
					}

					
					if (isObstacle(middleUsSensor, FRONT_DETECTABLE_DISTANCE)) {
						this.setSpeeds(0, 0);
						delay();
						avoid();
						delay();
						travelTo(x, y);
						return;
					}
				}
				this.setSpeeds(0, 0);

			}
		}

		if (!(Math.abs(y - currentY) < CM_ERR)) {

			if ((y - currentY) >= 0) {
				
				
				turnTo(90, true);
			//	this.lineCount = 0;
				
				while (((y - odometer.getY()) > CM_ERR)) {
					
					if (headingCorrect) {
						this.setSpeeds(0, 0);
						headingCorrection(leftDistance, rightDistance);
						headingCorrect = false;
					}
					
				
					if (setToSlow) {
						this.setSpeeds(SLOW_LINE, SLOW_LINE);
					} else {
						this.setSpeeds(FAST, FAST);
					}

					
					if (isObstacle(middleUsSensor, FRONT_DETECTABLE_DISTANCE)) {
						this.setSpeeds(0, 0);
						delay();
						avoid();
						delay();
						travelTo(odometer.getX(), y);
						travelTo(x, y);
						return;
					}
				}
				this.setSpeeds(0, 0);

			} else {
				
			
				
				turnTo(270, true);
			//	this.lineCount = 0;
				
				while ((odometer.getY()-y) > CM_ERR) {

					if (headingCorrect) {
						this.setSpeeds(0, 0);
						headingCorrection(leftDistance, rightDistance);
						headingCorrect = false;
					}

					if (setToSlow) {
						this.setSpeeds(SLOW_LINE, SLOW_LINE);
					}else {
						this.setSpeeds(FAST, FAST);
					}

					
					if (isObstacle(middleUsSensor, FRONT_DETECTABLE_DISTANCE)) {
						this.setSpeeds(0, 0);
						delay();
						avoid();
						delay();
						travelTo(odometer.getX(), y);
						travelTo(x, y);
						return;
					}
				}
				this.setSpeeds(0, 0);

			}
		}
	}

	/*
	 * TurnTo function which takes an angle and boolean as arguments The boolean
	 * controls whether or not to stop the motors when the turn is completed
	 */
	
	/**
	 * Method which turns the Robot to face a certain heading
	 * 
	 * @param angle The requested angle (in degrees)
	 * @param stop Boolean variable determining whether or not to stop the motors after turning
	 * 
	 */
	public void turnTo(double angle, boolean stop) {

		double error = angle - this.odometer.getAng();

		while (Math.abs(error) > DEG_ERR) {

			error = angle - this.odometer.getAng();

			if (error < -180.0) {
				this.setSpeeds(-SLOW, SLOW);
			} else if (error < 0.0) {
				this.setSpeeds(SLOW, -SLOW);
			} else if (error > 180.0) {
				this.setSpeeds(SLOW, -SLOW);
			} else {
				this.setSpeeds(-SLOW, SLOW);
			}
		}

		if (stop) {
			this.setSpeeds(0, 0);
		}
	}



	// converts amount of rotation based on the wheel radius
	
	/**
	 * Method which converts a requested distance to the angle which 
	 * the driving motors must rotate to drive that distance
	 * 
	 * @param radius The wheel radius (in cm)
	 * @param distance The requested distance (in cm)
	 * @return int The angle which the wheels must rotate to travel the distance (in degrees)
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	// calculates the rotations need to get to a certain angle
	/**
	 * Method which converts a requested angle to the angle which 
	 * the driving motors must rotate to drive that angle
	 * 
	 * @param radius The wheel radius (in cm)
	 * @param width The distance between the two wheels (in cm)
	 * @param angle The requested distance (in degrees)
	 * @return int The angle which the wheels must rotate to travel the distance (in degrees)
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	/**
	 * Delay method for synchronization
	 * 
	 * 
	 * 
	 */
	public void delay() {
		Delay.msDelay(2000);
	}
	
	/**
	 * Setter method called by Odometry Correction to inform the navigator
	 * that it must perform a heading correction 
	 * 
	 * @param leftDistance The distance sensed by the left color sensor (in cm)
	 * @param rightDistance The distance sensed by the right color sensor (in cm)
	 * @param headingCorrect Boolean variable representing the need for a heading correction
	 * 
	 * 
	 */
	public void setHeadingCorrection(double leftDistance, double rightDistance, boolean headingCorrect) {
		this.leftDistance = leftDistance;
		this.rightDistance = rightDistance;
		this.headingCorrect = headingCorrect;
	}
	
	/**
	 * Method which sets the navigator to rotate at 50 degrees/sec
	 * 
	 * 
	 * 
	 */
	public void setToSlow(boolean set) {
		this.setToSlow = set;
	}
	
	/**
	 * Method which performs the heading correction
	 * 
	 * @param leftDistance The distance sensed by the left color sensor (in cm)
	 * @param rightDistance The distance sensed by the right color sensor (in cm)
	 * @return non
	 */
	public void headingCorrection (double leftDistance, double rightDistance){
		
		Sound.buzz();
		
		double hypotenuse = LIGHT_SENSOR_DIST; 
		double opposite = Math.abs(leftDistance - rightDistance);
		
		double angle = Math.asin(opposite/hypotenuse) * 180/Math.PI;
		int angleToRotate = convertAngle(MainProgram.WHEEL_RADIUS, MainProgram.TRACK, angle);
		
		if (leftDistance > rightDistance) {
			leftMotor.setSpeed(SLOW); rightMotor.setSpeed(SLOW);
			rightMotor.rotate(-angleToRotate, true);
			leftMotor.rotate(angleToRotate, false);
		}
		else {
			leftMotor.setSpeed(SLOW); rightMotor.setSpeed(SLOW);
			rightMotor.rotate(angleToRotate, true);
			leftMotor.rotate(-angleToRotate, false);
		}
		

	}
	
}
