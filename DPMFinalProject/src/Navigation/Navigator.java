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

public class Navigator {
	public final static int FAST = 200;
	static final int SLOW = 100;
	static final int ACCELERATION = 4000;
	final static int DETECTABLE_DISTANCE = 20;
	final static double DEG_ERR = 3.0, CM_ERR = .5;
	private Odometer odometer;
//	private Direction currentDirection1;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private EV3UltrasonicSensor leftUsSensor, middleUsSensor;
	
	private boolean headingCorrect, setToSlow;
	private double leftDistance, rightDistance;
	
	private final double LIGHT_SENSOR_DIST = 9.9;
	
	public int lineCount;
	
//	private static final double xshoot = 152.4, yshoot = 45.72; // point to
																// shoot from

	public Navigator(Odometer odo, EV3UltrasonicSensor leftUsSensor, EV3UltrasonicSensor middleUsSensor) {
		this.lineCount = 0;
		
		this.odometer = odo;
		this.leftUsSensor = leftUsSensor;
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
	 * method used to test odometry correction, not important
	 */
	public void path() {
		turnLeft();
		turnLeft();
		turnLeft();
		turnLeft();

		float deg = convertDistance(MainProgram.WHEEL_RADIUS, 0);
		leftMotor.rotate((int) deg, true);
		rightMotor.rotate((int) deg, false);
		// turnRight();

	}

	/*
	 * Called if an object appears in front of it and navigates around it. Uses
	 * 90 degree turns needs to take coordinates it's traveling too
	 */
	public void avoid() {
		// check if left is are free
		boolean leftBlocked = false;
		
		if (isObstacle(leftUsSensor)) {
			leftBlocked = true;
		}

		// if one isn't free turn other way
		if (leftBlocked) {
			// turn right
			turnRight();
			travelForward();
		} else {
			// turn left
			turnLeft();
			travelForward();

		}
	}


	/*
	 * robot turns 90 degrees to the left
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
	private void travelForward() {
		float deg = convertDistance(MainProgram.WHEEL_RADIUS, 30);
		leftMotor.setSpeed(FAST);
		rightMotor.setSpeed(FAST);
		// System.out.println("forward");
		// System.out.println(deg);
		//leftMotor.forward();		SAME THING AS BEFORE. SHOULDNT BE HERE
		//rightMotor.forward();
		leftMotor.rotate((int) deg, true);
		rightMotor.rotate((int) deg, false);
	}

	/*
	 * checks if US sensor can detect an object in adjacent tile
	 */
	public boolean isObstacle(EV3UltrasonicSensor usSensor) {
		boolean isObstacle = false;
		float[] distance = { 0 };
		usSensor.fetchSample(distance, 0);
		distance[0] = distance[0] * 100;
		// System.out.print(" d: ");
		// System.out.println(distance[0]);
		if (distance[0] < DETECTABLE_DISTANCE) {
			isObstacle = true;
		}
		return isObstacle;
	}

	/*
	 * Functions to set the motor speeds jointly
	 */
	public void setSpeeds(float lSpd, float rSpd) {
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
					
					this.setSpeeds(FAST, FAST);
					if (setToSlow) {
						this.setSpeeds(50, 50);
					}

					
					if (isObstacle(middleUsSensor)) {
						this.setSpeeds(0, 0);
						delay();
						avoid();
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
					this.setSpeeds(FAST, FAST);
					if (setToSlow) {
						this.setSpeeds(50, 50);
					}

					
					if (isObstacle(middleUsSensor)) {
						this.setSpeeds(0, 0);
						delay();
						avoid();
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
					this.setSpeeds(FAST, FAST);
					if (setToSlow) {
						this.setSpeeds(50, 50);
					}

					
					if (isObstacle(middleUsSensor)) {
						this.setSpeeds(0, 0);
						delay();
						avoid();
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
					this.setSpeeds(FAST, FAST);
					if (setToSlow) {
						this.setSpeeds(50, 50);
					}

					
					if (isObstacle(middleUsSensor)) {
						this.setSpeeds(0, 0);
						delay();
						avoid();
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

	public void goForward(double distance) {
		if (distance < 0) {
			this.setSpeeds(-FAST, -FAST);
		} else {
			this.setSpeeds(FAST, FAST);
		}

		leftMotor.rotate(convertDistance(MainProgram.WHEEL_RADIUS, distance), true);
		rightMotor.rotate(convertDistance(MainProgram.WHEEL_RADIUS, distance), false);
	}

	// converts amount of rotation based on the wheel radius
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	// calculates the rotations need to get to a certain angle
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	public void delay() {
		Delay.msDelay(2000);
	}
	
	public void setHeadingCorrection(double leftDistance, double rightDistance, boolean headingCorrect) {
		this.leftDistance = leftDistance;
		this.rightDistance = rightDistance;
		this.headingCorrect = headingCorrect;
	}
	
	public void setToSlow(boolean set) {
		this.setToSlow = set;
	}
	
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
