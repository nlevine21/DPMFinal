package Navigation;

import Main.MainProgram;
import Odometry.Odometer;
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

public class Navigator {
	final static int FAST = 200, SLOW = 100, ACCELERATION = 4000;
	final static int DETECTABLE_DISTANCE = 30;
	final static double DEG_ERR = 3.0, CM_ERR = 1.0;
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private EV3UltrasonicSensor leftUsSensor, middleUsSensor, rightUsSensor;

	public Navigator(Odometer odo, EV3UltrasonicSensor leftUsSensor, EV3UltrasonicSensor middleUsSensor, EV3UltrasonicSensor rightUsSensor) {
		this.odometer = odo;
		this.leftUsSensor=leftUsSensor;
		this.middleUsSensor=middleUsSensor;
		this.rightUsSensor=rightUsSensor;
		

		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];

		// set acceleration
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
	}
	
	/*
	 * Called if an object appears in front of it and navigates around it. Uses
	 * 90 degree turns needs to take coordinates it's traveling too
	 */
	private void avoid() {
		// check if left or right are free
		boolean leftBlocked = false;
		boolean rightBlocked = false;
		if (isObstacle(leftUsSensor)) {
			leftBlocked = true;
		}
		if (isObstacle(rightUsSensor)) {
			rightBlocked = true;
		}

		// if one isn't free turn other way
		if (leftBlocked & !rightBlocked) {
			// turn right
			turnRight();
			travelForward();
		} else if (rightBlocked & !leftBlocked) {
			// turn left
			turnLeft();
			travelForward();

		}

		// if both aren't free reverse 30cm, call avoid()
		else if (rightBlocked & leftBlocked) {
			reverse();
			avoid();
		}

		// if both are free chose direction which will bring robot closer to
		// destination
		else {
			// will need to be changed to shortest turn in direction of
			// destination, for now turn left
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
		System.out.println("left");
		System.out.println(deg);
		leftMotor.setSpeed(FAST);
		rightMotor.setSpeed(FAST);
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
		leftMotor.setSpeed(FAST);
		rightMotor.setSpeed(FAST);
		System.out.println("right");
		System.out.println(deg);
		leftMotor.forward();
		rightMotor.backward();
		leftMotor.rotate((int) deg, true);
		rightMotor.rotate((int) -deg, false);
		leftMotor.stop();
		rightMotor.stop();
	}

	/*
	 * robot reverses one square (30cm)
	 */
	private void reverse() {
		float deg = convertDistance(MainProgram.WHEEL_RADIUS, 30);
		leftMotor.setSpeed(FAST);
		rightMotor.setSpeed(FAST);
		leftMotor.rotate((int) -deg, true);
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
		leftMotor.forward();
		rightMotor.forward();
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
//		System.out.print(" d: ");
//		System.out.println(distance[0]);
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
		this.leftMotor.flt(true);
		this.rightMotor.flt(true);
	}

	/*
	 * TravelTo function which takes as arguments the x and y position in cm Will travel to designated position, while
	 * constantly updating it's heading
	 */
	public void travelTo(double x, double y) {
		double minAng;
		while (Math.abs(x - odometer.getX()) > CM_ERR || Math.abs(y - odometer.getY()) > CM_ERR) {
			minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);
			if (minAng < 0)
				minAng += 360.0;
			this.turnTo(minAng, false);
			this.setSpeeds(FAST, FAST);
		}
		this.setSpeeds(0, 0);
	}

	/*
	 * TurnTo function which takes an angle and boolean as arguments The boolean controls whether or not to stop the
	 * motors when the turn is completed
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
		}
		else {
			this.setSpeeds(FAST,FAST);
		}
		
		leftMotor.rotate(convertDistance(MainProgram.WHEEL_RADIUS, distance), true);
		rightMotor.rotate(convertDistance(MainProgram.WHEEL_RADIUS, distance), false);
	}
	
	//converts amount of rotation based on the wheel radius
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	// calculates the rotations need to get to a certain angle
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
