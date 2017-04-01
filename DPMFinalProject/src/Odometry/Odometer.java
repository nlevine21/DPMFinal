
package Odometry;

/*


 * 

 * File: Odometer.java
 * Written by: Sean Lawlor
 * ECSE 211 - Design Principles and Methods, Head TA
 * Fall 2011
 * Ported to EV3 by: Francois Ouellet Delorme
 * Fall 2015
 * 
 * Class which controls the odometer for the robot
 * 
 * Odometer defines cooridinate system as such...
 * 
 * 					90Deg:pos y-axis
 * 							|
 * 							|
 * 							|
 * 							|
 * 180Deg:neg x-axis------------------0Deg:pos x-axis
 * 							|
 * 							|
 * 							|
 * 							|
 * 					270Deg:neg y-axis
 * 
 * The odometer is initalized to 90 degrees, assuming the robot is facing up the positive y-axis
 * 
 */

import lejos.utility.Timer;
import lejos.utility.TimerListener;
import Main.MainProgram;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer implements TimerListener {
	/**
	 * Odometer obtained from myCourses with some small changes
	 * 
	 * @author Unknown, Rowan Kennedy 
	 * @version 1.0
	 * @since Unknown
	 */

	private Timer timer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private final int DEFAULT_TIMEOUT_PERIOD = 20;
	private double leftRadius, rightRadius, width;
	private double x, y, theta;
	private double[] oldDH, dDH;
	
	public int lineCount;
	
	/**
	 * Enum representing the Direction of the Robot (N,E,S,W)
	 * 
	 * @author Rowan Kennedy 
	 * @version 1.0
	 * @since 2017-03-15
	 */
	
	public enum Direction {
		N, E, S, W
	};
	

	public Direction currentDirection;
	public int[] TILE = { 0, 0 };

	// constructor
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, int INTERVAL,
			boolean autostart) {

		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		// default values, modify for your robot
		this.rightRadius = MainProgram.WHEEL_RADIUS;
		this.leftRadius = MainProgram.WHEEL_RADIUS;
		this.width = MainProgram.TRACK;

		this.x = 0.0;
		this.y = 0.0;
		this.theta = 0.0;
		this.oldDH = new double[2];
		this.dDH = new double[2];
		

		if (autostart) {
			// if the timeout interval is given as <= 0, default to 20ms timeout
			this.timer = new Timer((INTERVAL <= 0) ? INTERVAL : DEFAULT_TIMEOUT_PERIOD, this);
			this.timer.start();
		} else
			this.timer = null;
	}
	

	// functions to start/stop the timerlistener
	public void stop() {
		if (this.timer != null)
			this.timer.stop();
	}

	public void start() {
		if (this.timer != null)
			this.timer.start();
	}

	/*
	 * Calculates displacement and heading as title suggests
	 */
	private void getDisplacementAndHeading(double[] data) {
		int leftTacho, rightTacho;
		leftTacho = leftMotor.getTachoCount();
		rightTacho = rightMotor.getTachoCount();

		data[0] = (leftTacho * leftRadius + rightTacho * rightRadius) * Math.PI / 360.0;
		data[1] = (rightTacho * rightRadius - leftTacho * leftRadius) / width;
	}

	/*
	 * Recompute the odometer values using the displacement and heading changes
	 */
	public void timedOut() {
		this.getDisplacementAndHeading(dDH);
		dDH[0] -= oldDH[0];
		dDH[1] -= oldDH[1];

		// update the position in a critical region
		synchronized (this) {
			theta += dDH[1];
			theta = fixDegAngle(theta);

			x += dDH[0] * Math.cos(Math.toRadians(theta));
			y += dDH[0] * Math.sin(Math.toRadians(theta));
		}

		oldDH[0] += dDH[0];
		oldDH[1] += dDH[1];
	}

	// return X value
	public double getX() {
		synchronized (this) {
			return x;
		}
	}

	// return Y value
	public double getY() {
		synchronized (this) {
			return y;
		}
	}

	// return theta value
	public double getAng() {
		synchronized (this) {
			return theta;
		}
	}

	// set x,y,theta
	public void setPosition(double[] position, boolean[] update) {
		synchronized (this) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	// return x,y,theta
	public void getPosition(double[] position) {
		synchronized (this) {
			position[0] = x;
			position[1] = y;
			position[2] = theta;
		}
	}

	public double[] getPosition() {
		synchronized (this) {
			return new double[] { x, y, theta };
		}
	}

	// accessors to motors
	public EV3LargeRegulatedMotor[] getMotors() {
		return new EV3LargeRegulatedMotor[] { this.leftMotor, this.rightMotor };
	}

	public EV3LargeRegulatedMotor getLeftMotor() {
		return this.leftMotor;
	}

	public EV3LargeRegulatedMotor getRightMotor() {
		return this.rightMotor;
	}

	// static 'helper' methods
	public static double fixDegAngle(double angle) {
		if (angle < 0.0)
			angle = 360.0 + (angle % 360.0);

		return angle % 360.0;
	}

	public static double minimumAngleFromTo(double a, double b) {
		double d = fixDegAngle(b - a);

		if (d < 180.0)
			return d;
		else
			return d - 360.0;
	}


	/*
	 * returns the current direction robot is facing: N,E,S or W
	 */
	/**
	 * Method which returns the direction of the odometer
	 * 
	 *
	 *
	 */
	public Direction getDirection() {
		if (theta > 315 || theta <= 45) {
			return Direction.E;
		} else if (theta > 45 & theta <= 135) {
			return Direction.N;
		} else if (theta > 135 & theta <= 225) {
			return Direction.W;
		} else {
			return Direction.S;
		}

	}

}
