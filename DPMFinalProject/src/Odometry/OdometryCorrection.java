package Odometry;

import Navigation.Navigator;
import Odometry.Odometer.Direction;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;

public class OdometryCorrection extends Thread {
	/**
	 * Odometry Correction
	 * 
	 * @author Noah Levine, Ryan Servera, Rowan Kennedy 
	 * @version 4.0
	 * @since 2017-03-27
	 */

	private Odometer odometer;
	private double TILE = 30.48;
	private EV3ColorSensor leftColorSensor, rightColorSensor;
	// Arrays that contain the RGB values of light
	private float[] leftColor = { 0, 0, 0 }, rightColor = {0,0,0};
	private float[] leftColorsIniti = { 0, 0, 0 }, rightColorsIniti = {0,0,0}; // initial colors
	
	private final double SENSOR_DIST = 5.2;
	
	private Navigator nav;
	private int initialTileX, initialTileY;
	
	
	private static final int COUNT = 8;
	

	/**
	 * Constructor for OdometryCorrection
	 * 
	 * @param odometer The odometer 
	 * @param leftColorSensor The left color sensor
	 * @param rightColorSensor The right color sensor
	 * @param nav The navigator
	 * 
	 * @return OdometryCorrection The OdometryCorrection thread
	 * 
	 * 
	 */
	public OdometryCorrection(Odometer odometer, EV3ColorSensor leftColorSensor, EV3ColorSensor rightColorSensor, Navigator nav, int corner) {
		this.odometer = odometer;
		this.leftColorSensor = leftColorSensor;
		this.rightColorSensor = rightColorSensor;
		this.nav = nav;
		
		if (corner == 1) {
			this.initialTileX = 0;
			this.initialTileY = 0;
		}
		else if (corner == 2) {
			this.initialTileX = 0;
			this.initialTileY = 11;
		}
		else if (corner == 3) {
			this.initialTileX = 11;
			this.initialTileY = 11;
		}
		else if (corner == 4) {
			this.initialTileX = 11;
			this.initialTileY = 0;
		}
		
		
	}

	/**
	 * Run method for the thread
	 * 
	 * 
	 * 
	 * 
	 * 
	 */
	public void run() {
		
		initializeTile();
		leftColorSensor.getRGBMode().fetchSample(leftColorsIniti, 0);
		rightColorSensor.getRGBMode().fetchSample(rightColorsIniti, 0);
		
		double lastX = odometer.getX();
		double lastY = odometer.getY();
		Direction prevDirection = odometer.getDirection();
		
		while (true) {
			
			if(!(odometer.getDirection().equals(prevDirection))) {
				lastX = odometer.getX();
				lastY = odometer.getY();
				prevDirection = odometer.getDirection();
			}
			
			if (Math.abs(odometer.getX() - lastX) >= 32) {
				
				updateTileAndOdometer(odometer.getDirection(), true);
				lastX = odometer.getX();
				continue;

			}
			
			if (Math.abs(odometer.getY() - lastY) >= 32) {
				updateTileAndOdometer(odometer.getDirection(), true);
				lastY = odometer.getY();
				continue;
			}
			
			if (odometer.TILE[0] == initialTileX && Math.abs(odometer.getX() - lastX) > 15) {
				updateTileAndOdometer(odometer.getDirection(), true);
				lastX = odometer.getX();
				continue;
			}
			
			if (odometer.TILE[1] == initialTileY && Math.abs(odometer.getY() - lastY) > 15) {
				updateTileAndOdometer(odometer.getDirection(), true);
				lastY = odometer.getY();
				continue;
			}
			
			double leftDistance, rightDistance;
			
			if (!isLineOnLeft() && !isLineOnRight()) {
				continue;
			}
			
			
			if (isLineOnLeft() && !isLineOnRight()) {
				int count;
				
				if (isFacingX()) {
					leftDistance = odometer.getX();
					nav.setToSlow(true);
					
					count = 0;
					while (!isLineOnRight() && count < COUNT) {
						count++;
						Sound.beep();
					}
					 rightDistance = odometer.getX();
				}
				else {
					 leftDistance = odometer.getY();
					 nav.setToSlow(true);
					 
					count = 0;
					while (!isLineOnRight() && count < COUNT) {
						Sound.beep();
						count++;

					}
					rightDistance = odometer.getY();
				}
				
				if (count < COUNT) {
					nav.setHeadingCorrection(leftDistance, rightDistance, true);
					updateTileAndOdometer(odometer.getDirection(), false);
					lastX = odometer.getX();
					lastY = odometer.getY();

				}
			}
			
			else if (!isLineOnLeft() && isLineOnRight()) {
				
				int count;
				
				if(isFacingX()) {
					rightDistance = odometer.getX();
					nav.setToSlow(true);
					
					count = 0;
					while (!isLineOnLeft() && count < COUNT) {
						Sound.beepSequenceUp();
						count++;
						
					}
					leftDistance = odometer.getX();
				}
				else {
					rightDistance = odometer.getY();
					nav.setToSlow(true);
					
					count = 0;
					while (!isLineOnLeft() && count < COUNT) {
						Sound.beepSequenceUp();
						count++;
						
					}
					leftDistance = odometer.getY();
				}
				
					if (count < COUNT) {
						nav.setHeadingCorrection(leftDistance, rightDistance, true);
						updateTileAndOdometer(odometer.getDirection(), false);
						lastX = odometer.getX();
						lastY = odometer.getY();

					}
			}
			
			
			
			nav.setToSlow(false);
			
			try {
				Thread.sleep(800);
			} catch (InterruptedException e) {
				
				return;
			} 

		}

	}

	/*
	 * returns the tile the robot starts in
	 */
	/**
	 * Method which sets the initial tile of the robot
	 * 
	 * @param corner The starting corner of the robot
	 * 
	 * 
	 * 
	 */
	private void initializeTile() {
		// TODO fix this to get info from wifi
		odometer.TILE[0] = initialTileX;
		odometer.TILE[1] = initialTileY;
		

	}

	/*
	 * returns if there is a line on the left or not
	 */
	/**
	 * Method which returns whether the left color sensor senses a line
	 * 
	 * 
	 * @return boolean Whether a line was sensed
	 */
	public boolean isLineOnLeft() {
		leftColorSensor.getRGBMode().fetchSample(leftColor, 0);
		if (leftColor[0] < (leftColorsIniti[0] - 0.07) && leftColor[1] < leftColorsIniti[1] && leftColor[2] < (leftColorsIniti[2] - 0.004)) {
			return true;
		} else {
			return false;
		}
	}
	
	/**
	 * Method which returns whether the right color sensor senses a line
	 * 
	 * 
	 * @return boolean Whether a line was sensed
	 */
	public boolean isLineOnRight() {
		rightColorSensor.getRGBMode().fetchSample(rightColor, 0);
		if (rightColor[0] < (rightColorsIniti[0] - 0.07) && rightColor[1] < rightColorsIniti[1] && rightColor[2] < (rightColorsIniti[2] - 0.004)) {
			return true;
		} else {
			return false;
		}
	}
		
 
	/*
	 * called when a line is sensed and updates the odometer to the correct x or
	 * y value
	 */
	/**
	 * Method which updates the odometer for correction purposes
	 * 
	 * @param dir The direction which the robot is heading
	 * 
	 */
	public void updateTileAndOdometer(Odometer.Direction dir, boolean missedTile) {
		
		Sound.beepSequence();
		double[] position = { 0, 0, 0 };
		boolean[] update = { false, false, false };
		switch (dir) {
		case E:
			
			if (!missedTile) {
				position[0] = (odometer.TILE[0] * TILE) - SENSOR_DIST;
				update[0] = true;
				position[2] = 0;
				update[2] = true;
				odometer.setPosition(position, update);
			}
			
			odometer.TILE[0] += 1;
			

			break;
		case W:
			if (!missedTile) {
				position[0] = ((odometer.TILE[0] - 1) * TILE) + SENSOR_DIST;
				update[0] = true;
				position[2] = 180;
				update[2] = true;
				odometer.setPosition(position, update);
			}
			
			odometer.TILE[0] -= 1;
			break;
		case N:
			if(! missedTile) {
				position[1] = ((odometer.TILE[1]) * TILE) - SENSOR_DIST;
				update[1] = true;			
				position[2] = 90;
				update[2] = true;
				odometer.setPosition(position, update);
			}
			odometer.TILE[1] += 1;
			break;
		case S:
			if (!missedTile) {
				position[1] = ((odometer.TILE[1] - 1) * TILE) + SENSOR_DIST;
				update[1] = true;
				position[2] = 270;
				update[2] = true;
				odometer.setPosition(position, update);
			}
			
			odometer.TILE[1] -= 1;
			break;
		}
		
	}
	
	/**
	 * Method which returns whether the robot is traveling in the X direction
	 * 
	 * 
	 * @return boolean Whether the robot is traveling in X direction
	 */
	private boolean isFacingX(){
		if(odometer.getDirection().equals(Direction.E) || odometer.getDirection().equals(Direction.W))
			return true;
		else 
			return false;
	}

}