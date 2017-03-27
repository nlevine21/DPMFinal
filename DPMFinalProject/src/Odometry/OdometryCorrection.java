package Odometry;

import Navigation.Navigator;
import Odometry.Odometer.Direction;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;

public class OdometryCorrection extends Thread {

	private Odometer odometer;
	private double TILE = 30.48;
	private EV3ColorSensor leftColorSensor, rightColorSensor;
	// Arrays that contain the RGB values of light
	private float[] leftColor = { 0, 0, 0 }, rightColor = {0,0,0};
	private float[] colorsIniti = { 0, 0, 0 }; // initial colors
	
	private double distance1, distance2;
	private Navigator nav;

	public OdometryCorrection(Odometer odometer, EV3ColorSensor leftColorSensor, EV3ColorSensor rightColorSensor, Navigator nav) {
		this.odometer = odometer;
		this.leftColorSensor = leftColorSensor;
		this.rightColorSensor = rightColorSensor;
		this.nav = nav;
		
	}

	public void run() {
		initialTile();
		leftColorSensor.getRGBMode().fetchSample(colorsIniti, 0);
		rightColorSensor.getRGBMode().fetchSample(colorsIniti, 0);
		
		while (true) {
			
			
			double leftDistance, rightDistance;
			
			if (!isLineOnLeft() && !isLineOnRight()) {
				continue;
			}
			
			
			if (isLineOnLeft() && !isLineOnRight()) {
				if (isFacingX()) {
					leftDistance = odometer.getX();
					nav.setToSlow(true);
					while (!isLineOnRight()) {
						Sound.beep();
					}
					 rightDistance = odometer.getX();
				}
				else {
					 leftDistance = odometer.getY();
					 nav.setToSlow(true);
					while (!isLineOnRight()) {

					}
					rightDistance = odometer.getY();
				}
				
				nav.setHeadingCorrection(leftDistance, rightDistance, true);
			}
			
			else if (!isLineOnLeft() && isLineOnRight()) {
				
				if(isFacingX()) {
					rightDistance = odometer.getX();
					nav.setToSlow(true);
					while (!isLineOnLeft()) {
						
					}
					leftDistance = odometer.getX();
				}
				else {
					rightDistance = odometer.getY();
					nav.setToSlow(true);
					while (!isLineOnLeft()) {
						
					}
					leftDistance = odometer.getY();
				}
				
				nav.setHeadingCorrection(leftDistance, rightDistance, true);
			}
			
			nav.setToSlow(false);
			
			updateTileAndOdometer(odometer.getDirection());
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

		}

	}

	/*
	 * returns the tile the robot starts in
	 */
	private void initialTile() {
		// TODO fix this to get info from wifi
		odometer.TILE[0] = 0;
		odometer.TILE[1] = 0;

	}

	/*
	 * returns if there is a line on the left or not
	 */
	public boolean isLineOnLeft() {
		leftColorSensor.getRGBMode().fetchSample(leftColor, 0);
		if (leftColor[0] < (colorsIniti[0] - 0.05) && leftColor[1] < colorsIniti[1] && leftColor[2] < (colorsIniti[2] - 0.005)) {
			return true;
		} else {
			return false;
		}
	}
	
	public boolean isLineOnRight() {
		rightColorSensor.getRGBMode().fetchSample(rightColor, 0);
		if (rightColor[0] < (colorsIniti[0] - 0.05) && rightColor[1] < colorsIniti[1] && rightColor[2] < (colorsIniti[2] - 0.005)) {
			return true;
		} else {
			return false;
		}
	}
		
 
	/*
	 * called when a line is sensed and updates the odometer to the correct x or
	 * y value
	 */
	public void updateTileAndOdometer(Odometer.Direction dir) {
		double[] position = { 0, 0, 0 };
		boolean[] update = { false, false, false };
		switch (dir) {
		case E:
			position[0] = odometer.TILE[0] * TILE;
			update[0] = true;
			odometer.setPosition(position, update);
			odometer.TILE[0] += 1;

			break;
		case W:
			position[0] = (odometer.TILE[0] - 1) * TILE;
			update[0] = true;
			odometer.setPosition(position, update);
			odometer.TILE[0] -= 1;
			break;
		case N:
			position[1] = (odometer.TILE[1]) * TILE;
			update[1] = true;
			odometer.setPosition(position, update);
			odometer.TILE[1] += 1;
			break;
		case S:
			position[1] = (odometer.TILE[1] - 1) * TILE;
			update[1] = true;
			odometer.setPosition(position, update);
			odometer.TILE[1] -= 1;
			break;
		}
		
	}
	
	private boolean isFacingX(){
		if(odometer.getDirection().equals(Direction.E) || odometer.getDirection().equals(Direction.W))
			return true;
		else 
			return false;
	}

}