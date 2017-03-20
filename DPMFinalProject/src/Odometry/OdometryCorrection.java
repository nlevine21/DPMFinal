package Odometry;

import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class OdometryCorrection extends Thread {

	private Odometer odometer;
	private double TILE = 30.48;
	private SensorModes colorSensor;
	private SampleProvider colorSample;
	
	// Arrays that contain the RGB values of light
	private float[] color = { 0, 0, 0 };
	private float[] colorsIniti = { 0, 0, 0 }; // initial colors

	public OdometryCorrection(Odometer odometer, EV3ColorSensor colorSensor) {
		this.odometer = odometer;
		this.colorSensor = colorSensor;
		this.colorSample = this.colorSensor.getMode("Red");
	}

	public void run() {
		initialTile();
		colorSample.fetchSample(colorsIniti, 0);
		while (true) {
			if (isLine()) {
				Sound.beep();
				updateTileAndOdometer(odometer.getDirection());

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
	 * returns if there is a line or not
	 */
	public boolean isLine() {
		colorSample.fetchSample(color, 0);
		if (color[0] < (colorsIniti[0] - 0.05) && color[1] < colorsIniti[1] && color[2] < (colorsIniti[2] - 0.003)) {
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

}
