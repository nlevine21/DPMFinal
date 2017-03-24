package Localization;

import Navigation.Navigator;
import Odometry.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class USLocalizer2{

	private Odometer odometer;
	private EV3UltrasonicSensor rightUSSensor;
	private Navigator nav;

	public USLocalizer2(Odometer odometer, Navigator nav, EV3UltrasonicSensor rightUSSensor) {
		this.odometer = odometer;
		this.rightUSSensor = rightUSSensor;
		this.nav = nav;
		nav.setSpeeds(Navigator.FAST, Navigator.FAST);
	}

	public void run() {
		int[] firstTile = { 1, 0 };
		int[] secondTile = { 2, 0 };

		while (true) {
			if (odometer.TILE[0] == firstTile[0]) {
				
	
				float[] sample1 = { 0, 0, 0, 0 };
				rightUSSensor.fetchSample(sample1, 0);
				
				while (true) {
					if (odometer.TILE[0] != firstTile[0]) {
						if ((odometer.TILE[0] == secondTile[0]) & (odometer.getDirection() == Odometer.Direction.E)) {
							float[] sample2 = { 0, 0, 0, 0 };
							rightUSSensor.fetchSample(sample2, 0);
							nav.setSpeeds(0,0);
							nav.delay();
							doThetaCorrection(sample1, sample2);
							nav.turnTo(0, true);
							return;
						} 
					}
				}
			}
		}
	}

	private void doThetaCorrection(float[] sample1, float[] sample2) {
		
		float total1 = 0;
		float total2 = 0;
		// take average of 1 and 2
		for (int i = 0; i < 4; i++) {
			total1 = total1 + sample1[i];
			total2 = total2 + sample2[i];
		}
		double mean1 = total1 / 4 * 100;
		double  mean2 = total2 / 4 * 100;
		double theta = Math.atan2((mean2 - mean1), 30.48) * 180 / Math.PI;
		if (theta < 0) {
			theta = theta + 360;
		}
		double[] odoValues = { 30.48, 0, theta };
		boolean[] update = { true, false, true };

		odometer.setPosition(odoValues, update);

	}

}