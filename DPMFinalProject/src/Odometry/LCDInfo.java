package Odometry;

import Odometry.Odometer;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class LCDInfo implements TimerListener {
	public static final int LCD_REFRESH = 100;
	private Odometer odo;
	private Timer lcdTimer;
	private TextLCD LCD = LocalEV3.get().getTextLCD();;

	// arrays for displaying data
	private double[] pos;

	public LCDInfo(Odometer odo) {
		this.odo = odo;
		this.lcdTimer = new Timer(LCD_REFRESH, this);

		// initialise the arrays for displaying data
		pos = new double[3];

		// start the timer
		lcdTimer.start();
		
		LCD.clear();
	}

	public void timedOut() {
		odo.getPosition(pos);
		LCD.clear();
		LCD.drawString("X: ", 0, 0);
		LCD.drawString("Y: ", 0, 1);
		LCD.drawString("H: ", 0, 2);
		LCD.drawString("D: ", 0, 3);
		LCD.drawInt((int) (pos[0] * 1), 3, 0);
		LCD.drawInt((int) (pos[1] * 1), 3, 1);
		LCD.drawInt((int) pos[2], 3, 2);
		LCD.drawString((String) odo.getDirection().name(), 3, 3);
		LCD.drawString("X: " + odo.TILE[0] + " Y: " + odo.TILE[1], 4, 4);

	}
}
