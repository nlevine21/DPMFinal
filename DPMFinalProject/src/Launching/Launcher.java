package Launching;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.utility.Delay;

/**
 * The Launcher Arm of The Robot
 * 
 * @author Darius Piecaitis, Steven Cangul
 * @version 2.0
 * @since 2017-03-23  	
 */

public class Launcher {
	
	// Static Resources:
		// Left motor connected to output A
		// Right motor connected to output D
		// Top Motors connected to outputs B and C
		private  EV3LargeRegulatedMotor topMotor, topMotor2;
		private int d1;

		// Constants
		private static final int ACCELERATION = 2500;
		private static final int LAUNCH_ANGLE1 = 90;
		private static final int LAUNCH_ANGLE2 = 170;
		private static final int LAUNCH_ANGLE3 = 230;
		private static final int INITIAL_ANGLE = 30;	
		private static final int FOUR_TILES = 80;
		private static final int FIVE_TILES = 80;
		private static final int SIX_TILES = 180;
		private static final int SEVEN_TILES = 240;
		private static final int EIGHT_TILES = 240;
		
		
		/**
		 * Constructor of the Launcher
		 * 
		 * @param topMotor One of the launching arm motors
		 * @param topMotor2 The other launching arm motor
		 * @param distance The distance that the launcher must shoot (in Tile Units)
		 * 			
		 * @return Launcher The Launcher Object    	
		 */
		public Launcher (EV3LargeRegulatedMotor topMotor, EV3LargeRegulatedMotor topMotor2, int distance) {
			this.topMotor = topMotor;
			this.topMotor2 = topMotor2;
			this.d1 = distance;
			
		}
		
		/**
		 * This method launches the ball
		 * 
		 * 
		 *     	
		 */
		public void launchBall() {
			//Rotate the motors forward to prepare for launch

			delay();
			
			topMotor.rotate(-INITIAL_ANGLE, true);
			topMotor2.rotate(-INITIAL_ANGLE, false);
			
			delay();

			
			//Lock both of the top motors to prepare for the launch
			topMotor.stop();
			topMotor2.stop();
			
		
				
					if (d1==4){
						//Launch the ball
						launch(FOUR_TILES, LAUNCH_ANGLE1);
					}
					else if (d1==5){
						//Launch the ball
						launch(FIVE_TILES, LAUNCH_ANGLE1);
					}
					else if (d1==6){
						//Launch the ball
						launch(SIX_TILES, LAUNCH_ANGLE1);
					}
					else if (d1==7){
						//Launch the ball
						launch(SEVEN_TILES, LAUNCH_ANGLE2);
					}
					else if (d1==8){
						//Launch the ball
						launch(EIGHT_TILES, LAUNCH_ANGLE3);
					}
				
		


	}
		
		
		
		/**
		 * This is a helper method to launchBall which sets the acceleration
		 * of the launch motors appropriately 
		 * 
		 * @param tiles The number of tiles to launch
		 * @param angle The angle which the launch motors must rotate
		 *     	
		 */
		private void launch(int tiles, int angle) {
			
				//Set the accelerations of the top motors
				topMotor.setAcceleration((tiles* ACCELERATION)/100);
				topMotor2.setAcceleration((tiles * ACCELERATION)/100);
				
				//Set the speeds of the top motors
				topMotor.setSpeed(topMotor.getMaxSpeed());
				topMotor2.setSpeed(topMotor2.getMaxSpeed());
				
				//Rotate the top motors forward to launch ball
				topMotor.rotate(-angle, true);
				topMotor2.rotate(-angle, false);
				
				//Rotate the top motors back to the initial orientation
				topMotor.rotate(angle,true);
				topMotor2.rotate(angle,false);
				
		}
		
		
		/**
		 * This method delays the thread by 2 seconds for synchronization
		 * 
		 * 
		 *     	
		 */
		private static void delay() {
			Delay.msDelay(2000);
		}
}
