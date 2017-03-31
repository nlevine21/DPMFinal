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
		private int TILES;

		// Constants
		private static final int ACCELERATION = 6000;
		private static final int LAUNCH_ANGLE4 = 90;
		private static final int LAUNCH_ANGLE5 = 90;
		private static final int LAUNCH_ANGLE6 = 90;
		private static final int LAUNCH_ANGLE7 = 90;
		private static final int LAUNCH_ANGLE8 = 90;
		private static final int INITIAL_ANGLE = 30;	
		private static final int FOUR_TILES = 4;
		private static final int FIVE_TILES = 5;
		private static final int SIX_TILES = 6;
		private static final int SEVEN_TILES = 7;
		private static final int EIGHT_TILES = 8;
		private static final int PRIME_ANGLE = 130;  // NEED TO CALCULATE THIS EXPERIMENTALLY. IT IS THE ANGLE TO MAKE ARM UP
		
		
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
			this.TILES = distance;
			
		}
		
		/**
		 * This method launches the ball
		 * 
		 * 
		 *     	
		 */
		public void launchBall() {
			//Rotate the motors forward to prepare for launch
	
			
			
			//Lock both of the top motors to prepare for the launch
			topMotor.stop();
			topMotor2.stop();
			
		
				
					if (TILES==4){
						//Launch the ball
						launch(FOUR_TILES, LAUNCH_ANGLE4);
					}
					else if (TILES==5){
						//Launch the ball
						launch(FIVE_TILES, LAUNCH_ANGLE5);
					}
					else if (TILES==6){
						//Launch the ball
						launch(SIX_TILES, LAUNCH_ANGLE6);
					}
					else if (TILES==7){
						//Launch the ball
						launch(SEVEN_TILES, LAUNCH_ANGLE7);
					}
					else if (TILES==8){
						//Launch the ball
						launch(EIGHT_TILES, LAUNCH_ANGLE8);
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
				topMotor.setAcceleration(ACCELERATION); 
				topMotor2.setAcceleration(ACCELERATION);
				
				//Set the speeds of the top motors
				topMotor.setSpeed((tiles / 8) * topMotor.getMaxSpeed());
				topMotor2.setSpeed((tiles/8)*topMotor2.getMaxSpeed());
				
				// Set up tachometer values
				topMotor.resetTachoCount();
			
				
				// Set the motors to move
				topMotor.backward();
				topMotor2.backward();
				
				while (true) {
					
					if (topMotor.getTachoCount() >= angle){
						
						topMotor.flt(true);
						topMotor.flt(false);
						break;
					}
					
					delay (5000);
					
					// Set return speed
					
					topMotor.setSpeed(100);
					topMotor2.setSpeed(100);
				
					// Bring arm back down
					
					topMotor.forward();
					topMotor2.forward();
					topMotor.setStallThreshold(1, 2000);
					
					delay(5000);

					
					if (topMotor.isStalled() == true){
						
						Sound.beepSequenceUp();
						
						topMotor.stop();
						topMotor2.stop();
						
						topMotor.rotate(-PRIME_ANGLE, true);
						topMotor2.rotate(-PRIME_ANGLE, false);
						
						topMotor.stop();
						topMotor2.stop();
						
						Sound.beepSequence();
					}
				}
				
				
				
		}
		
		
		/**
		 * This method delays the thread by 2 seconds for synchronization
		 * 
		 * 
		 *     	
		 */
		private static void delay(int time) {
			Delay.msDelay(time);
		}
}
