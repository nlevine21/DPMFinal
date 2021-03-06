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
		private static final int LAUNCH_ANGLE5 = 230;
		private static final int LAUNCH_ANGLE6 = 230;
		private static final int LAUNCH_ANGLE7 = 230; // AMAZING
		private static final int LAUNCH_ANGLE8 = 230; // OK
		private static final int INITIAL_ANGLE = 15;	
		private static final double FIVE_TILES = 3;
		private static final double SIX_TILES = 3.2;
		private static final double SEVEN_TILES = 3.8;
		private static final double EIGHT_TILES = 4;
		private static final int PRIME_ANGLE = 160;  // NEED TO CALCULATE THIS EXPERIMENTALLY. IT IS THE ANGLE TO MAKE ARM UP
		
		
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
			topMotor.rotate(-INITIAL_ANGLE, true);
			topMotor2.rotate(-INITIAL_ANGLE, false);
			
			
			//Lock both of the top motors to prepare for the launch
			topMotor.stop();
			topMotor2.stop();
			
			
				
				 if (TILES==5){
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
		private void launch(double tiles, double angle) {
			
				
				
				//Set the accelerations of the top motors
				topMotor.setAcceleration(ACCELERATION); 
				topMotor2.setAcceleration(ACCELERATION);
				
				//Set the speeds of the top motors
				topMotor.setSpeed(((int)(100*tiles/8)*topMotor.getMaxSpeed())/100);
				topMotor2.setSpeed(((int)((100*tiles/8)*topMotor.getMaxSpeed())/100));
				


				// Move motors and store current position
				
				double Count1 = topMotor.getTachoCount();
				double Count2;
				
				
				topMotor.backward();
				topMotor2.backward();
				
			
			
				
				while (true){
				
				Sound.beep();
				Count2 = topMotor.getTachoCount();
				if (Math.abs((Count2 -  Count1)) <= angle){
					
					Sound.buzz();
					topMotor.flt(true);
					topMotor2.flt(true);
				
				}
				break;
				}
				
					delay (6000);
					
					topMotor.rotate(PRIME_ANGLE+110, true);
					topMotor2.rotate(PRIME_ANGLE+90, false);
					
					
				delay(1500);
				
				topMotor.flt(true);
				topMotor2.flt(true);
				
				delay(3000);


						Sound.beepSequenceUp();
					
						
						topMotor.rotate(-PRIME_ANGLE, true);
						topMotor2.rotate(-PRIME_ANGLE, false);
						
						topMotor.stop();
						topMotor2.stop();
						
						Sound.beepSequence();
						
						
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