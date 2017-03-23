package Launching;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;


public class Launcher {
	
	// Static Resources:
		// Left motor connected to output A
		// Right motor connected to output D
		// Top Motors connected to outputs B and C
		private static final EV3LargeRegulatedMotor topMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
		private static final EV3LargeRegulatedMotor topMotor2 = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));


		// Constants
		private static final int ACCELERATION = 2500;	
		private static final int RETURN_ACCEL = 300;	
		private static final int SLOW = 75;
		private static final int LAUNCH_ANGLE1 = 90;
		private static final int LAUNCH_ANGLE2 = 45;
		private static final int INITIAL_ANGLE = 35;
		private static final int FOUR_TILES = 80;
		private static final int FIVE_TILES = 97;
		private static final int SIX_TILES = 180;
		private static final int SEVEN_TILES = 240;
		public static void main(String[] args) {
			int d1=7; //taken from wifi class

			//Rotate the motors forward to prepare for launch
			topMotor.setSpeed(100);
			topMotor2.setSpeed(100);


			topMotor.rotate(-INITIAL_ANGLE, true);
			topMotor2.rotate(-INITIAL_ANGLE, false);

			//Lock both of the top motors to prepare for the launch
			topMotor.stop();
			topMotor2.stop();

			while (true){
				int buttonChoice = Button.waitForAnyPress();
				if (buttonChoice == Button.ID_UP) {
					if (d1==4){
						//Launch the ball
						launch(FOUR_TILES, 150);
					}
					else if (d1==5){
						//Launch the ball
						launch(FIVE_TILES, 120);
					}
					else if (d1==6){
						//Launch the ball
						launch(SIX_TILES, LAUNCH_ANGLE1);
					}
					else if (d1==7){
						//Launch the ball
						launch(SEVEN_TILES, LAUNCH_ANGLE2);
					}

			}

			else if (buttonChoice == Button.ID_ESCAPE) {
				System.exit(0);
			}
				}
		}



		//Method which launches the ball
		private static void launch(int tiles, int angle) {

				//Prepare the motors for launch
				topMotor.resetTachoCount();
				topMotor2.resetTachoCount();
			
				//Set the accelerations of the top motors
				topMotor.setAcceleration((tiles* ACCELERATION)/100);
				topMotor2.setAcceleration((tiles * ACCELERATION)/100);

				//Set the speeds of the top motors
				topMotor.setSpeed(topMotor.getMaxSpeed());
				topMotor2.setSpeed(topMotor2.getMaxSpeed());


				
				//RotateMotors (-angle);
				// Move motors and store current position
				
				double Count1 = topMotor.getTachoCount();
				double Count2 = topMotor.getTachoCount();
				topMotor.backward();
				topMotor2.backward();
				
			
			
				
				while (true){
					
				Count2 = topMotor.getTachoCount();
				if ( Math.abs( Count1 - Count2 )== angle){
					
					topMotor.flt(true);
					Sound.beep();
					topMotor2.flt(false);
					break;
					
					
				}}
				
				
				
				topMotor.setAcceleration(RETURN_ACCEL);
				topMotor2.setAcceleration(RETURN_ACCEL);
				
				topMotor.setSpeed(SLOW);
				topMotor2.setSpeed(SLOW); 
				
				//Rotate the top motors forward to launch ball
				
				
				RotateMotors ((int)(Count1 - topMotor.getTachoCount()));
				//topMotor.stop();
				//topMotor2.stop();
				

		}

		public static void RotateMotors ( int angle){
			
			topMotor.rotate(angle, true);
			topMotor2.rotate(angle, false);
			
			
		}

}
