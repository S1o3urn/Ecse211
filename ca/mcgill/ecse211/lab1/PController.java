package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 300;
	private static final int FILTER_OUT = 20;

	private final int bandCenter;
	private final int bandWidth;
	private int distance;
	private int currentLeftSpeed;
	private int currentRightSpeed;
	private int filterControl;

	public PController(int bandCenter, int bandwidth) {
		this.bandCenter = bandCenter;
		this.bandWidth = bandwidth;
		this.filterControl = 0;

		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);

		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();

		currentLeftSpeed = MOTOR_SPEED;
		currentRightSpeed = MOTOR_SPEED;
	}

	@Override
	public void processUSData(int distance) {

		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
		// There is an abnormally large value
		if ((distance >= 75) && (filterControl < FILTER_OUT)) {
			filterControl++;
		}

		// FILTER_OUT amount of repeated large value
		// meaning there could be nothing there to scan
		else if (distance >= 75) {
			this.distance = distance;
		}
		
		//Filtered values
		else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;

			// Create error variable used throughout code
			// error used to divide later, for now if 0, change to 1
			int error = Math.abs(this.distance - bandCenter);
			if (error == 0) {
				error = 1;
			}
			// TODO: process a movement based on the us distance passed in (P style)
			
			//Robot is on right path, advance straight
			if((this.distance >= (bandCenter - bandWidth)) 
					&& (this.distance <= (bandCenter + bandWidth))) {
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
			}
			
			//Robot is too close to wall
			else if((this.distance < (bandCenter - bandWidth))) {
				//Need to turn right
				//Increase left motor speed proportionally to error
				//Decrease right motor speed proportionally to error
				currentLeftSpeed = MOTOR_SPEED * error / bandWidth;
				currentRightSpeed = MOTOR_SPEED * bandWidth / error;
				
				//Set a speed limit to avoid overworking motors
				if(currentLeftSpeed > 400) {
					currentLeftSpeed = 425;
				}
				
				if(currentRightSpeed < 100) {
					currentRightSpeed = 125;
				}
				
				//Change motor speeds
				WallFollowingLab.leftMotor.setSpeed(currentLeftSpeed);
				WallFollowingLab.rightMotor.setSpeed(currentRightSpeed);
			}
			
			else if (this.distance > (bandCenter - bandWidth)) {
				//Need to turn left
				//Decrease left motor speed proportionally to error
				//Increase right motor speed proportionally to error
				currentRightSpeed = MOTOR_SPEED * error / (bandWidth);
				currentLeftSpeed = MOTOR_SPEED * (bandWidth) / error;
				
				//we don't want the motor to run too fast, so we add a min and max speed

				if (currentRightSpeed > 400) {
					currentRightSpeed = 425;
				}
				if (currentLeftSpeed < 100) {
					currentLeftSpeed = 125;
				}
				
				//Change motor speeds
				WallFollowingLab.leftMotor.setSpeed(currentLeftSpeed);
				WallFollowingLab.rightMotor.setSpeed(currentRightSpeed);
			}
		}
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
