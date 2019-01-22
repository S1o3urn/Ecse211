package ca.mcgill.ecse211.lab1;

/*This class implements the p-type controller for Lab1 on the EV3 platform.
 * Note: this controller assumes wall is on the left and the us sensor is positioned at a 45degree angle from the left.
 * 
 * @author TianhanJiang (260795887)
 */
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 300;
	private static final int FILTER_OUT = 20;

	/* Variables */
	private final int bandCenter;
	private final int bandWidth;
	private int distance;
	private int currentLeftSpeed;
	private int currentRightSpeed;
	private int filterControl;

	/*
	 * This method sets up variable data and starts driving the robot forward in a
	 * straight line. It takes in the bandCenterand bandwidth
	 */
	public PController(int bandCenter, int bandwidth) {
		this.bandCenter = bandCenter;
		this.bandWidth = bandwidth;
		this.filterControl = 0;

		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initialize motor rolling forward
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);

		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();

		currentLeftSpeed = MOTOR_SPEED;
		currentRightSpeed = MOTOR_SPEED;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see ca.mcgill.ecse211.lab1.UltrasonicController#processUSData(int)
	 * 
	 * This method filters data from the us sensor so that everything further that
	 * 75 is not considered. This data is then used to calculate whether to turn or
	 * not and by how much.
	 */
	@Override
	public void processUSData(int distance) {

		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).

		// There is an abnormally large value
		if ((distance == 255) && (filterControl < FILTER_OUT)) {
			filterControl++;
		}

		// FILTER_OUT amount of repeated large value
		// meaning there could be nothing there to scan
		else if (distance == 255) {
			this.distance = distance;
			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + 150);
			WallFollowingLab.rightMotor.setSpeed(300);
		}

		// Filtered values
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

			// Robot is on right path, advance straight
			if ((this.distance >= (bandCenter - bandWidth)) && (this.distance <= (bandCenter + bandWidth))) {
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
			}

			// Robot is too close to wall
			else if ((this.distance < (bandCenter - bandWidth))) {
				// Need to turn right
				// Increase left motor speed proportionally to error
				// Decrease right motor speed proportionally to error
				currentLeftSpeed = (int) (0.75 * (MOTOR_SPEED * error) / bandWidth);
				currentRightSpeed = (MOTOR_SPEED * bandWidth) / error;

				// Set a speed limit to avoid overworking motors
				if (currentLeftSpeed > 400) {
					currentLeftSpeed = 425;
				}

				if (currentRightSpeed < 100) {
					currentRightSpeed = 125;
				}

				// Change motor speeds
				WallFollowingLab.leftMotor.setSpeed(currentLeftSpeed);
				WallFollowingLab.rightMotor.setSpeed(currentRightSpeed);
			}

			else if (this.distance > (bandCenter - bandWidth)) {
				// Need to turn left
				// Decrease left motor speed proportionally to error
				// Increase right motor speed proportionally to error
				currentRightSpeed = (MOTOR_SPEED * error) / bandWidth;
				currentLeftSpeed = (MOTOR_SPEED * bandWidth) / error;

				// we don't want the motor to run too fast, so we add a min and max speed
				if (currentRightSpeed > 400) {
					currentRightSpeed = 425;
				}
				if (currentLeftSpeed < 100) {
					currentLeftSpeed = 125;
				}

				// Change motor speeds
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
