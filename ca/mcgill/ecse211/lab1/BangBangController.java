package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

	/* Constants */
	private static final int FILTER_OUT = 20;

	private final int bandCenter; // offset from wall
	private final int bandwidth; // error threshold
	private final int motorLow;
	private final int motorHigh;
	private int distance; // simulated distance
	private int filterControl; // filter threshold
	private boolean hasReversed;
	private int reverseCounter;
	private int speedIncrease = 200;

	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = (int) (bandCenter * 1.25);
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;

		this.reverseCounter = 0;

		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)

		// Filter out false negative from us (taken from PController)

		// There is an abnormally large value
		if ((distance >= 60) && (filterControl < FILTER_OUT)) {
			filterControl++;
		}

		// FILTER_OUT amount of repeated large value
		// meaning there could be nothing there to scan
		// No changes to distance
		else if (distance >= 255) {
			this.distance = distance;
		}

		// Filtered values
		else {
			filterControl = 0;
			// If hasReversed in the last 10 cycles,
			// halve the distance to compensate for large sensor angle with wall
			if (hasReversed && (reverseCounter < 10)) {
				reverseCounter++;
				this.distance = distance / 2;
			} else {
				hasReversed = false;
				this.distance = distance;
			}
		}

		// Robot too close to wall and needs a sharp turn to avoid collision
		if (distance < 10) {
			reverseCounter = 0;
			hasReversed = true;
			WallFollowingLab.rightMotor.setSpeed(motorHigh + speedIncrease);
			WallFollowingLab.leftMotor.setSpeed(motorLow);
			WallFollowingLab.rightMotor.backward();
			WallFollowingLab.leftMotor.forward();
		}

		// Robot outside of offset from wall and needs to accelerate outside wheel
		else if (distance > bandCenter + bandwidth) {
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.setSpeed(motorLow);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}

		// Robot inside of off from wall and needs to accelerate inside wheel
		else if (distance < bandCenter - bandwidth) {
			WallFollowingLab.rightMotor.setSpeed(motorLow);
			WallFollowingLab.leftMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}

		// Robot is within bandCenter and should keep both wheels at same speed
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();

	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
