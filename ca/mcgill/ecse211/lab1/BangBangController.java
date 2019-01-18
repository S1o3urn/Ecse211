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
	private int speedIncrease = 100;

	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;

		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)
		// ASSUMPTION: Robot always follows left wall

		// Filter out false negative from us (taken from PController)

		// There is an abnormally large value
		if ((distance >= 75) && (filterControl < FILTER_OUT)) {
			filterControl++;
		}

		// FILTER_OUT amount of repeated large value
		// meaning there could be nothing there to scan
		else if (distance >= 255) {
			this.distance = distance;
		}

		// Filtered values
		else {
			filterControl = 0;
			this.distance = distance;
		}

		// Robot is too close and doesn't have enough space to turn anymore
		if (distance <= 25) {
			WallFollowingLab.leftMotor.setSpeed(motorHigh);
			WallFollowingLab.rightMotor.setSpeed(motorHigh + speedIncrease);
			WallFollowingLab.leftMotor.backward();
			WallFollowingLab.rightMotor.backward();
		}

		// Robot is at offset and within the appropriate bandwidth
		// Robot will go straight
		/*
		 * else if((distance >= (bandCenter - bandwidth)) && (distance <= (bandCenter +
		 * bandwidth))) { WallFollowingLab.leftMotor.setSpeed(motorHigh +
		 * speedIncrease); WallFollowingLab.rightMotor.setSpeed(motorHigh +
		 * speedIncrease); WallFollowingLab.leftMotor.forward();
		 * WallFollowingLab.rightMotor.forward(); }
		 */

		// Robot is on the inside of the offset and over the bandwidth
		// Robot will steer right
		else if (distance < (bandCenter - bandwidth)) {
			WallFollowingLab.leftMotor.setSpeed(motorHigh + speedIncrease);
			WallFollowingLab.rightMotor.setSpeed(motorHigh - (speedIncrease / 2));
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}

		// Robot is on the outside of the offset and over the bandwidth
		// Robot will steer left
		else if ((distance > (bandCenter + bandwidth))) {
			WallFollowingLab.leftMotor.setSpeed(motorHigh - (speedIncrease / 2));
			WallFollowingLab.rightMotor.setSpeed(motorHigh + speedIncrease);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}

		else {
			WallFollowingLab.leftMotor.setSpeed(motorHigh + speedIncrease);
			WallFollowingLab.rightMotor.setSpeed(motorHigh + speedIncrease);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
