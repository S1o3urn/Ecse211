package ca.mcgill.ecse211.lab1;

/*This class implements the BangBang-type controller for Lab1 on the EV3 platform.
 * Note: this controller assumes the wall is on the left and the us sensor is positioned at a 45degree angle from the left.
 * 
 * @author TianhanJiang (260795887)
 */

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

	/* Constants */
	private static final int FILTER_OUT = 20;

	/* Variables */
	private final int bandCenter; // offset from wall
	private final int bandwidth; // error threshold
	private final int motorLow;
	private final int motorHigh;
	private int distance; // simulated distance
	private int filterControl; // filter threshold
	private int speedIncrease = 150;

	/*
	 * This method sets up variable data and starts driving the robot forward in a
	 * straight line. It takes in the bandCenter, the bandWidth and the high and low
	 * motor speeds.
	 */
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

	/*
	 * (non-Javadoc)
	 * 
	 * @see ca.mcgill.ecse211.lab1.UltrasonicController#processUSData(int)
	 * 
	 * This method filters data from the us sensor so that everything further that
	 * 75 is not considered. This data is then used to determined whether to drive
	 * straight, turn at a fixed speed or in special cases, backup a bit.
	 */
	@Override
	public void processUSData(int distance) {
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)
		// ASSUMPTION: Robot always follows left wall
		// us sensor is positioned at around 45-60 degrees

		// Filter out false negative from us (taken from PController)

		// There is an abnormally large value
		if ((distance >= 75) && (filterControl < FILTER_OUT)) {
			filterControl++;
		}

		// FILTER_OUT amount of repeated large value
		// meaning there could be nothing there to scan
		else if (distance >= 75) {
			this.distance = distance;
		}

		// Filtered values
		else {
			filterControl = 0;
			this.distance = distance;
		}

		// Robot is too close and doesn't have enough space to turn anymore
		// Drive robot backwards turning slightly right to reposition better
		if (distance <= 15) {
			WallFollowingLab.leftMotor.setSpeed(motorLow);
			WallFollowingLab.rightMotor.setSpeed((int) (motorHigh + (speedIncrease*(1.5))));
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.backward();
		}

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
			WallFollowingLab.leftMotor.setSpeed(motorHigh - (speedIncrease / 6));
			WallFollowingLab.rightMotor.setSpeed(motorHigh + (speedIncrease/6));
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}

		// Robot can drive straight
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
