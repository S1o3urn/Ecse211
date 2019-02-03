package ca.mcgill.ecse211.lab3;

/*
 * This class implements the odometer functionality.
 * Given the wheel radius, the speed at which is travels and the tachometer count, 
 * it is then possible to calculate the distance traveled
 * in both an x and y axis defined at program launch time.
 *  
 * We chose to model this class after previous year odometer classes 
 * since it utilises 1 class rather than both an odometer and an odometerData class
 */

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {

	// Robot positioning
	private double x;
	private double y;
	private double theta;

	// Motors
	private int leftMotorTachoCount;
	private int rightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	// Odometer update period, in ms
	private static final long ODOMETER_PERIOD = 25;

	// Wheel Radius and Wheel Distance
	public static final double WR = EV3Navigation.WHEEL_RADIUS; // Same values as lab2
	public static final double WD = EV3Navigation.WHEEL_BASE;
	public static final double PI = Math.PI;

	// Lock object for mutual exclusion
	private Object lock;

	// Default constructor
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.x = 0.0;
		this.y = 0.0;
		this.theta = 0.0;
		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;
		lock = new Object();
	}

	// Run method (required for Thread)
	public void run() {

		// Time tracking
		long updateStart;
		long updateEnd;

		// Tachometer count tracking
		int leftMotorTachoCount;
		int rightMotorTachoCount;
		int lastLeftMotorTachoCount;
		int lastRightMotorTachoCount;

		// Calculated measures
		double leftDistance;
		double rightDistance;
		double deltaDistance;
		double deltaTheta;
		double deltaX;
		double deltaY;

		// Reset and initialize tachometer count to 0
		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();
		lastLeftMotorTachoCount = leftMotor.getTachoCount();
		lastRightMotorTachoCount = rightMotor.getTachoCount();

		while (true) {
			updateStart = System.currentTimeMillis();

			// Odometer code adapted from lab2 odometer.java and previous years' lab code
			leftMotorTachoCount = leftMotor.getTachoCount();
			rightMotorTachoCount = rightMotor.getTachoCount();

			// Calculate distance traveled for each wheel in metric units
			leftDistance = PI * WR * (leftMotorTachoCount - lastLeftMotorTachoCount) / 180;
			rightDistance = PI * WR * (rightMotorTachoCount - lastRightMotorTachoCount) / 180;

			// Save current tacho count to check with next cycle
			lastLeftMotorTachoCount = leftMotorTachoCount;
			lastRightMotorTachoCount = rightMotorTachoCount;

			// Compute distances in x and y axis
			deltaDistance = 0.5 * (leftDistance + rightDistance);
			deltaTheta = (leftDistance - rightDistance) / WD;
			deltaX = deltaDistance * Math.sin(theta);
			deltaY = deltaDistance * Math.cos(theta);
			synchronized (lock) {

				/**
				 * Don't use the variables x, y, or theta anywhere but here! Only update the
				 * values of x, y, and theta in this block. Do not perform complex math
				 * 
				 */

				theta += deltaTheta; // Update direction
				x = x + deltaX; // Update estimates of X and Y position
				y = y + deltaY;
			}

			// This ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// There is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	// Getters

	/**
	 * This method returns the x,y and theta coordinates of the robot
	 * 
	 * @param position
	 *            array, and update boolean
	 * @return x, y, theta
	 */
	public void getPosition(double[] position, boolean[] update) {
		// Ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
	}

	/**
	 * This method retrieve the X value
	 * 
	 * @return value
	 */
	public double getX() {
		double value;

		synchronized (lock) {
			value = x;
		}

		return value;
	}

	/**
	 * This method retrieve the Y value
	 * 
	 * @return value
	 */
	public double getY() {
		double value;

		synchronized (lock) {
			value = y;
		}

		return value;
	}

	/**
	 * This method retrieve the theta value
	 * 
	 * @return value
	 */
	public double getTheta() {
		double value;

		synchronized (lock) {
			value = theta;
		}

		return value;
	}

	/**
	 * @return the rightMotorTachoCount
	 */
	public int getRightMotorTachoCount() {
		return rightMotorTachoCount;
	}

	/**
	 * @return the leftMotorTachoCount
	 */
	public int getLeftMotorTachoCount() {
		return leftMotorTachoCount;
	}

	// Setters

	/**
	 * This method updates the x, y, and theta positions
	 * 
	 * @param position
	 * @param update
	 */
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	/**
	 * @param x
	 */
	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	/**
	 * @param y
	 */
	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	/**
	 * @param theta
	 */
	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}

	/**
	 * @param leftMotorTachoCount
	 */
	public void setLeftMotorTachoCount(int leftMotorTachoCount) {
		synchronized (lock) {
			this.leftMotorTachoCount = leftMotorTachoCount;
		}
	}

	/**
	 * @param rightMotorTachoCount
	 */
	public void setRightMotorTachoCount(int rightMotorTachoCount) {
		synchronized (lock) {
			this.rightMotorTachoCount = rightMotorTachoCount;
		}
	}
}