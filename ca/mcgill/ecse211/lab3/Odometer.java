package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {
	
	// Robot positioning and motors
	private double x, y, theta;
	private int leftMotorTachoCount, rightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	// Odometer update period, in ms
	private static final long ODOMETER_PERIOD = 25;

	// Wheel Radius and Wheel Distance
	public static final double WR = EV3Navigation.WHEEL_RADIUS;
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
		long updateStart, updateEnd;
		int currentTachoL, currentTachoR, lastTachoL, lastTachoR;
		double distL, distR, dDistance, dTheta, dX, dY;
		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();
		lastTachoL = leftMotor.getTachoCount();
		lastTachoR = rightMotor.getTachoCount();

		while (true) {
			updateStart = System.currentTimeMillis();

			// Odometer code adapted from lab2 odometer.java and previous years' lab code
			currentTachoL = leftMotor.getTachoCount();
			currentTachoR = rightMotor.getTachoCount();

			distL = 2 * PI * WR * (currentTachoL - lastTachoL) / 360;
			distR = 2 * PI * WR * (currentTachoR - lastTachoR) / 360;

			lastTachoL = currentTachoL; // Save tacho counts for next iteration
			lastTachoR = currentTachoR;

			dDistance = 0.5 * (distL + distR);	// Compute vehicle displacement
			dTheta = (distL - distR) / WD; 		// Compute change in heading
			dX = dDistance * Math.sin(theta); 	// Compute X component of displacement
			dY = dDistance * Math.cos(theta); 	// Compute Y component of displacement
			synchronized (lock) {
				
				/**
				 * Don't use the variables x, y, or theta anywhere but here! Only update the
				 * values of x, y, and theta in this block. Do not perform complex math
				 * 
				 */

				theta += dTheta; 	// Update direction
				x = x + dX; 		// Update estimates of X and Y position
				y = y + dY;
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

	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
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

	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

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