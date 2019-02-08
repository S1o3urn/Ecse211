package ca.mcgill.ecse211.lab4;

import lejos.utility.Timer;
import lejos.utility.TimerListener;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

import static ca.mcgill.ecse211.lab4.Lab4.*;

public class Odometer implements TimerListener {

	private Timer timer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private double width;
	private double x;
	private double y;
	private double theta;
	private double[] lastDisplacementAndHeading;
	private double[] deltaDisplacementAndHeading;
	private final int DEFAULT_TIMEOUT_PERIOD = 20;

/**
 * This constructor.
 * @param leftMotor
 * @param rightMotor
 * @param INTERVAL
 * @param autostart
 */
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, int INTERVAL,
			boolean autostart) {

		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.x = 0.0;
		this.y = 0.0;
		this.theta = 0;
		this.lastDisplacementAndHeading = new double[2];
		this.deltaDisplacementAndHeading = new double[2];

		if (autostart) {
			// if the timeout interval is given as <= 0, default to 20ms timeout
			this.timer = new Timer((INTERVAL <= 0) ? INTERVAL : DEFAULT_TIMEOUT_PERIOD, this);
			this.timer.start();
		} else {
			this.timer = null;
		}
	}

	/**
	 * This method stops the timer.
	 */
	public void stop() {
		if (this.timer != null)
			this.timer.stop();
	}

	/**
	 * This method starts the timer.
	 */
	public void start() {
		if (this.timer != null)
			this.timer.start();
	}

	/**
	 * This method calculates the robots displacement and orientation.
	 * @param data
	 */
	private void getDisplacementAndHeading(double[] data) {
		int leftTacho;
		int rightTacho;
		leftTacho = leftMotor.getTachoCount();
		rightTacho = rightMotor.getTachoCount();

		data[0] = (leftTacho + rightTacho) * WHEEL_RADIUS * Math.PI / 360.0;
		data[1] = (rightTacho - leftTacho) * WHEEL_RADIUS / width;
	}

	/**
	 * This method corrects the displacement and orientation based on odmeter's values.
	 */
	public void timedOut() {
		this.getDisplacementAndHeading(deltaDisplacementAndHeading);
		deltaDisplacementAndHeading[0] -= lastDisplacementAndHeading[0];
		deltaDisplacementAndHeading[1] -= lastDisplacementAndHeading[1];

		// Update to position
		synchronized (this) {
			theta += deltaDisplacementAndHeading[1];
			theta = correctAngle(theta);

			x += deltaDisplacementAndHeading[0] * Math.cos(Math.toRadians(theta));
			y += deltaDisplacementAndHeading[0] * Math.sin(Math.toRadians(theta));
		}

		lastDisplacementAndHeading[0] += deltaDisplacementAndHeading[0];
		lastDisplacementAndHeading[1] += deltaDisplacementAndHeading[1];
	}

	// Accessors
	
	/**
	 * This method gets the x value of the robot's current position.
	 * @return X
	 */
	public double getX() {
		synchronized (this) {
			return x;
		}
	}

	/**
	 * This method gets the y value of the robot's current position.
	 * @return Y
	 */
	public double getY() {
		synchronized (this) {
			return y;
		}
	}

	/**
	 * This method gets the theta value of the robot's current position.
	 * @return theta
	 */
	public double getAng() {
		synchronized (this) {
			return theta;
		}
	}

	/**
	 * This method gets the robot's position.
	 * @param position
	 */
	public void getPosition(double[] position) {
		synchronized (this) {
			position[0] = x;
			position[1] = y;
			position[2] = theta;
		}
	}

	/**
	 * This method gets the robot's position.
	 * Overloaded method.
	 * @return a new array with x, y and theta
	 */
	public double[] getPosition() {
		synchronized (this) {
			return new double[] { x, y, theta };
		}
	}

	/**
	 * This method returns an array of both motors.
	 * @return a new array with both motors
	 */
	public EV3LargeRegulatedMotor[] getMotors() {
		return new EV3LargeRegulatedMotor[] { this.leftMotor, this.rightMotor };
	}

	/**
	 * This method returns the left motor.
	 * @return leftMotor
	 */
	public EV3LargeRegulatedMotor getLeftMotor() {
		return this.leftMotor;
	}

	/**
	 * This method returns the right motor.
	 * @return rightMotor
	 */
	public EV3LargeRegulatedMotor getRightMotor() {
		return this.rightMotor;
	}

	// Mutators
	
	/**
	 * This method changes the robot's perceived position.
	 * @param position
	 * @param update
	 */
	public void setPosition(double[] position, boolean[] update) {
		synchronized (this) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	/**
	 * This method corrects angle issues with 0 = 360 degrees.
	 * @param angle
	 * @return angle
	 */
	public static double correctAngle(double angle) {
		if (angle < 0.0)
			angle = 360.0 + (angle % 360.0);

		return angle % 360.0;
	}

	/**
	 * This method finds the minimum angle to turn.
	 * @param a
	 * @param b
	 * @return minimum angle
	 */
	public static double minimumAngleFromTo(double a, double b) {
		double d = correctAngle(b - a);

		if (d < 180.0)
			return d;
		else
			return d - 360.0;
	}
}