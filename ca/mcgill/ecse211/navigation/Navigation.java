package ca.mcgill.ecse211.navigation;

/**
 * This class implements the navigation logic.
 * It provides methods to travel to a waypoint, turn a specific angle and check whether the robot is navigating.
 * 
 * @author tianh
 */
import ca.mcgill.ecse211.lab4.Lab4;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation extends Thread {

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private double deltax;
	private double deltay;

	// Current location of the vehicle
	private double x;
	private double y;
	private double theta;

	// Navigation constants
	private static final int FORWARD_SPEED = 180;
	private static final int ROTATE_SPEED = 100;
	private static final double TILE_SIZE = 30.48;

	private boolean navigating = true;

	// Constructor for navigation
	public Navigation(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odometer = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}

	/**
	 * Travel to a designated coordinate.
	 * @param x
	 * @param y
	 * 
	 * Taken from lab 3.
	 */
	public void travelTo(double x, double y) {

		x = odometer.getXYT()[0];
		y = odometer.getXYT()[1];

		deltax = x - x;
		deltay = y - y;

		// Calculate the angle to turn
		theta = (odometer.getXYT()[2]) * Math.PI / 180;
		double mTheta = Math.atan2(deltax, deltay) - theta;

		double hypotenuse = Math.hypot(deltax, deltay);

		// Turn to the correct angle towards the endpoint
		turnTo(mTheta);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(Lab4.WHEEL_RAD, hypotenuse), true);
		rightMotor.rotate(convertDistance(Lab4.WHEEL_RAD, hypotenuse), false);

		// stop vehicle
		leftMotor.stop(true);
		rightMotor.stop(true);
	}

	/**
	 * Turns the robot by a set theta.
	 * 
	 * @param theta
	 * 
	 * Taken from lab 3.
	 */
	public void turnTo(double theta) {

		// Minimum angle
		if (theta > Math.PI) {
			theta -= 2 * Math.PI;
		} else if (theta < -Math.PI) {
			theta += 2 * Math.PI;
		}

		// Set Speed
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		// Left turn for negative angle
		if (theta < 0) {
			leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, -(theta * 180) / Math.PI), true);
			rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, -(theta * 180) / Math.PI), false);

		} else {
			// Right turn for positive angle
			leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, (theta * 180) / Math.PI), true);
			rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, (theta * 180) / Math.PI), false);
		}
	}

	/**
	 * Check to see if robot is traveliing to, or turning to.
	 * @return
	 * @throws OdometerExceptions
	 * 
	 * Taken from lab 3.
	 */
	boolean isNavigating() throws OdometerExceptions {
		return navigating;
	}

	/**
	 * This method takes in the total distance needed to travel and transforms it
	 * into the number of wheel rotations needed
	 * 
	 * @param distance
	 * @return distance in wheel rotations.
	 * 
	 * Taken from lab 3.
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method converts radians into degrees.
	 * 
	 * @param angle
	 * @return wheel rotations needed
	 * 
	 * Taken from lab 3.
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}