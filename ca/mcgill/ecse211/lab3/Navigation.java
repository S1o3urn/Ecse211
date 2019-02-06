package ca.mcgill.ecse211.lab3;

/**
 * This method implements the normal navigation through waypoints 
 * @author Tian Han Jiang
 */

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This imports existing motor instances.
 */
import static ca.mcgill.ecse211.lab3.EV3Navigation.*;

public class Navigation extends Thread {

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final double PI = Math.PI;
	
	private static boolean navigating = false;

	/**
	 * Constructor
	 * 
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 */
	public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}

	@Override
	public void run() {

		// Input travel points here
		travelTo((1 * TILE_MEASURE), (1 * TILE_MEASURE));
		travelTo((0 * TILE_MEASURE), (2 * TILE_MEASURE));
		travelTo((2 * TILE_MEASURE), (2 * TILE_MEASURE));
		travelTo((2 * TILE_MEASURE), (1 * TILE_MEASURE));
		travelTo((1 * TILE_MEASURE), (0 * TILE_MEASURE));
	}

	/**
	 * This method calculates the distances in x and y axis that the robot needs to
	 * travel through trigonometry and then finds the hypotenuse in order to travel
	 * in a straight line
	 * 
	 * @param x
	 * @param y
	 */
	public void travelTo(double x, double y) {

		// Path calculation variables
		double xPath;
		double yPath;
		double path;
		double angle;

		// Reset motor speeds
		leftMotor.stop();
		rightMotor.stop();
		leftMotor.setAcceleration(3000);
		rightMotor.setAcceleration(3000);

		navigating = true;

		// Calculate path and angle
		xPath = x - odometer.getX();
		yPath = y - odometer.getY();
		path = Math.hypot(xPath, yPath);
		angle = Math.atan2(xPath, yPath);

		// Turn to face the waypoint
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		turnTo(angle);

		// Advance forward equal to path distance
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(distanceToRotations(path), true);
		rightMotor.rotate(distanceToRotations(path), false);
	}

/**
 * This method implements the logic behind turning to face a waypoint
 * 
 * @param theta
 */
	public void turnTo(double theta) {
		double angle = getMinAngle(theta - odometer.getTheta());

		//double angle = theta - odometer.getTheta();

		leftMotor.rotate(radianToDegree(angle), true);
		rightMotor.rotate(-radianToDegree(angle), false);
	}

	public double getMinAngle(double angle) {
		if (angle > PI) {
			angle = angle - 2 * PI;
		} else if (angle < -PI) {
			angle = 2 * PI + angle;
		}
		return angle;
	}

	/**
	 * This method takes in the total distance needed to travel and transforms it
	 * into the number of wheel rotations needed
	 * 
	 * @param distance
	 * @return
	 */
	public int distanceToRotations(double distance) {
		return (int) (180 * distance / (PI * WHEEL_RADIUS));
	}

	/**
	 * This method converts radians into degrees
	 * @param angle
	 * @return wheel rotations needed
	 */
	public int radianToDegree(double angle) {
		return distanceToRotations(WHEEL_BASE * angle / 2);
	}
	
	/**
	 * Checks to see if the robot is on the move or not
	 * 
	 * @return true/false
	 */
	public boolean isNavigating() {
		return navigating;
	}

}