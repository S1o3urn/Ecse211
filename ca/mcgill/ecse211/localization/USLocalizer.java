package ca.mcgill.ecse211.localization;

/**
 * This class implements the US localizer logic.
 * 
 * @author tianh
 */
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.lab4.*;

public class USLocalizer {

	// Vehicle constants
	public static int ROTATION_SPEED = 100;
	private double deltaTheta;

	private Odometer odometer;
	private float[] usData;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private boolean isRisingEdge;
	private SampleProvider usDistance;

	public Navigation navigation;

	/**
	 * Wall distance
	 */
	private double d = 35.00;
	
	/**
	 * Gap distance
	 */
	private double k = 2;

	/**
	 * The constructor.
	 * 
	 * @param Odometer
	 * @param EV3LargeRegulatedMotor
	 * @param EV3LargeRegulatedMotor
	 * @param boolean
	 * @param SampleProvider
	 */
	public USLocalizer(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			boolean localizationType, SampleProvider usDistance) {
		this.odometer = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.isRisingEdge = localizationType;
		this.usDistance = usDistance;
		this.usData = new float[usDistance.sampleSize()];
		navigation = new Navigation(odometer, leftMotor, rightMotor);

		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);
	}

	/**
	 * This method runs the appropriate localizer method.
	 */
	public void localize() {
		if (isRisingEdge) {
			localizeRisingEdge();
		} else {
			localizeFallingEdge();
		}
	}

	/**
	 * This method implements the risingEdge localization technique.
	 */
	public void localizeRisingEdge() {

		double angleA;
		double angleB;
		double turningAngle;

		// Rotate left to wall
		while (fetchUSData() > d) {
			leftMotor.backward();
			rightMotor.forward();
		}
		// Rotate until open space detected
		while (fetchUSData() < d + k) {
			leftMotor.backward();
			rightMotor.forward();
		}
		Sound.beep();
		// Record angle
		angleA = odometer.getXYT()[2];

		// Rotate until wall spotted
		while (fetchUSData() > d) {
			leftMotor.forward();
			rightMotor.backward();
		}

		// Rotate until open space detected
		while (fetchUSData() < d + k) {
			leftMotor.forward();
			rightMotor.backward();
		}
		Sound.beep();
		angleB = odometer.getXYT()[2];

		leftMotor.stop(true);
		rightMotor.stop();

		// Calculate rotation angle
		if (angleA < angleB) {
			deltaTheta = 45 - (angleA + angleB) / 2 + 180;
		} else if (angleA > angleB) {
			deltaTheta = 225 - (angleA + angleB) / 2 + 180;
		}

		turningAngle = deltaTheta + odometer.getXYT()[2];

		// Rotate robot to 0 theta
		// 4.5 fixes physical errors
		leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, turningAngle + 4.5), true);
		rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, turningAngle + 4.5), false);

		odometer.setXYT(0.0, 0.0, 0.0);
	}

	/**
	 * This method implements the risingEdge localization technique.
	 * inverse wall detection logic from RisingEdge
	 */
	public void localizeFallingEdge() {

		double angleA;
		double angleB;
		double turningAngle;

		// Rotate until open space detected
		while (fetchUSData() < d + k) {
			leftMotor.backward();
			rightMotor.forward();
		}
		// Rotate left to wall
		while (fetchUSData() > d) {
			leftMotor.backward();
			rightMotor.forward();
		}
		Sound.beep();
		
		// Record angle
		angleA = odometer.getXYT()[2];

		// Rotate right until no walls detected
		while (fetchUSData() < d + k) {
			leftMotor.forward();
			rightMotor.backward();
		}

		// Rotate right until wall detected
		while (fetchUSData() > d) {
			leftMotor.forward();
			rightMotor.backward();
		}
		Sound.beep();
		
		angleB = odometer.getXYT()[2];

		leftMotor.stop(true);
		rightMotor.stop();

		// Calculate rotation angle
		if (angleA < angleB) {
			deltaTheta = 45 - (angleA + angleB) / 2;

		} else if (angleA > angleB) {
			deltaTheta = 225 - (angleA + angleB) / 2;
		}

		turningAngle = deltaTheta + odometer.getXYT()[2];

		// Rotate robot to 0 theta
		// 4.5 fixes physical errors
		leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, turningAngle - 1), true);
		rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, turningAngle - 1), false);

		odometer.setXYT(0.0, 0.0, 0.0);

	}

	/**
	 * This method fetches the distance measured by the ultrasonic sensor.
	 * @return transformed distance value
	 */
	private int fetchUSData() {
		usDistance.fetchSample(usData, 0);
		return (int) (usData[0] * 100);
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