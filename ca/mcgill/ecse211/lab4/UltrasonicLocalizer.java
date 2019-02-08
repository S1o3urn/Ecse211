package ca.mcgill.ecse211.lab4;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer {
	public enum LocalizationType {
		FALLING_EDGE, RISING_EDGE
	};

	// constants
	public static int ROTATION_SPEED = 60;
	public static int ACCELERATION = 600;
	// class variables
	private Odometer odo;
	private SampleProvider usSensor;
	private float[] usData;
	private LocalizationType locType;
	private static int WALL_DIST = 30;
	private static int WALL_GAP = 3;
	private static int FILTER_OUT = 3;
	private int filterControl;
	private float lastDistance;
	// Motors (we will get these from the odometer)
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	public UltrasonicLocalizer(Odometer odo, SampleProvider usSensor, float[] usData, LocalizationType locType) {
		// get incoming values
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.locType = locType;
		// get the motors from the odometer object.
		EV3LargeRegulatedMotor[] motors = this.odo.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
		leftMotor.setAcceleration(ACCELERATION);
		rightMotor.setAcceleration(ACCELERATION);
		filterControl = 0;
		lastDistance = 100;
	}

	public void doLocalization() {
		double[] pos = new double[3];
		double angleA, angleB;

		// set the rotational speed of the motors
		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);

		if (locType == LocalizationType.FALLING_EDGE) {

			// rotate the robot until it sees no wall
			while (getFilteredData() < WALL_DIST + WALL_GAP) {
				leftMotor.forward();
				rightMotor.backward();
			}

			// keep rotating until the robot sees a wall, then latch the angle
			while (getFilteredData() > WALL_DIST) {
				leftMotor.forward();
				rightMotor.backward();
			}
			// get the angle from the odometer
			angleA = odo.getAng();

			// switch direction and wait until it sees no wall
			while (getFilteredData() < WALL_DIST + WALL_GAP) {
				leftMotor.backward();
				rightMotor.forward();
			}
			// Sound.beep();

			// keep rotating until the robot sees a wall, then latch the angle
			while (getFilteredData() > WALL_DIST) {
				leftMotor.backward();
				rightMotor.forward();
			}
			rightMotor.stop(true);
			leftMotor.stop(true);
			// get the angle from the odometer
			angleB = odo.getAng();
			// if our angle A is larger than B, it means we passed the 0 point, and that
			// angle A is "negative".
			if (angleA > angleB) {
				angleA = angleA - 360;
			}
			// angleA is clockwise from angleB, so assume the average of the
			// angles to the right of angleB is 45 degrees past 'north'
			double averageAngle = (angleA + angleB) / 2;
			double ZeroPoint = angleB - averageAngle + 45;

			// System.out.println("A" + angleA);
			// System.out.println("B:" + angleB);
			// System.out.println("Average" + averageAngle);
			// System.out.println("To Turn" + (FortyFiveDegPastNorth + 45));
			// rotate to the diagonal + 45 (to the horizontal x axis)
			leftMotor.rotate(convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, ZeroPoint), true);
			rightMotor.rotate(-convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, ZeroPoint), false);

			// update the odometer position to 0 0 0 (that's how we are facing. Position (x
			// and y) will
			// be wrong but that will be fixed by the LightLocalizer
			odo.setPosition(new double[] { 0.0, 0.0, 0 }, new boolean[] { true, true, true });
		} else {
			/*
			 * The robot should turn until it sees the wall, then look for the
			 * "rising edges:" the points where it no longer sees the wall. This is very
			 * similar to the FALLING_EDGE routine, but the robot will face toward the wall
			 * for most of it.
			 */
			// rotate the robot until it sees a wall
			while (getFilteredData() > WALL_DIST - WALL_GAP) {
				leftMotor.backward();
				rightMotor.forward();
			}
			// keep rotating until the robot no longer sees the wall, then latch the angle
			while (getFilteredData() < WALL_DIST) {
				leftMotor.backward();
				rightMotor.forward();
			}

			angleA = odo.getAng();

			// switch directions and rotate until the robot sees the wall.
			while (getFilteredData() > WALL_DIST - WALL_GAP) {
				leftMotor.forward();
				rightMotor.backward();
			}

			// rotate until the robot no longer sees the wall and latch the angle.
			while (getFilteredData() < WALL_DIST) {
				leftMotor.forward();
				rightMotor.backward();
			}
			leftMotor.stop(true);
			rightMotor.stop(true);
			angleB = odo.getAng();

			// if our angle A is bigger than B, subtract 360.
			if (angleA > angleB) {
				angleA = angleA - 360;
			}
			// calculate the average angle andd the zero point (zeropoint is x axis)
			double averageAngle = (angleA + angleB) / 2;
			double ZeroPoint = angleB - averageAngle + 45;

			// angleA is clockwise from angleB, so assume the average of the
			// angles to the right of angleB is 45 degrees past 'north'
			// double averageAngle = (angleA + angleB)/2;
			// double FortyFiveDegPastNorth = angleB - averageAngle;
			// Sound.beep();
			// rotate to the diagonal + 45 (to the x axis).
			leftMotor.rotate(convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, ZeroPoint), true);
			rightMotor.rotate(-convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, ZeroPoint), false);

			// update the odometer position to 0 0 0. The x and y will be wrong
			// but that will be fixed by the LightLocalizer
			odo.setPosition(new double[] { 0.0, 0.0, 0 }, new boolean[] { true, true, true });
		}
	}

	// Conversion methods.
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	private float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		float distance = (int) (usData[0] * 100.0);
		float result = 0;
		if (distance > 50 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the filter value
			filterControl++;
			result = lastDistance;
		} else if (distance > 50) {
			// true 255, therefore set distance to 255
			result = 50; // clips it at 50
		} else {
			// distance went below 255, therefore reset everything.
			filterControl = 0;
			result = distance;
		}
		lastDistance = distance;
		return result;
	}

}