package ca.mcgill.ecse211.lab4;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

import static ca.mcgill.ecse211.lab4.Lab4.*;

public class UltrasonicLocalizer {
	public enum LocalizationType {
		FALLING_EDGE, RISING_EDGE
	};

	// class variables
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private SampleProvider ultrasonicSensor;
	private float[] ultrasonicData;
	private LocalizationType localizationType;
	private int filterControl;
	private float lastDistance;

	public UltrasonicLocalizer(Odometer odometer, SampleProvider ultrasonicSensor, float[] ultrasonicData, LocalizationType localizationType) {
		this.odometer = odometer;
		this.ultrasonicSensor = ultrasonicSensor;
		this.ultrasonicData = ultrasonicData;
		this.localizationType = localizationType;
		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
		leftMotor.setAcceleration(ULTRASONIC_ACCELERATION);
		rightMotor.setAcceleration(ULTRASONIC_ACCELERATION);
		filterControl = 0;
		lastDistance = 100;
	}

	public void doLocalization() {
		double[] position = new double[3];
		double angleA;
		double angleB;

		// set the rotational speed of the motors
		leftMotor.setSpeed(ULTRASONIC_ROTATION_SPEED);
		rightMotor.setSpeed(ULTRASONIC_ROTATION_SPEED);

		if (localizationType == LocalizationType.FALLING_EDGE) {

			// rotate the robot until it sees no wall
			while (getFilteredData() < ULTRASONIC_WALL_DISTANCE + ULTRASONIC_WALL_GAP) {
				leftMotor.forward();
				rightMotor.backward();
			}

			// keep rotating until the robot sees a wall, then latch the angle
			while (getFilteredData() > ULTRASONIC_WALL_DISTANCE) {
				leftMotor.forward();
				rightMotor.backward();
			}
			// get the angle from the odometer
			angleA = odometer.getAngle();

			// switch direction and wait until it sees no wall
			while (getFilteredData() < ULTRASONIC_WALL_DISTANCE + ULTRASONIC_WALL_GAP) {
				leftMotor.backward();
				rightMotor.forward();
			}
			// Sound.beep();

			// keep rotating until the robot sees a wall, then latch the angle
			while (getFilteredData() > ULTRASONIC_WALL_DISTANCE) {
				leftMotor.backward();
				rightMotor.forward();
			}
			rightMotor.stop(true);
			leftMotor.stop(true);
			// get the angle from the odometer
			angleB = odometer.getAngle();
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
			odometer.setPosition(new double[] { 0.0, 0.0, 0 }, new boolean[] { true, true, true });
		} else {
			/*
			 * The robot should turn until it sees the wall, then look for the
			 * "rising edges:" the points where it no longer sees the wall. This is very
			 * similar to the FALLING_EDGE routine, but the robot will face toward the wall
			 * for most of it.
			 */
			// rotate the robot until it sees a wall
			while (getFilteredData() > ULTRASONIC_WALL_DISTANCE - ULTRASONIC_WALL_GAP) {
				leftMotor.backward();
				rightMotor.forward();
			}
			// keep rotating until the robot no longer sees the wall, then latch the angle
			while (getFilteredData() < ULTRASONIC_WALL_DISTANCE) {
				leftMotor.backward();
				rightMotor.forward();
			}

			angleA = odometer.getAngle();

			// switch directions and rotate until the robot sees the wall.
			while (getFilteredData() > ULTRASONIC_WALL_DISTANCE - ULTRASONIC_WALL_GAP) {
				leftMotor.forward();
				rightMotor.backward();
			}

			// rotate until the robot no longer sees the wall and latch the angle.
			while (getFilteredData() < ULTRASONIC_WALL_DISTANCE) {
				leftMotor.forward();
				rightMotor.backward();
			}
			leftMotor.stop(true);
			rightMotor.stop(true);
			angleB = odometer.getAngle();

			// if our angle A is bigger than B, subtract 360.
			if (angleA > angleB) {
				angleA = angleA - 360;
			}
			// calculate the average angle and the zero point (zeropoint is x axis)
			double averageAngle = (angleA + angleB) / 2;
			double ZeroPoint = angleB - averageAngle + 45;

			// rotate to the diagonal + 45 (to the x axis).
			leftMotor.rotate(convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, ZeroPoint), true);
			rightMotor.rotate(-convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, ZeroPoint), false);

			// update the odometer position to 0 0 0. The x and y will be wrong
			// but that will be fixed by the LightLocalizer
			odometer.setPosition(new double[] { 0.0, 0.0, 0 }, new boolean[] { true, true, true });
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
		ultrasonicSensor.fetchSample(ultrasonicData, 0);
		float distance = (int) (ultrasonicData[0] * 100.0);
		float result = 0;
		if (distance > 50 && filterControl < ULTRASONIC_FILTER_OUT) {
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