package ca.mcgill.ecse211.localization;

import lejos.hardware.Sound;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.localization.*;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.lab4.*;



public class LightLocalizer {

	// Vehicle constants
	public static int ROTATION_SPEED = 100;
	private double LIGHT_SENSOR_LENGTH = 11.8;

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	public Navigation navigation;
	
	// Instantiate color sensor
	private static final EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	private float sample;

	private SensorMode idColour;

	double[] lineMeasures;

	public LightLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {

		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		idColour = lightSensor.getRedMode();
		lineMeasures = new double[4];
		navigation = new Navigation(odometer, leftMotor, rightMotor);
	}

	/**
	 * This method uses the light sensor to move to the appropriate coordinate.
	 */
	public void localize() {

		// Rotating index
		int index = 0;
		
		// Set to rotating speed
		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);

		// Verify close to origin before rotating
		toOrigin();

		// Scan all four lines
		while (index < 4) {
			
			// Turning right
			leftMotor.forward();
			rightMotor.backward();

			sample = fetchSample();

			if (sample < 0.38) {
				lineMeasures[index] = odometer.getXYT()[2];
				Sound.beepSequenceUp();
				index++;
			}
		}

		leftMotor.stop(true);
		rightMotor.stop();
		
		double deltaX;
		double deltaY;
		double thetaX;
		double thetaY;

		// Calculate location from 0 with calculated angles
		thetaY = lineMeasures[3] - lineMeasures[1];
		thetaX = lineMeasures[2] - lineMeasures[0];

		deltaX = -1 * LIGHT_SENSOR_LENGTH * Math.cos(Math.toRadians(thetaY / 2));
		deltaY = -1 * LIGHT_SENSOR_LENGTH * Math.cos(Math.toRadians(thetaX / 2));

		// Correct position by moving to origin
		odometer.setXYT(deltaX, deltaY, odometer.getXYT()[2]-6);
		navigation.travelTo(0.0, 0.0);

		leftMotor.setSpeed(ROTATION_SPEED / 2);
		rightMotor.setSpeed(ROTATION_SPEED / 2);

		// Face 0
		if (odometer.getXYT()[2] <= 350 && odometer.getXYT()[2] >= 10.0) {
			Sound.beep();
			leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, -odometer.getXYT()[2]), true);
			rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, -odometer.getXYT()[2]), false);
		}

		leftMotor.stop(true);
		rightMotor.stop();

	}

	/**
	 * This method moves the robot towards the (0,0).
	 */
	public void toOrigin() {

		navigation.turnTo(Math.PI / 4);

		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);

		sample = fetchSample();

		// Move to see a line
		while (sample > 0.38) {
			sample = fetchSample();
			leftMotor.forward();
			rightMotor.forward();

		}
		leftMotor.stop(true);
		rightMotor.stop();
		Sound.beep();

		// Move back to origin
		leftMotor.rotate(convertDistance(Lab4.WHEEL_RAD, -12), true);
		rightMotor.rotate(convertDistance(Lab4.WHEEL_RAD, -12), false);

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

	/**
	 * This method fetches the distance measured by the light sensor.
	 * @return color value
	 */
	private float fetchSample() {
		float[] colorValue = new float[idColour.sampleSize()];
		idColour.fetchSample(colorValue, 0);
		return colorValue[0];
	}

}
