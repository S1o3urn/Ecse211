package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

//Constants declared and initialized in Lab4
import static ca.mcgill.ecse211.lab4.Lab4.*;

public class LightLocalizer {

	// Class variables
	private Odometer odometer;
	private SampleProvider colorSensor;

	// Buffers
	private float[] colorData;
	private double[] angles;
	private int angleIndex;
	private double ambientLight;
	private float lightLevel; // Light values as a mean of RGB values

	// Motors and navigation
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	Navigation navigation;

	/**
	 * The constructor.
	 * 
	 * @param odometer
	 * @param colorSensor
	 * @param colorData
	 * @param navigation
	 */
	public LightLocalizer(Odometer odometer, SampleProvider colorSensor, float[] colorData, Navigation navigation) {
		this.odometer = odometer;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		this.navigation = navigation;

		// Motors set up
		EV3LargeRegulatedMotor[] motors = odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
		this.leftMotor.setAcceleration(LIGHT_LOCALIZER_ACCELERATION);
		this.rightMotor.setAcceleration(LIGHT_LOCALIZER_ACCELERATION);

		// Initialize storage array
		angles = new double[4];
		angleIndex = 0;
	}

	/**
	 * This method implements the logic behind light localization.
	 */
	public void localize() {

		// Fetch initial ambient lighting
		ambientLight = getColorData();

		/*
		 * Assumption: robot always starts at theta of 0. Hence, moving forward at start
		 * time is translated into incrementing Y value.
		 */

		// Robot at a line and has backed up slightly
		lineDetection();

		// Rotate 90 degrees
		leftMotor.setSpeed(LIGHT_LOCALIZER_ROTATION_SPEED);
		rightMotor.setSpeed(LIGHT_LOCALIZER_ROTATION_SPEED);
		leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, 90.0), true);
		rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, 90.0), false);

		lineDetection();

		// start rotating and clock all 4 gridlines
		// we just want to rotate 360 degrees. Each time we hit something, record it's
		// angle...
		leftMotor.setSpeed(LIGHT_LOCALIZER_ROTATION_SPEED);
		rightMotor.setSpeed(LIGHT_LOCALIZER_ROTATION_SPEED);
		leftMotor.backward();
		rightMotor.forward();

		// now we want to read the four angles for our spin (we store them in an array)
		/*
		 * While we still haven't read four black lines, spin around.
		 */
		while (angleIndex < 4) {
			/*
			 * Upon reaching a black line, beep and record it.
			 */
			if (100 * Math.abs(getColorData() - ambientLight) / ambientLight > LINE_DETECTION_THRESHOLD) {
				angles[angleIndex] = odometer.getAng();
				angleIndex += 1;
				Sound.beep();
				try {
					Thread.sleep(200);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
		// now stop the motors (we have out angle values)
		leftMotor.stop(true);
		rightMotor.stop(true);

		// 0th element = first y line, 1st = first x point, 3rd = second y, 4th = second
		// x
		// calculate the deltas.
		double deltaY = angles[2] - angles[0];
		double deltaX = angles[3] - angles[1];
		// do trig to compute (0,0) and 0 degrees
		double xValue = (-1) * LIGHT_SENSOR_DISTANCE * Math.cos(Math.PI * deltaX / (2 * 180));
		double yValue = (-1) * LIGHT_SENSOR_DISTANCE * Math.cos(Math.PI * deltaY / (2 * 180));
		double thetaYMinus = angles[0];
		double deltaTheta = +180 - deltaY / 2 - thetaYMinus;

		// turn to 0 now that we have adjusted correctly
		navigation.turnTo(0, true); // navi.turnTo(deltaTheta, true);

		// set the position of the robot to where we are and an angle of 0.
		odometer.setPosition(new double[] { xValue, yValue, 0 }, new boolean[] { true, true, true });
		// now travel to 0,0 and turn to 0 (we are done!)
		navigation.travelTo(0, 0);
		navigation.turnTo(0, true);

	}

	/**
	 * This method converts degrees to radians.
	 * 
	 * @param radius
	 * @param width
	 * @param angle
	 * @return angle in radians
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * This method converts a distance into the number of wheel rotations.
	 * 
	 * @param radius
	 * @param distance
	 * @return distance in wheel rotations
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	/**
	 * This method drives the robot forward until the light sensor detects a line before stopping.
	 */
	private void lineDetection() {
		// Move forward
		leftMotor.setSpeed(LIGHT_LOCALIZER_FORWARD_SPEED);
		rightMotor.setSpeed(LIGHT_LOCALIZER_FORWARD_SPEED);
		leftMotor.forward();
		rightMotor.forward();

		// Detection of line when difference in colour is smaller than 20%, keep moving
		// forward
		while ((100 * Math.abs(getColorData() - ambientLight) / ambientLight) < LINE_DETECTION_THRESHOLD) {
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

		// Reached a line on the x or y axis
		leftMotor.stop(true);
		rightMotor.stop(true);
		
		// Back up a set amount
		leftMotor.rotate(-convertDistance(WHEEL_RADIUS, LIGHT_LOCALIZER_EDGE_DISTANCE), true); // two tiles = 60.96
		rightMotor.rotate(-convertDistance(WHEEL_RADIUS, LIGHT_LOCALIZER_EDGE_DISTANCE), false);
	}

	/**
	 * This method fetches the light sensor values and returns an overall light
	 * level.
	 * 
	 * @return lightLevel
	 */
	private float getColorData() {
		colorSensor.fetchSample(colorData, 0);
		return lightLevel = (colorData[0] + colorData[1] + colorData[2]) / 3;
	}

}
