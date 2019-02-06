package ca.mcgill.ecse211.lab3;

/**
 * This class displays user settings to switch between use modes
 * 
 * motors, sensors and changeable constants are all declared and initialized here for interface simplicity
 * @author Tian Han Jiang
 * Inspired from lab2 and previous years code
 */

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.Button;
import lejos.hardware.port.Port;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class EV3Navigation {

	// Initialize class variables
	// Motors
	/**
	 * The left motor.
	 */
	private static final EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	
	/**
	 * The right motor.
	 */
	private static final EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	/**
	 * The sensor motor.
	 */
	private static final EV3LargeRegulatedMotor SENSOR_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	
	/**
	 * The port associated to the ultrasonic sensor port.
	 */
	private static final Port ULTRASONIC_SENSOR_PORT = LocalEV3.get().getPort("S4");

	/**
	 * The constant used to determine the wheel radius.
	 */
	public static final double WHEEL_RADIUS = 2.15;
	
	/**
	 * The constant used to represent the wheel base.
	 */
	public static final double WHEEL_BASE = 13.1;	//13.25 originally from lab2
	
	/**
	 * The forward speed.
	 */
	public static final int FORWARD_SPEED = 250;
	
	/**
	 * The rotate speed, used when calling the turnTo method.
	 */
	public static final int ROTATE_SPEED = 150;
	
	/**
	 * The tile length.
	 */
	public static final double TILE_MEASURE = 30.00;

	public static void main(String[] args) {

		// Sensor setup
		@SuppressWarnings("resource") // Since ultrasonic sensor is always on
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(ULTRASONIC_SENSOR_PORT); // ultrasonicSensor is the
																						// instance
		SampleProvider ultrasonicDistance = ultrasonicSensor.getMode("Distance"); // ultrasonicDistance provides samples
																					// from this instance
		float[] ultrasonicData = new float[ultrasonicDistance.sampleSize()]; // ultrasonicData is the buffer in which
																				// data are returned

		// Display
		int buttonPressed;

		final TextLCD text = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(LEFT_MOTOR, RIGHT_MOTOR);
		Display odometryDisplay = new Display(odometer, text);
		Navigation navigation = new Navigation(odometer, LEFT_MOTOR, RIGHT_MOTOR);
		ObstacleAvoidanceNavigation advancedNavigation = new ObstacleAvoidanceNavigation(odometer, LEFT_MOTOR,
				RIGHT_MOTOR, SENSOR_MOTOR, ultrasonicDistance, ultrasonicData);

		do {
			text.clear();

			text.drawString("< Left | Right >", 0, 0);
			text.drawString("       |        ", 0, 1);
			text.drawString(" Simple|   adv  ", 0, 2);
			text.drawString(" Nav   |   Nav  ", 0, 3);

			buttonPressed = Button.waitForAnyPress();
		} while (buttonPressed != Button.ID_LEFT && buttonPressed != Button.ID_RIGHT);

		if (buttonPressed == Button.ID_LEFT) {
			odometer.start();
			odometryDisplay.start();
			navigation.start();
		} else {
			odometer.start();
			odometryDisplay.start();
			advancedNavigation.start();
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}
}
