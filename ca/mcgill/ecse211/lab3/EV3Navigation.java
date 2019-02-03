package ca.mcgill.ecse211.lab3;

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
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final Port ultrasonicSensorPort = LocalEV3.get().getPort("S4");

	// Wheel constants
	public static final double WHEEL_RADIUS = 2.3;
	public static final double WHEEL_BASE = 13.25;

	public static void main(String[] args) {

		// Sensor setup
		@SuppressWarnings("resource") // Since ultrasonic sensor is always on
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(ultrasonicSensorPort); // ultrasonicSensor is the
																						// instance
		SampleProvider ultrasonicDistance = ultrasonicSensor.getMode("Distance"); // ultrasonicDistance provides samples
																					// from this instance
		float[] ultrasonicData = new float[ultrasonicDistance.sampleSize()]; // ultrasonicData is the buffer in which
																				// data are returned

		// Display
		int buttonPressed;
		
		final TextLCD text = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		Display odometryDisplay = new Display(odometer, text);
		Navigation navigation = new Navigation(odometer, leftMotor, rightMotor);
		ObstacleAvoidanceNavigation advancedNavigation = new ObstacleAvoidanceNavigation(odometer, leftMotor,
				rightMotor, sensorMotor, ultrasonicDistance, ultrasonicData);

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
