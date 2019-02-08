package ca.mcgill.ecse211.lab4;

import lejos.hardware.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;

public class Lab4 {

	// Motors and sensors
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port ultrasonicPort = LocalEV3.get().getPort("S1");
	private static final Port colorPort = LocalEV3.get().getPort("S2");

	// Physical constants
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 16.2;

	// Navigation constants
	final static int FAST = 80;
	final static int SLOW = 50;
	final static int ACCELERATION = 600;
	final static double DEG_ERR = 0.6;
	final static double CM_ERR = 0.4;
	
	// Localizers
	static UltrasonicLocalizer ultrasonicLocalizer;
	static LightLocalizer lightLocalizer;

	public static void main(String[] args) {

		// Set up ultrasonic sensor
		@SuppressWarnings("resource")
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(ultrasonicPort);
		SampleProvider ultrasonicValue = ultrasonicSensor.getMode("Distance");
		float[] ultrasonicData = new float[ultrasonicValue.sampleSize()];

		// Setup color sensor
		SensorModes colorSensor = new EV3ColorSensor(colorPort);
		SampleProvider colorValue = colorSensor.getMode("Red");
		float[] colorData = new float[colorValue.sampleSize()];

		// Display
		int buttonPressed;
		
		final TextLCD text = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor, 30, true);
		Display odometryDisplay = new Display(odometer, ultrasonicValue, ultrasonicData);
		Navigation navigation = new Navigation(odometer);
		
		// Choose between both ultrasonic localizers
		do {
			text.clear();

			text.drawString("< Left | Right >", 0, 0);
			text.drawString("       |        ", 0, 1);
			text.drawString("Falling| Rising ", 0, 2);
			text.drawString(" Edge  |  Edge  ", 0, 3);

			buttonPressed = Button.waitForAnyPress();
		} while (buttonPressed != Button.ID_LEFT && buttonPressed != Button.ID_RIGHT);
		
		if (buttonPressed == Button.ID_LEFT) {
			ultrasonicLocalizer = new UltrasonicLocalizer(odometer, ultrasonicValue, ultrasonicData,
					UltrasonicLocalizer.LocalizationType.FALLING_EDGE);
			ultrasonicLocalizer.doLocalization();
		} else {
			ultrasonicLocalizer = new UltrasonicLocalizer(odometer, ultrasonicValue, ultrasonicData,
					UltrasonicLocalizer.LocalizationType.RISING_EDGE);
		}

		// Wait until light localizer is needed
		do {
			text.clear();

			text.drawString("Press Any Arrow ", 0, 0);
			text.drawString("   To continue  ", 0, 1);

			buttonPressed = Button.waitForAnyPress();
		} while (buttonPressed != Button.ID_LEFT && buttonPressed != Button.ID_RIGHT 
				&& buttonPressed != Button.ID_UP && buttonPressed != Button.ID_DOWN);

		if (buttonPressed == Button.ID_LEFT || buttonPressed == Button.ID_RIGHT
				|| buttonPressed == Button.ID_UP || buttonPressed == Button.ID_DOWN) {
			lightLocalizer = new LightLocalizer(odometer, colorValue, colorData, navigation);
			lightLocalizer.doLocalization();
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);

	}

}