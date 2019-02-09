package ca.mcgill.ecse211.lab4;

/**
 * This class declares and instanciates the motors and sensors.
 * It also provides an interface to choose between localizer methods.
 * 
 * @author Tian Han Jiang
 */
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
	final static int FAST_SPEED = 80;
	final static int SLOW_SPEED = 50;
	final static int ACCELERATION = 600;
	final static double DEGREE_ERROR = 0.6;
	final static double DISTANCE_ERROR = 0.4;
	
	// Display constants
	final static int LCD_REFRESH_RATE = 100;
	
	// Light localizer constants
	public static int LIGHT_LOCALIZER_ROTATION_SPEED = 60;
	public static int LIGHT_LOCALIZER_FORWARD_SPEED = 100;
	public static int LIGHT_LOCALIZER_EDGE_DISTANCE = 18;
	public static int LIGHT_LOCALIZER_ACCELERATION = 600;
	public static int LIGHT_SENSOR_DISTANCE = 15;
	public static double LINE_DETECTION_THRESHOLD = 20;
	
	// Localizers
	static UltrasonicLocalizer ultrasonicLocalizer;
	static LightLocalizer lightLocalizer;

	public static void main(String[] args) {

		// Set up ultrasonic sensor
		@SuppressWarnings("resource")	// Doesn't need to be closed
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(ultrasonicPort);
		SampleProvider ultrasonicValue = ultrasonicSensor.getMode("Distance");
		float[] ultrasonicData = new float[ultrasonicValue.sampleSize()];

		// Setup color sensor
		@SuppressWarnings("resource")	// Doesn't need to be closed
		SensorModes colorSensor = new EV3ColorSensor(colorPort);
		SampleProvider colorValue = colorSensor.getMode("RGB");
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
		// Light sensor activated on any arrow key pressed
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
			lightLocalizer.localize();
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);

	}

}