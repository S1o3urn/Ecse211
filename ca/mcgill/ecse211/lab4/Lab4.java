package ca.mcgill.ecse211.lab4;

/**
 * This class is the base uniting all other classes. 
 * It displays useful information on the LCD screen.
 * 
 * @author tianh
 */

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.localization.*;

public class Lab4 {

	// Motors and sensors
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port usPort = LocalEV3.get().getPort("S2");
	private static boolean isRisingEdge = true;

	// Vehicle physical constants
	public static final double WHEEL_RAD = 2.15;
	public static final double TRACK = 13.5;	// 13.2

	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// Odometer and display
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Display odometryDisplay = new Display(lcd);

		@SuppressWarnings("resource")	// Never closed
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");

		do {
			// Reset screen
			lcd.clear();

			// GUI
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("Rising |Falling ", 0, 2);
			lcd.drawString(" Edge  |  Edge  ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			isRisingEdge = true;
		} else {
			isRisingEdge = false;
		}

		// Odometer thread
		Thread odoThread = new Thread(odometer);
		odoThread.start();

		// Odometer display thread
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();

		// USLocalizer and lightLocaliser
		USLocalizer USLocalizer = new USLocalizer(odometer, leftMotor, rightMotor, isRisingEdge, usDistance);
		LightLocalizer lightLocatizer = new LightLocalizer(odometer, leftMotor, rightMotor);

		USLocalizer.localize();
		
		// Application shut down for easier reach
		if(Button.waitForAnyPress() == Button.ID_DOWN) {
			System.exit(0);
		}
		
		// Light localization on right arrow press
		while (Button.waitForAnyPress() != Button.ID_RIGHT);

		lightLocatizer.localize();

		while (Button.waitForAnyPress() != Button.ID_RIGHT);
		System.exit(0);
	}

}