package ca.mcgill.ecse211.lab4;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class Display implements TimerListener {
	private Odometer odometer;
	private Timer screenTimer;
	private TextLCD screen = LocalEV3.get().getTextLCD();
	private SampleProvider ultrasonicSensor;
	private float[] ultrasonicData;		// For Display
	private double[] position;		// For Display
	
	public static final int LCD_REFRESH = 100;

	public Display(Odometer odo, SampleProvider usSensor, float[] usData) {
		this.odometer = odometer;
		this.screenTimer = new Timer(LCD_REFRESH, this);

		// initialise the arrays for displaying data
		position = new double[3];
		this.ultrasonicSensor = usSensor;
		this.ultrasonicData = usData;
		// start the timer
		screenTimer.start();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see lejos.utility.TimerListener#timedOut() Prints the info on the LCD.
	 */
	public void timedOut() {
		odometer.getPosition(position);
		screen.clear();
		screen.drawString("X: ", 0, 0);
		screen.drawString("Y: ", 0, 1);
		screen.drawString("H: ", 0, 2);
		screen.drawString("U: ", 0, 3);
		screen.drawString(String.valueOf(position[0] * 10), 3, 0);
		screen.drawString(String.valueOf(position[1] * 10), 3, 1);
		screen.drawString(String.valueOf(position[2]), 3, 2);
		screen.drawString(String.valueOf(getFilteredData()), 3, 3);
	}

	/*
	 * Gets the data from the Ultrasonic sensor.
	 */
	private float getFilteredData() {
		ultrasonicSensor.fetchSample(ultrasonicData, 0);
		float distance = (int) (ultrasonicData[0] * 100.0);

		return distance;
	}
}
