package ca.mcgill.ecse211.lab4;

/**
 * This class implements the display functionality.
 * Given the odometer's data, the robot's position is displayed on it's LCD screen at a fixed interval.
 * @author Tian Han Jiang
 */
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

import static ca.mcgill.ecse211.lab4.Lab4.*;

public class Display implements TimerListener {
	private Odometer odometer;
	private Timer screenTimer;
	private TextLCD screen = LocalEV3.get().getTextLCD();
	private SampleProvider ultrasonicSensor;
	private float[] ultrasonicData;		// For Display
	private double[] position;			// For Display

	/**
	 * The constructor.
	 * @param odometer
	 * @param ultrasonicSensor
	 * @param ultrasonicData
	 */
	public Display(Odometer odometer, SampleProvider ultrasonicSensor, float[] ultrasonicData) {
		this.odometer = odometer;
		this.screenTimer = new Timer(LCD_REFRESH_RATE, this);

		// initialize data displaying arrays
		position = new double[3];
		this.ultrasonicSensor = ultrasonicSensor;
		this.ultrasonicData = ultrasonicData;
		// start the timer
		screenTimer.start();
	}

	/**
	 * This method displays the robot's last position when the timer has count down.
	 */
	public void timedOut() {
		// Retrieve last position
		odometer.getPosition(position);
		
		// Update information on screen
		screen.clear();
		screen.drawString("X:       ", 0, 0);
		screen.drawString("Y:       ", 0, 1);
		screen.drawString("Theta:   ", 0, 2);
		screen.drawString("US dist.:", 0, 3);
		screen.drawString(String.valueOf(position[0] * 10), 3, 0);
		screen.drawString(String.valueOf(position[1] * 10), 3, 1);
		screen.drawString(String.valueOf(position[2]), 3, 2);
		screen.drawString(String.valueOf(getFilteredData()), 3, 3);
	}

	/**
	 * This method returns the ultrasonic sensor's measured value after transformation.
	 * @return distance
	 */
	private float getFilteredData() {
		ultrasonicSensor.fetchSample(ultrasonicData, 0);
		float distance = (int) (ultrasonicData[0] * 100.0);
		return distance;
	}
}
