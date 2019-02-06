package ca.mcgill.ecse211.lab3;

/**
 * This class implements the logic behind the display 
 * while the robot is running in either simple navigation 
 * or obstacle avoidance navigation
 * 
 * adapted from lab2 Display.java
 * and previous years Display.java
 */
import java.text.DecimalFormat;
import lejos.hardware.lcd.TextLCD;

public class Display extends Thread {

	/**
	 * The odometer instance.
	 */
	private Odometer odometer;
	
	/**
	 * The LCD display instance.
	 */
	private TextLCD LCD;
	
	/** 
	 * The refresh rate for the display.
	 */
	private static final long DISPLAY_PERIOD = 25;

	/**
	 * Constructor
	 * 
	 * @param odometer
	 * @param text
	 */
	public Display(Odometer odometer, TextLCD lcd) {
		this.odometer = odometer;
		this.LCD = lcd;
	}

	// run method (required for Thread)
	public void run() {
		long displayStart;
		long displayEnd;
		double[] position = new double[3];

		// Reset display
		LCD.clear();

		while (true) {
			displayStart = System.currentTimeMillis();

			// Retrieve odometer position information
			odometer.getPosition(position, new boolean[] { true, true, true });

			// display odometry information
			DecimalFormat numberFormat = new DecimalFormat("######0.00");
			LCD.drawString("X: " + numberFormat.format(position[0]), 0, 0);
			LCD.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
			LCD.drawString("T: " + numberFormat.format(position[2]), 0, 2);

			// Check to make sure update only happens every period
			displayEnd = System.currentTimeMillis();
			if (displayEnd - displayStart < DISPLAY_PERIOD) {
				try {
					Thread.sleep(DISPLAY_PERIOD - (displayEnd - displayStart));
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
	}
}