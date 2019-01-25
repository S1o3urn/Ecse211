package ca.mcgill.ecse211.lab2;

import lejos.hardware.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

public class OdometryCorrection implements Runnable {
	private static final double TILE_MEASURE = 30.48;
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;

	private static final Port colourSampler = LocalEV3.get().getPort("S4");

	private SensorModes colourSamplerSensor = new EV3ColorSensor(colourSampler);
	private SampleProvider colourSensorValue = colourSamplerSensor.getMode("Red");

	private float[] colourSensorValues = new float[colourSamplerSensor.sampleSize()];

	private float lastValue = 0;

	private int xCounter = 0;
	private int yCounter = 0;
	private double theta;

	private double[] position;

	/**
	 * This is the default class constructor. An existing instance of the odometer
	 * is used. This is to ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection() throws OdometerExceptions {

		this.odometer = Odometer.getOdometer();
	}

	/**
	 * Here is where the odometer correction code should be run.
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;

		while (true) {
			correctionStart = System.currentTimeMillis();

			// TODO Trigger correction (When do I have information to correct?)
			// Check if there's a black line
			// Get data from colour sensor
			colourSensorValue.fetchSample(colourSensorValues, 0);
			// Scale up for simpler computations
			float value = colourSensorValues[0] * 1000;
			// Find the derivative
			float difference = value - lastValue;
			lastValue = value;
			// There is a black line
			if (difference < -50) {
				Sound.beep();

				// TODO Calculate new (accurate) robot position
				// TODO Update odometer with new calculated (and more accurate) values
				position = odometer.getXYT();
				theta = position[2] * 180 / Math.PI;

				if ((theta <= 360 && theta >= 315) || (theta >= 0 && theta <= 45)) {
					odometer.setY(yCounter * TILE_MEASURE);
					yCounter++;
				}

				else if (theta > 45 && theta <= 315) {
					odometer.setX(xCounter * TILE_MEASURE);
					xCounter++;
				}

				else if (theta > 135 && theta <= 225) {
					yCounter--;
					odometer.setY(yCounter * TILE_MEASURE);
				}

				else if (theta > 225 && theta < 315) {
					xCounter--;
					odometer.setX(xCounter * TILE_MEASURE);
				}

				try {
					Thread.sleep(500);
				}

				catch (InterruptedException e) {
					e.printStackTrace();
				}
			}

			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
	}
}
