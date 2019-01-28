package ca.mcgill.ecse211.lab2;

import lejos.hardware.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

public class OdometryCorrection implements Runnable {
	private static final double TILE_MEASURE = 30.000; //30.48
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;

	private static final Port colourSampler = LocalEV3.get().getPort("S4");

	private SensorModes colourSamplerSensor = new EV3ColorSensor(colourSampler);
	private SampleProvider colourSensorValue = colourSamplerSensor.getMode("Red");

	private float[] colourSensorValues = new float[colourSamplerSensor.sampleSize()];

	private float lastValue = 0;

	private int xCounterEastbound = 0; // Keeps track of how many tiles driven on the x-axis
	private int yCounterNorthbound = 0; 
	private int xCounterWestbound = 0;
	private int yCounterSouthbound  = 0;
	private double theta; // Absolute angle of the robot in relation with a fixed plane
	
	private double startNorthboundDistance = 0;
	private double startEastboundDistance = 0;
	private double startWestboundDistance = 0;
	private double startSouthboundDistance = 2;

	private double[] position;
	private double[] location;

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
	 * 
	 *             This method detects and keeps track of black lines on the floor
	 *             and its corresponding direction. With this information, more
	 *             accurate x-y coordinates are computed.
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
			// Find the difference
			float difference = value - lastValue;
			lastValue = value;
			// Notify there is a black line
			if (difference < -40) {
				Sound.twoBeeps();

				// TODO Calculate new (accurate) robot position
				// TODO Update odometer with new calculated (and more accurate) values
				position = odometer.getXYT();
				theta = position[2] * 180 / Math.PI;

				// Split into 4 directions for navigation
				// East side
				if (theta > 45 && theta < 135) {
					if(xCounterEastbound == 0) {
						location = odometer.getXYT();
						startEastboundDistance = location[0];
						xCounterEastbound++;
					}
					
					else {
						xCounterEastbound++;
						odometer.setX((xCounterEastbound * TILE_MEASURE) - (TILE_MEASURE - startEastboundDistance));
					}
				}

				// North side
				else if ((theta > 315 && theta <= 360) || (theta >= 0 && theta < 45)) {
					if(yCounterNorthbound == 0) {
						location = odometer.getXYT();
						startNorthboundDistance = location[1];
						yCounterNorthbound++;
					}
					else {
						yCounterNorthbound++;
						odometer.setY((yCounterNorthbound * TILE_MEASURE) - (TILE_MEASURE - startNorthboundDistance));
					}
				}

				// West side
				else if (theta > 225 && theta < 315) {
					if(xCounterWestbound == 0) {
						location = odometer.getXYT();
						startWestboundDistance = (3 * TILE_MEASURE) - location[0];
						xCounterWestbound++;
					}
					else {
						xCounterWestbound++;
						odometer.setX((xCounterWestbound * TILE_MEASURE) - startWestboundDistance);
					}
				}

				// South zone
				else if (theta > 135 && theta < 225) {
					if(yCounterSouthbound == 0) {
						location = odometer.getXYT();
						startSouthboundDistance = (3 * TILE_MEASURE) - location[1];
						yCounterSouthbound++;
					}
					else {
						yCounterSouthbound++;
						odometer.setY((yCounterSouthbound * TILE_MEASURE) - startSouthboundDistance);
					}
				}

				try {
					Thread.sleep(500);
				}

				catch (InterruptedException e) {
					e.printStackTrace();
				}
			}

			// This ensure the odometry correction occurs only once every period
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
