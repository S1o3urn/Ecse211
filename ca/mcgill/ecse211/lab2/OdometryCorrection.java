package ca.mcgill.ecse211.lab2;

/*
 * This class corrects the calculated positions in the Odometer class and updates its values
 */
import lejos.hardware.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

public class OdometryCorrection implements Runnable {
	private static final double TILE_MEASURE = 30.48; // 30.48
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;

	private static final Port colourSampler = LocalEV3.get().getPort("S4");

	private SensorModes colourSamplerSensor = new EV3ColorSensor(colourSampler);
	private SampleProvider colourSensorValue = colourSamplerSensor.getMode("Red");

	private float[] colourSensorValues = new float[colourSamplerSensor.sampleSize()];

	private float lastValue = 0;

	/*
	 * private int xCounterEastbound = 0; // Keeps track of how many tiles driven on
	 * the x-axis private int yCounterNorthbound = 0; private int xCounterWestbound
	 * = 0; private int yCounterSouthbound = 0;
	 */
	private int yCounterNorthbound = 0;
	private double startNorthboundDistance = 0;
	private double theta; // Absolute angle of the robot in relation with a fixed plane

	/*
	 * private double startNorthboundDistance = 0; private double
	 * startEastboundDistance = 0; private double startWestboundDistance = 0;
	 * private double startSouthboundDistance = 0;
	 * 
	 * private double firstLineSouthbound; private double firstLineWestbound;
	 * private double topSouthDistance; private double topWestDistance;
	 */

	private double[] position;

	/*
	 * private double northY = 0; private double eastX = 0; private double southY =
	 * 0; private double westX = 0;
	 */

	double counterY = 1;
	double counterX = 1;

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
				
				//Depending on theta, knows which direction the robot is going and calculates correct distance
				//Heading north relative to origin
				if ((theta <= 360 && theta >= 315) || (theta >= 0 && theta <= 45)) {
					//Find distance from starting position to first black line
					if (yCounterNorthbound == 0) {
						startNorthboundDistance = position[1];
						yCounterNorthbound++;
					} 
					//For every other line, add a tile measures length
					else {
						odometer.setY(counterY * TILE_MEASURE + startNorthboundDistance);
						counterY++;
					}
				} 
				//Heading east
				else if (theta > 45 && theta <= 135) {
					odometer.setX(counterX * TILE_MEASURE);
					counterX++;
				} 
				//Heading south
				else if (theta > 135 && theta <= 225) {
					counterY--;
					odometer.setY((counterY - 1) * TILE_MEASURE + startNorthboundDistance);
				} 
				//Heading west
				else if (theta > 225 && theta < 315) {
					counterX--;
					odometer.setX(counterX * TILE_MEASURE);
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
