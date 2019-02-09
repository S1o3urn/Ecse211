package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

import static ca.mcgill.ecse211.lab4.Lab4.*;

public class LightLocalizer {
	private static final double LIGHT_LOCALIZER_EGDE_DISTANCE = 0;

	// class variables
	private Odometer odometer;
	private SampleProvider colorSensor;
	private float[] colorData;
	private double[] angles;
	private int angleIndex;
	private double firstBrightness;
	// motors
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	Navigation navigation;

	public LightLocalizer(Odometer odometer, SampleProvider colorSensor, float[] colorData, Navigation navigation) {
		// get incoming values for variables
		this.odometer = odometer;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		this.navigation = navigation;
		// set up motors
		EV3LargeRegulatedMotor[] motors = odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
		this.leftMotor.setAcceleration(LIGHT_LOCALIZER_ACCELERATION);
		this.rightMotor.setAcceleration(LIGHT_LOCALIZER_ACCELERATION);
		// initialize arrays
		angles = new double[4];
		angleIndex = 0;
	}

	/*
	 * Localizes the robot (using the light sensor)
	 */
	public void doLocalization() {
		// gets the first brightness (so it knows what to base color readings everything
		// off of)
		firstBrightness = getColorData();

		// we assume we are at theta 0 to start...
		// set the speeds of the motors
		leftMotor.setSpeed(LIGHT_LOCALIZER_FORWARD_SPEED);
		rightMotor.setSpeed(LIGHT_LOCALIZER_FORWARD_SPEED);
		leftMotor.forward();
		rightMotor.forward();

		// until we reach a black line go forward ((this will be the first
		// y-vertical-line))
		while (100 * Math.abs(getColorData() - firstBrightness) / firstBrightness < LINE_DETECTION_THRESHOLD) {
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		// now that we've reached a black line, stop.
		leftMotor.stop(true);
		rightMotor.stop(true);
		// now go backwards a set amount
		leftMotor.rotate(-convertDistance(Lab4.WHEEL_RADIUS, LIGHT_LOCALIZER_EGDE_DISTANCE), true); // two tiles = 60.96
		rightMotor.rotate(-convertDistance(Lab4.WHEEL_RADIUS, LIGHT_LOCALIZER_EDGE_DISTANCE), false);

		// now rotate to do the same thing in the Y.
		leftMotor.setSpeed(LIGHT_LOCALIZER_ROTATION_SPEED);
		rightMotor.setSpeed(LIGHT_LOCALIZER_ROTATION_SPEED);

		// we must rotate 90 degrees
		leftMotor.rotate(-convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, 90.0), true);
		rightMotor.rotate(convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, 90.0), false);

		// theta is now 90.
		// set the speeds of the motors
		leftMotor.setSpeed(LIGHT_LOCALIZER_FORWARD_SPEED);
		rightMotor.setSpeed(LIGHT_LOCALIZER_FORWARD_SPEED);
		leftMotor.forward();
		rightMotor.forward();

		// go forward until we hit a black line (this will be the first
		// x-horizontal-line)
		while (100 * Math.abs(getColorData() - firstBrightness) / firstBrightness < LINE_DETECTION_THRESHOLD) {
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		// now stop the robot
		leftMotor.stop(true);
		rightMotor.stop(true);
		// now go backwards a little bit.
		leftMotor.rotate(-convertDistance(Lab4.WHEEL_RADIUS, LIGHT_LOCALIZER_EDGE_DISTANCE), true); // two tiles = 60.96
		rightMotor.rotate(-convertDistance(Lab4.WHEEL_RADIUS, LIGHT_LOCALIZER_EDGE_DISTANCE), false);

		// start rotating and clock all 4 gridlines
		// we just want to rotate 360 degrees. Each time we hit something, record it's
		// angle...
		leftMotor.setSpeed(LIGHT_LOCALIZER_ROTATION_SPEED);
		rightMotor.setSpeed(LIGHT_LOCALIZER_ROTATION_SPEED);
		leftMotor.backward();
		rightMotor.forward();

		// now we want to read the four angles for our spin (we store them in an array)
		/*
		 * While we still haven't read four black lines, spin around.
		 */
		while (angleIndex < 4) {
			/*
			 * Upon reaching a black line, beep and record it.
			 */
			if (100 * Math.abs(getColorData() - firstBrightness) / firstBrightness > LINE_DETECTION_THRESHOLD) { // getColorData()
																													// -
																													// firstBrightness
																													// >
																													// 10){
				angles[angleIndex] = odometer.getAng();
				angleIndex += 1;
				Sound.beep();
				try {
					Thread.sleep(200);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
		// now stop the motors (we have out angle values)
		leftMotor.stop(true);
		rightMotor.stop(true);

		// 0th element = first y line, 1st = first x point, 3rd = second y, 4th = second
		// x
		// calculate the deltas.
		double deltaY = angles[2] - angles[0];
		double deltaX = angles[3] - angles[1];
		// do trig to compute (0,0) and 0 degrees
		double xValue = (-1) * LIGHT_SENSOR_DISTANCE * Math.cos(Math.PI * deltaX / (2 * 180));
		double yValue = (-1) * LIGHT_SENSOR_DISTANCE * Math.cos(Math.PI * deltaY / (2 * 180));
		double thetaYMinus = angles[0];
		double deltaTheta = +180 - deltaY / 2 - thetaYMinus;

		// turn to 0 now that we have adjusted correctly
		navigation.turnTo(0, true); // navi.turnTo(deltaTheta, true);

		// set the position of the robot to where we are and an angle of 0.
		odometer.setPosition(new double[] { xValue, yValue, 0 }, new boolean[] { true, true, true });
		// now travel to 0,0 and turn to 0 (we are done!)
		navigation.travelTo(0, 0);
		navigation.turnTo(0, true);

	}

	// conversion methods
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	// gets the data from the color sensor, and returns a value corresponding
	// to the overall "brightness".
	private float getColorData() {
		colorSensor.fetchSample(colorData, 0);
		// we define the brightness as the average of the magnitudes of R,G,B (really
		// "Whiteness")
		float colorBrightnessLevel = (colorData[0] + colorData[1] + colorData[2]);
		return colorBrightnessLevel;
	}

}
