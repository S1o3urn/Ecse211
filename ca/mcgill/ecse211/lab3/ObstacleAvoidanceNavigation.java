package ca.mcgill.ecse211.lab3;

/**
 * This class implements an obstacle avoidance navigation.
 * While navigating towards a waypoint, if an obstacle is detected, a bang-bang style controller is activated.
 * 
 * @author Tian Han Jiang
 */
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

import static ca.mcgill.ecse211.lab3.EV3Navigation.*;

public class ObstacleAvoidanceNavigation extends Thread {

	// Motors and sensors
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor sensorMotor;
	private SampleProvider ultrasonicSensor;
	private float[] ultrasonicData;

	
	/**
	 * The sensor motor turn speed.
	 */
	private static final int SCAN_SPEED = 175;
	
	/**
	 * The max right angle for the sensor.
	 */
	private static final int RIGHT_ANGLE = 55;
	
	/**
	 * The max left angle for the sensor.
	 */
	private static final int LEFT_ANGLE = -55;
	
	/**
	 * The critical count for changing rotation direction of the sensor.
	 */
	private static final int MAX_TACHO_COUNT = 10;
	
	/**
	 * The sensor angle while bang-bang style controller is engaged.
	 */
	private static final int OBSTACLE_SENSOR_ANGLE = -45;
	
	/**
	 * The forward speed while in obstacle avoidance mode.
	 */
	private static final int OBSTACLE_FWD_SPEED = 135;
	
	/**
	 * The turning in speed while in obstacle avoidance mode.
	 */
	private static final int OBSTACLE_TURN_IN_SPEED = 225;
	
	/**
	 * The obstacle turn out speed while in obstacle avoidance mode.
	 */
	private static final int OBSTACLE_TURN_OUT_SPEED = 25;
	
	/**
	 * Pi value.
	 */
	private static final double PI = Math.PI;
	
	/** 
	 * Sensor data filter critical value.
	 */
	private static final int FILTER_OUT = 20;
	
	/**
	 * The filter control value.
	 */
	private int filterControl = 0;

	int sensorDistance;
	int bandCenter = 14;
	int bandWidth = 2;

	private static boolean navigating = true;

	/**
	 * Constructor
	 * 
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 * @param sensorMotor
	 * @param ultrasonicSensor
	 * @param utrasonicData
	 */
	public ObstacleAvoidanceNavigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor sensorMotor, SampleProvider ultrasonicSensor,
			float[] utrasonicData) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sensorMotor = sensorMotor;
		this.ultrasonicSensor = ultrasonicSensor;
		this.ultrasonicData = utrasonicData;
	}

	@Override
	public void run() {

		// Input travel points here
		travelTo((1 * TILE_MEASURE), (1 * TILE_MEASURE));
		travelTo((0 * TILE_MEASURE), (2 * TILE_MEASURE));
		travelTo((2 * TILE_MEASURE), (2 * TILE_MEASURE));
		travelTo((2 * TILE_MEASURE), (1 * TILE_MEASURE));
		travelTo((1 * TILE_MEASURE), (0 * TILE_MEASURE));
	}

	/**
	 * This method is operates the same way as that of Navigation.java However, it
	 * also implements the avoiding obstacle feature
	 * 
	 * @param x
	 * @param y
	 */
	public void travelTo(double x, double y) {

		// Path calculation variables
		double xPath;
		double yPath;
		double path;
		double angle;

		// Reset motors
		leftMotor.stop();
		rightMotor.stop();
		leftMotor.setAcceleration(3000);
		rightMotor.setAcceleration(3000);

		// Calculate path and angle
		xPath = x - odometer.getX();
		yPath = y - odometer.getY();
		path = Math.hypot(xPath, yPath);
		angle = Math.atan2(xPath, yPath);

		// Turn to face the waypoint
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		turnTo(angle);

		// Advance forward equal to path distance
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(distanceToRotations(path), true);
		rightMotor.rotate(distanceToRotations(path), true);

		// Sensor motor setup
		sensorMotor.resetTachoCount();
		sensorMotor.setSpeed(SCAN_SPEED);

		// Obstacle avoidance sensor
		while (leftMotor.isMoving() || rightMotor.isMoving()) {
			// Sensor not rotating, determine which way to rotate
			while (!(sensorMotor.isMoving())) {
				if (sensorMotor.getTachoCount() <= MAX_TACHO_COUNT) {
					sensorMotor.rotateTo(RIGHT_ANGLE, true);
				} else {
					sensorMotor.rotateTo(LEFT_ANGLE, true);
				}
			}

			// Fetch sensor data and filter
			ultrasonicSensor.fetchSample(ultrasonicData, 0);
			sensorDistance = (int) (ultrasonicData[0] * 100.0);
			filter(sensorDistance);

			// Turn left until sensor distance is within bandCenter
			if (sensorDistance <= bandCenter) {
				leftMotor.stop(true);
				rightMotor.stop(false);
				navigating = false;
			}
			try {
				Thread.sleep(50);
			} catch (Exception e) {
			} // Poor man's timed sampling
		}

		// Obstacle avoidance
		if (!this.isNavigating()) {
			avoidObstacle();
			sensorMotor.rotateTo(0);
			navigating = true;
			travelTo(x, y);
			return;
		}

		// When vehicle reach a waypoint, reset sensor to rest position
		sensorMotor.rotateTo(0);

	}

	/**
	 * This method implements the logic behind turning to face a waypoint
	 * 
	 * @param theta
	 */
	public void turnTo(double theta) {

		double angle = getMinAngle(theta - odometer.getTheta());

		leftMotor.rotate(radianToDegree(angle), true);
		rightMotor.rotate(-radianToDegree(angle), false);
	}

	/**
	 * This method calculates the minimum angle required to face the waypoint
	 * 
	 * @param angle
	 */
	public double getMinAngle(double angle) {
		if (angle > PI) {
			angle = angle - 2 * PI;
		} else if (angle < -PI) {
			angle = 2 * PI + angle;
		}
		return angle;
	}

	/**
	 * Checks to see if the robot is on the move or not
	 * 
	 * @return true/false
	 */
	public boolean isNavigating() {
		return navigating;
	}

	/**
	 * This method takes in the total distance needed to travel and transforms it
	 * into the number of wheel rotations needed
	 * 
	 * @param distance
	 * @return
	 */
	public int distanceToRotations(double distance) {
		return (int) (180 * distance / (PI * WHEEL_RADIUS));
	}

	/**
	 * This method converts radians into degrees
	 * 
	 * @param angle
	 * @return wheel rotations needed
	 */
	public int radianToDegree(double angle) {
		return distanceToRotations(WHEEL_BASE * angle / 2);
	}

	/**
	 * This method implements a Bang-Bang type logic to avoid an obstacle
	 */
	public void avoidObstacle() {

			// Turn 90degrees right
			turnTo(odometer.getTheta() - PI / 2);

			// Adjust sensor motor to peak angle for obstacle detection
			sensorMotor.rotateTo(OBSTACLE_SENSOR_ANGLE);

		// Define when to stop obstacle avoidance
		double endAngle = odometer.getTheta() + PI * 0.8;

		// Bang-Bang controller like logic to avoid obstacle
		while (odometer.getTheta() < endAngle) {
			ultrasonicSensor.fetchSample(ultrasonicData, 0);
			sensorDistance = (int) (ultrasonicData[0] * 100.0);
			int errorDistance = bandCenter - sensorDistance;

			// Drive straight
			if (Math.abs(errorDistance) <= bandWidth) {
				leftMotor.setSpeed(OBSTACLE_FWD_SPEED);
				rightMotor.setSpeed(OBSTACLE_FWD_SPEED);
				leftMotor.forward();
				rightMotor.forward();
			}

			// Robot is on the inside of the offset and over the bandwidth
			else if (errorDistance > 0) {

					rightMotor.setSpeed(OBSTACLE_FWD_SPEED);
					leftMotor.setSpeed(OBSTACLE_TURN_OUT_SPEED);
					rightMotor.forward();
					leftMotor.backward();
			}

			// Robot is on the outside of the offset and over the bandwidth
			else if (errorDistance < 0) {
				
					rightMotor.setSpeed(OBSTACLE_FWD_SPEED);
					leftMotor.setSpeed(OBSTACLE_TURN_IN_SPEED);
					rightMotor.forward();
					leftMotor.forward();
			}
		}

		// Finished cornering the obstacle
		leftMotor.stop();
		rightMotor.stop();

	}

	/**
	 * This method fetches the distance measured by the ultrasonic sensor
	 * 
	 * @return sensorDistance
	 */
	public int readUSDistance() {
		return this.sensorDistance;
	}

	// Filtering out bad results
	public void filter(int distance) {

		// Basic filter adapted from lab1
		// There is an abnormally large value
		if (distance >= 75 && filterControl < FILTER_OUT) {
			filterControl++;
		}

		// FILTER_OUT amount of repeated large value
		// meaning there could be nothing there to scan
		else if (distance >= 75) {
			this.sensorDistance = distance;
		}

		// Filtered values
		else {
			filterControl = 0;
			this.sensorDistance = distance;
		}
	}

}