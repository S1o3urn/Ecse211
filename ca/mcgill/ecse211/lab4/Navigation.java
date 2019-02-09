package ca.mcgill.ecse211.lab4;

/**
 * This class implements the navigation functionality of the robot.
 * It provides methods to travel to waypoints, turn and set speeds for both wheels independently.
 * @author Tian Han Jiang
 */
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import static ca.mcgill.ecse211.lab4.Lab4.*;	// constants declared and initialized in Lab4

public class Navigation {

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	/**
	 * The constructor.
	 * @param odometer
	 */
	public Navigation(Odometer odometer) {
		this.odometer = odometer;

		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];

		// Configure acceleration speed
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
	}

	/**
	 * This method sets the speeds of both left and right motors as floats.
	 * This method is overloaded with the other setSpeeds method.
	 * @param leftSpeed
	 * @param rightSpeed
	 */
	public void setSpeeds(float leftSpeed, float rightSpeed) {
		this.leftMotor.setSpeed(leftSpeed);
		this.rightMotor.setSpeed(rightSpeed);
		
		if (leftSpeed < 0) {
			this.leftMotor.backward();
		} else {
			this.leftMotor.forward();
		}
		
		if (rightSpeed < 0) {
			this.rightMotor.backward();
		} else {
			this.rightMotor.forward();
		}
	}

	/**
	 * This method sets the speeds of both left and right motors as integers.
	 * This method is overloaded with the other setSpeeds method.
	 * @param leftSpeed
	 * @param rightSpeed
	 */
	public void setSpeeds(int leftSpeed, int rightSpeed) {
		this.leftMotor.setSpeed(leftSpeed);
		this.rightMotor.setSpeed(rightSpeed);
		
		if (leftSpeed < 0) {
			this.leftMotor.backward();
		} else {
			this.leftMotor.forward();
		}
		
		if (rightSpeed < 0) {
			this.rightMotor.backward();
		} else {
			this.rightMotor.forward();
		}
	}

	/**
	 * This method sets both motors in float mode.
	 */
	public void setMotorsToFloat() {
		this.leftMotor.stop();
		this.rightMotor.stop();
		this.leftMotor.flt(true);
		this.rightMotor.flt(true);
	}
	
	/**
	 * The travelTo method takes in an (x, y) coordinate in cm
	 * and will travel to that designated coordinate while updating its position.
	 * @param x
	 * @param y
	 */
	public void travelTo(double x, double y) {
		double minAng;
		while (Math.abs(x - odometer.getX()) > DISTANCE_ERROR || Math.abs(y - odometer.getY()) > DISTANCE_ERROR) {
			minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);
			if (minAng < 0)
				minAng += 360.0;
			this.turnTo(minAng, false);
			this.setSpeeds(FAST_SPEED, FAST_SPEED);
		}
		this.setSpeeds(0, 0);
	}

	/**
	 * The turnTo method takes in a angle and a boolean
	 * and will turn the robot to the angle.
	 * The boolean variable is there to stop the motors when the turn is complete.
	 * @param angle
	 * @param stop
	 */
	public void turnTo(double angle, boolean stop) {

		double error = angle - this.odometer.getAng();

		while (Math.abs(error) > DEGREE_ERROR) {

			error = angle - this.odometer.getAng();

			if (error < -180.0) {
				this.setSpeeds(-SLOW_SPEED, SLOW_SPEED);
			} else if (error < 0.0) {
				this.setSpeeds(SLOW_SPEED, -SLOW_SPEED);
			} else if (error > 180.0) {
				this.setSpeeds(SLOW_SPEED, -SLOW_SPEED);
			} else {
				this.setSpeeds(-SLOW_SPEED, SLOW_SPEED);
			}
		}

		if (stop) {
			this.setSpeeds(0, 0);
		}
	}

	/**
	 * The driveForward method drives the robot forward a set distance.
	 * @param distance
	 */
	public void driveForward(double distance) {
		this.travelTo(Math.cos(Math.toRadians(this.odometer.getAng())) * distance,
				Math.cos(Math.toRadians(this.odometer.getAng())) * distance);

	}
}