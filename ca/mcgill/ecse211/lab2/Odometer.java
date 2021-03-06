/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.lab2;

/*
 * This class implements the odometer functionality for lab 2
 * Given the wheel radius, the speed at which is travels and the tachometer count, 
 * it is then possible to calculate the distance traveled
 *  in both an x and y ax-s defined at program launch time
 */
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends OdometerData implements Runnable {

	private OdometerData odoData;
	private static Odometer odo = null; // Returned as singleton

	// Motors and related variables
	private int leftMotorTachoCount;
	private int rightMotorTachoCount;
	private int lastLeftMotorTachoCount;
	private int lastRightMotorTachoCount;

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private final double TRACK;
	private final double WHEEL_RAD;

	private double[] position;
	private double leftWheelDistance;
	private double rightWheelDistance;
	private double deltaDistance;
	private double deltaX;
	private double deltaY;
	private double deltaTheta;

	private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

	/**
	 * This is the default constructor of this class. It initiates all motors and
	 * variables once.It cannot be accessed externally.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @throws OdometerExceptions
	 */
	private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, final double TRACK,
			final double WHEEL_RAD) throws OdometerExceptions {
		odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
													// manipulation methods
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		// Reset the values of x, y and z to 0
		odoData.setXYT(0, 0, 0);

		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;
		// lastLeftMotorTachoCount = this.leftMotorTachoCount;
		// lastRightMotorTachoCount = this.rightMotorTachoCount;
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;

	}

	/**
	 * This method is meant to ensure only one instance of the odometer is used
	 * throughout the code.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @return new or existing Odometer Object
	 * @throws OdometerExceptions
	 */
	public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
		if (odo != null) { // Return existing object
			return odo;
		} else { // create object and return it
			odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
			return odo;
		}
	}

	/**
	 * This class is meant to return the existing Odometer Object. It is meant to be
	 * used only if an odometer object has been created
	 * 
	 * @return error if no previous odometer exists
	 */
	public synchronized static Odometer getOdometer() throws OdometerExceptions {

		if (odo == null) {
			throw new OdometerExceptions("No previous Odometer exits.");

		}
		return odo;
	}

	/**
	 * This method is where the logic for the odometer runs. Distance traveled since
	 * last cycle is calculated independently for both wheels based on wheel
	 * rotation speed, tacho count and wheel radius Theta variation is also found
	 * through basic trigonometry. Theses values are then used in the update method
	 * defined in odometerData
	 * 
	 */
	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();

			leftMotorTachoCount = leftMotor.getTachoCount();
			rightMotorTachoCount = rightMotor.getTachoCount();

			// TODO Calculate new robot position based on tachometer counts

			// Calculate distance traveled for each wheel in metric units
			leftWheelDistance = Math.PI * WHEEL_RAD * (leftMotorTachoCount - lastLeftMotorTachoCount) / 180;
			rightWheelDistance = Math.PI * WHEEL_RAD * (rightMotorTachoCount - lastRightMotorTachoCount) / 180;

			// Save current tacho count to check with next cycle
			lastLeftMotorTachoCount = leftMotorTachoCount;
			lastRightMotorTachoCount = rightMotorTachoCount;

			// Find deltaDistance, deltaTheta
			deltaDistance = (leftWheelDistance + rightWheelDistance) / 2;
			deltaTheta = ((leftWheelDistance - rightWheelDistance) / TRACK) * 180 / Math.PI;

			// get data and add deltaTheta to Theta
			position = odo.getXYT();
			position[2] += deltaTheta;

			// Find Dx and Dy
			deltaX = deltaDistance * Math.sin(position[2] * Math.PI / 180);
			deltaY = deltaDistance * Math.cos(position[2] * Math.PI / 180);

			// TODO Update odometer values with new calculated values
			odo.update(deltaX, deltaY, deltaTheta);

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done
				}
			}
		}
	}

}
