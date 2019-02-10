package ca.mcgill.ecse211.odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class implements the odometer functionality.
 * @author tianh
 *
 */
public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null;

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  
  private double deltaL;
  private double deltaR;
  private int oldLeftMotorTachoCount;
  private int oldRightMotorTachoCount;
  
  private double Theta;

  private final double TRACK;
  private final double WHEEL_RAD;

  private static final long ODOMETER_PERIOD = 25; // in MS

  /**
   * The default constructor. 
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData();
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset odometer values
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;
    this.Theta = 0;

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;

  }

  /**
   * Prevents multiple odometer instances.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) {
      return odo;
    } else {
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * Returns the existing Odometer. Used only if an
   * odometer has been created.
   * 
   * @return error if no odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * This method implements the logic behind the odometer.
   */
  // run method (required for Thread)
  public void run() {
    long updateStart;
    long updateEnd;

    //Taken from lab 2
    while (true) {
      updateStart = System.currentTimeMillis();

      leftMotorTachoCount = leftMotor.getTachoCount();
      rightMotorTachoCount = rightMotor.getTachoCount();
      
      double dX;
      double dY;
      double dTheta;
      double dDisplacement;

      deltaL = Math.PI * WHEEL_RAD * (leftMotorTachoCount - oldLeftMotorTachoCount) / 180;
      deltaR = Math.PI * WHEEL_RAD * (rightMotorTachoCount - oldRightMotorTachoCount) / 180;
      
      oldLeftMotorTachoCount = leftMotorTachoCount;
      oldRightMotorTachoCount = rightMotorTachoCount;
      dDisplacement = 0.5 * (deltaL+deltaR);
      dTheta = (deltaL-deltaR)/TRACK;
      Theta += dTheta;

      dX = dDisplacement * Math.sin(Theta);
      dY = dDisplacement * Math.cos(Theta);

      odo.update(dX, dY, dTheta * 180 / Math.PI);

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
