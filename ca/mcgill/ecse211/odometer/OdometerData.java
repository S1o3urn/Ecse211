package ca.mcgill.ecse211.odometer;

/**
 * This class stores and provides thread safe access to the odometer data.
 * Taken from previous labs
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class OdometerData {

  // Position parameters
  private volatile double x; // x-axis position
  private volatile double y; // y-axis position
  private volatile double theta; // Angle

  // Class control variables
  /**
   * Number of OdometerData objects instantiated so far.
   */
  private volatile static int numberOfIntances = 0;
  
  /**
   * Maximum number of OdometerData instances
   */
  private static final int MAX_INSTANCES = 1;

  // Thread control tools
  /**
   * Fair lock for concurrent writing
   */
  private static Lock lock = new ReentrantLock(true);
  
  /**
   * Indicates if a thread is trying to reset any position parameters
   */
  private volatile boolean isReseting = false;
  
  /**
   *  // Let other threads know that a reset operation is over.
   */
  private Condition doneReseting = lock.newCondition();

  private static OdometerData odoData = null;

  /**
   * The constructor.
   */
  protected OdometerData() {
    this.x = 0;
    this.y = 0;
    this.theta = 0;
  }

  /**
   * The overloaded factory constructor.
   * Prevents multiple instances of this class.
   * 
   * @return OdometerData object
   * @throws OdometerExceptions
   */
  public synchronized static OdometerData getOdometerData() throws OdometerExceptions {
	// Return existing object
	  if (odoData != null) {
      return odoData;
    } 
    
    // create object and return it
    else if (numberOfIntances < MAX_INSTANCES) {
      odoData = new OdometerData();
      numberOfIntances += 1;
      return odoData;
    } else {
      throw new OdometerExceptions("One instance maximal at once.");
    }

  }

  /**
   * Return the Odomometer data.
   * 
   * odoData[0] = x;
   * odoData[1] = y;
   * odoData[2] = theta;
   * 
   * @param position array to store position
   * @return odometer data
   */
  public double[] getXYT() {
    double[] position = new double[3];
    lock.lock();
    try {
      while (isReseting) {
        doneReseting.await(); // Lighter on the CPU
      }

      position[0] = x;
      position[1] = y;
      position[2] = theta;

    } catch (InterruptedException e) {
      // Print exception to screen
      e.printStackTrace();
    } finally {
      lock.unlock();
    }

    return position;

  }

  /**
   * Adds the difference in position values to current position values.
   * 
   * @param dx
   * @param dy
   * @param dtheta
   */
  public void update(double dx, double dy, double dtheta) {
    lock.lock();
    isReseting = true;
    try {
      x += dx;
      y += dy;
      // To fix issues when theta is 0 or 360 degrees
      theta = (theta + (360 + dtheta) % 360) % 360;
      isReseting = false;
      doneReseting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  // Mutators
  
  /**
   * Overrides position values for odometry correction.
   * 
   * @param x
   * @param y
   * @param theta
   */
  public void setXYT(double x, double y, double theta) {
    lock.lock();
    isReseting = true;
    try {
      this.x = x;
      this.y = y;
      this.theta = theta;
      isReseting = false;
      doneReseting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Modify the x value.
   * 
   * @param x
   */
  public void setX(double x) {
    lock.lock();
    isReseting = true;
    try {
      this.x = x;
      isReseting = false;
      doneReseting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Modify the y value.
   * 
   * @param y
   */
  public void setY(double y) {
    lock.lock();
    isReseting = true;
    try {
      this.y = y;
      isReseting = false;
      doneReseting.signalAll();
    } finally {
      lock.unlock();
    }
  }
  
  /**
   * Modify the theta value.
   * 
   * @param theta
   */
  public void setTheta(double theta) {
    lock.lock();
    isReseting = true;
    try {
      this.theta = theta;
      isReseting = false;
      doneReseting.signalAll();
    } finally {
      lock.unlock();
    }
  }

}
