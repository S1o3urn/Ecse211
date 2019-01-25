package ca.mcgill.ecse211.lab2;

import lejos.hardware.*;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.utility.Delay;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  EV3ColorSensor colourSensor;
  private int xCounter = 0;
  private int yCounter = 0;
  private double error = 0.2;
  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();
    //Create a colour sensor and attach to a port
    colourSensor = new EV3ColorSensor(SensorPort.S4);

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
      
      //Display current X and Y as if starting from initial position
      LCD.drawInt(xCounter, 0, 4);
      LCD.drawInt(yCounter, 0, 5);
      
      // TODO Trigger correction (When do I have information to correct?)
      //Check if a black line exists
      if(colourSensor.getColorIDMode().equals("1"));
      Delay.msDelay(100);
      Sound.beep();
      
      // TODO Calculate new (accurate) robot position
      
      
      // TODO Update odometer with new calculated (and more accurate) vales

      odometer.setXYT(0.3, 19.23, 5.0);

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
