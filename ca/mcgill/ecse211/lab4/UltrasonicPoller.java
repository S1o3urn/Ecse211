package ca.mcgill.ecse211.lab4;

import lejos.robotics.SampleProvider;

/**
 * Taken from the wall follower lab.
 * Assumptions: us.fetchSample, and cont.processUSData methods operate in about 20mS
 * 				thread sleeps for 50 mS at the end of each loop
 * 
 * Thus, one cycle through the loop is approximately 70 mS.
 * This corresponds to a sampling rate of 1/70mS
 * or about 14 Hz.
 */
public class UltrasonicPoller extends Thread {
	
  private SampleProvider us;
  private UltrasonicController cont;
  private float[] usData;

  /**
   * The constructor.
   * @param us
   * @param usData
   * @param cont
   */
  public UltrasonicPoller(SampleProvider us, float[] usData, UltrasonicController cont) {
    this.us = us;
    this.cont = cont;
    this.usData = usData;
  }

/**
 * Sensor returns float
 */
  public void run() {
    int distance;
    while (true) {
      us.fetchSample(usData, 0); // fetch data
      distance = (int) (usData[0] * 100.0); // cast to int and store in distance variable
      cont.processUSData(distance);	// Action on distance data
      try {
        Thread.sleep(50);
      } catch (Exception e) {
      } // Poor man's timed sampling
    }
  }

}