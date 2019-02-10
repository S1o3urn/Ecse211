package ca.mcgill.ecse211.lab4;

import java.text.DecimalFormat;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.lcd.TextLCD;

/**
 * Displays critical robot positioning and sensor acquired values.
 * 
 * @author tianh
 */
public class Display implements Runnable {

  private Odometer odometer;
  private TextLCD screen;
  private double[] position;
  private final long DISPLAY_PERIOD = 25;
  private long timeout = Long.MAX_VALUE;

/**
 * The constructor.
 * @param lcd
 * @throws OdometerExceptions
 */
  public Display(TextLCD lcd) throws OdometerExceptions {
    odometer = Odometer.getOdometer();
    this.screen = lcd;
  }

  /**
   * The overloaded class constructor.
   * 
   * @param odoData
   * @throws OdometerExceptions 
   */
  public Display(TextLCD lcd, long timeout) throws OdometerExceptions {
    odometer = Odometer.getOdometer();
    this.timeout = timeout;
    this.screen = lcd;
  }

  // Taken from previous labs
  public void run() {
    
    screen.clear();
    
    long updateStart;
    long updateEnd;

    long tStart = System.currentTimeMillis();
    
    do {
      updateStart = System.currentTimeMillis();

      // Fetch position information from odometer
      position = odometer.getXYT();
      
      // Format and display robot position
      DecimalFormat numberFormat = new DecimalFormat("######0.00");
      screen.drawString("X: " + numberFormat.format(position[0]), 0, 0);
      screen.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
      screen.drawString("T: " + numberFormat.format(position[2]), 0, 2);
      
      // Ensure that the data is updated only once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < DISPLAY_PERIOD) {
        try {
          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    } while ((updateEnd - tStart) <= timeout);

  }

}