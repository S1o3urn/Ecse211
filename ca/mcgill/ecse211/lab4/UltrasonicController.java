package ca.mcgill.ecse211.lab4;

/**
 * Taken from previous labs
 * @author tianh
 *
 */
public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}