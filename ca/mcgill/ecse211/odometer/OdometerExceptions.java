package ca.mcgill.ecse211.odometer;
/**
 * This class handles errors.
 * 
 * @author tianh
 */
@SuppressWarnings("serial")
public class OdometerExceptions extends Exception {

  public OdometerExceptions(String Error) {
    super(Error);
  }

}