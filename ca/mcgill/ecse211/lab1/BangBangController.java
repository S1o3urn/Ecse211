package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter; //offset from wall
  private final int bandwidth;	//error threshold
  private final int motorLow;	//slow speed
  private final int motorHigh;	//high speed
  private int distance;

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
    this.distance = distance;
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
    
    //ASSUMPTION: Always follows the wall on left side (may be opposite)
    
    //if distance is bigger or smaller than offset from wall by threshold
    //set motor speed accordingly to orientation
    
    //Robot too close to wall
    if(distance < bandCenter - bandwidth) {
    	WallFollowingLab.leftMotor.setSpeed(motorLow);
    	WallFollowingLab.rightMotor.setSpeed(motorHigh);
    }
    if(distance > bandCenter + bandwidth) {
    	WallFollowingLab.rightMotor.setSpeed(motorLow);
    	WallFollowingLab.leftMotor.setSpeed(motorHigh);
    }
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
