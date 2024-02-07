package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;

public class Encoders extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static Encoder FrontLeftMove;
  public static Encoder FrontLeftTurn;
  public static Encoder FrontRightMove;
  public static Encoder FrontRightTurn;
  public static Encoder BackLeftMove;
  public static Encoder BackLeftTurn;
  public static Encoder BackRightMove;
  public static Encoder BackRightTurn;
  //for calculating the degrees turned
  public double motorRadius;
  public double distanceRotated;
  public double deltaRadians;
  //storing value of current bearing of each wheel
  public double frontLeftBearing;
  public double frontRightBearing;
  public double backLeftBearing;
  public double backRightBearing;
  
  public Encoders(Encoder frontLeftMove, Encoder frontLeftTurn, Encoder frontRightTurn, Encoder frontRightMove, Encoder backLeftMove, Encoder backLeftTurn, Encoder backRightMove, Encoder backRightTurn, double motorRadius, double DistancePerPulse) {
    FrontLeftMove=frontLeftMove;
    FrontLeftTurn=frontLeftTurn;
    FrontRightTurn=frontRightTurn;
    FrontRightMove=frontRightMove;
    BackLeftMove=backLeftMove;
    BackLeftTurn=backLeftTurn;
    BackRightMove=backRightMove;
    BackRightTurn=backRightTurn;
    FrontLeftMove.setDistancePerPulse(DistancePerPulse);
    FrontRightMove.setDistancePerPulse(DistancePerPulse);
    BackLeftMove.setDistancePerPulse(DistancePerPulse);
    BackRightMove.setDistancePerPulse(DistancePerPulse);
    FrontLeftTurn.setDistancePerPulse(DistancePerPulse);
    FrontRightTurn.setDistancePerPulse(DistancePerPulse);
    BackLeftTurn.setDistancePerPulse(DistancePerPulse);
    BackRightTurn.setDistancePerPulse(DistancePerPulse);
    this.motorRadius=motorRadius;
    frontLeftBearing=0;
    frontRightBearing=0;
    backLeftBearing=0;
    backRightBearing=0;
  }
  public double getDistanceMoved(int motorNum){
    switch (motorNum){
      case 1:
        //return frontLeft distance
        return FrontLeftMove.getDistance();
      case 2:
        //return frontRight distance
        return FrontRightMove.getDistance();
      case 3: 
        //return backLeft distance
        return BackLeftMove.getDistance();
      case 4:    
        //return backRight distance
        return BackRightMove.getDistance();
    }
    //this should never happen, but just in case
    return 0.0;
  }
  public enum TurnEncoder{
    FRONT_LEFT,FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
  }
  public double motorTurned(TurnEncoder encoder){
    switch (encoder){
      case FRONT_LEFT:
        //return frontLeft degrees turned
        distanceRotated=FrontLeftTurn.getDistance();
        //using radians
        deltaRadians= distanceRotated/motorRadius;
        //if deltaRadians is more than 2pi, I am resetting it down
        frontLeftBearing+=deltaRadians;
        frontLeftBearing=frontLeftBearing%(Math.PI*2);
        return frontLeftBearing;
      case FRONT_RIGHT:
        //return frontLeft degrees turned
        distanceRotated=FrontLeftTurn.getDistance();
        //using radians
        deltaRadians= distanceRotated/motorRadius;
        //if deltaRadians is more than 2pi, I am resetting it down
        frontRightBearing+=deltaRadians;
        frontRightBearing=frontRightBearing%(Math.PI*2);
        return frontRightBearing;
      case BACK_LEFT: 
        //return frontLeft degrees turned
        distanceRotated=FrontLeftTurn.getDistance();
        //using radians
        deltaRadians= distanceRotated/motorRadius;
        //if deltaRadians is more than 2pi, I am resetting it down
        backLeftBearing+=deltaRadians;
        backLeftBearing=backLeftBearing%(Math.PI*2);
        return backLeftBearing;
      case BACK_RIGHT:    
        //return frontLeft degrees turned
        distanceRotated=FrontLeftTurn.getDistance();
        //using radians
        deltaRadians= distanceRotated/motorRadius;
        //if deltaRadians is more than 2pi, I am resetting it down
        backRightBearing+=deltaRadians;
        backRightBearing=backRightBearing%(Math.PI*2);
        return backRightBearing;
    }
    //this should never happen, but just so the code is happy
    return 0.0;
  }
}

