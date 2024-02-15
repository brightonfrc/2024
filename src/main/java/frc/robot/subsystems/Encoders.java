package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;

public class Encoders extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public RelativeEncoder frontLeftMove;
  public RelativeEncoder frontLeftTurn;
  public RelativeEncoder frontRightMove;
  public RelativeEncoder frontRightTurn;
  public RelativeEncoder backLeftMove;
  public RelativeEncoder backLeftTurn;
  public RelativeEncoder backRightMove;
  public RelativeEncoder backRightTurn;
  //for calculating the distance moved
  public double frontLeftPosition;
  public double frontRightPosition;
  public double backLeftPosition;
  public double backRightPosition;
  public double distanceMoved;
  //for calculating the degrees turned
  public double motorRadius;
  public double distanceRotated;
  public double deltaRadians;
  //storing value of current bearing of each wheel
  public double frontLeftBearing;
  public double frontRightBearing;
  public double backLeftBearing;
  public double backRightBearing;
  
  public Encoders(RelativeEncoder frontLeftMove, RelativeEncoder frontLeftTurn, RelativeEncoder frontRightTurn, RelativeEncoder frontRightMove, RelativeEncoder backLeftMove, RelativeEncoder backLeftTurn, RelativeEncoder backRightMove, RelativeEncoder backRightTurn, double motorRadius, double distancePerRotation, double movementPerRotation) {
    this.frontLeftMove = frontLeftMove;
    this.frontLeftTurn = frontLeftTurn;
    this.frontRightTurn = frontRightTurn;
    this.frontRightMove = frontRightMove;
    this.backLeftMove = backLeftMove;
    this.backLeftTurn = backLeftTurn;
    this.backRightMove = backRightMove;
    this.backRightTurn = backRightTurn;


    // Reset encoder positions
    frontLeftMove.setPosition(0);
    frontRightMove.setPosition(0);
    backLeftMove.setPosition(0);
    backRightMove.setPosition(0);
    frontLeftTurn.setPosition(0);
    frontRightTurn.setPosition(0);
    backLeftTurn.setPosition(0);
    backRightTurn.setPosition(0);

    //the movement motors use movement per rotation because 1 rotation for the moveement motor doesn't mean that 
    //the robot moves forward by precisely the circumference of the motor
    frontLeftMove.setPositionConversionFactor(movementPerRotation);
    frontRightMove.setPositionConversionFactor(movementPerRotation);
    backLeftMove.setPositionConversionFactor(movementPerRotation);
    backRightMove.setPositionConversionFactor(movementPerRotation);
    frontLeftTurn.setPositionConversionFactor(distancePerRotation);
    frontRightTurn.setPositionConversionFactor(distancePerRotation);
    backLeftTurn.setPositionConversionFactor(distancePerRotation);
    backRightTurn.setPositionConversionFactor(distancePerRotation);
    this.motorRadius = motorRadius;
    frontLeftBearing = 0;
    frontRightBearing = 0;
    backLeftBearing = 0;
    backRightBearing = 0;
    frontLeftPosition=0;
    frontRightPosition=0;
    backLeftPosition=0;
    backRightPosition=0;
  }
  public double getDistanceMoved(int motorNum){
    switch (motorNum){
      case 1:
        //return frontLeft distance
        distanceMoved= frontLeftMove.getPosition()-frontLeftPosition;
        frontLeftPosition=frontLeftMove.getPosition();
        return distanceMoved;
      case 2:
        //return frontRight distance
        distanceMoved= frontRightMove.getPosition()-frontRightPosition;
        frontRightPosition=frontRightMove.getPosition();
        return distanceMoved;
      case 3: 
        //return backLeft distance
        distanceMoved= backLeftMove.getPosition()-backLeftPosition;
        backLeftPosition=backLeftMove.getPosition();
        return distanceMoved;
      case 4:    
        //return backRight distance
        distanceMoved= backRightMove.getPosition()-backRightPosition;
        backRightPosition=backRightMove.getPosition();
        return distanceMoved;
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
        distanceRotated = frontLeftTurn.getPosition();
        //using radians
        deltaRadians =  distanceRotated/motorRadius;
        //if deltaRadians is more than 2pi, I am resetting it down
        frontLeftBearing += deltaRadians;
        frontLeftBearing = frontLeftBearing%(Math.PI*2);
        return frontLeftBearing;
      case FRONT_RIGHT:
        //return frontLeft degrees turned
        distanceRotated = frontLeftTurn.getPosition();
        //using radians
        deltaRadians =  distanceRotated/motorRadius;
        //if deltaRadians is more than 2pi, I am resetting it down
        frontRightBearing += deltaRadians;
        frontRightBearing = frontRightBearing%(Math.PI*2);
        return frontRightBearing;
      case BACK_LEFT: 
        //return frontLeft degrees turned
        distanceRotated = frontLeftTurn.getPosition();
        //using radians
        deltaRadians =  distanceRotated/motorRadius;
        //if deltaRadians is more than 2pi, I am resetting it down
        backLeftBearing += deltaRadians;
        backLeftBearing = backLeftBearing%(Math.PI*2);
        return backLeftBearing;
      case BACK_RIGHT:    
        //return frontLeft degrees turned
        distanceRotated = frontLeftTurn.getPosition();
        //using radians
        deltaRadians = distanceRotated/motorRadius;
        //if deltaRadians is more than 2pi, I am resetting it down
        backRightBearing += deltaRadians;
        backRightBearing = backRightBearing%(Math.PI*2);
        return backRightBearing;
    }
    //this should never happen, but just so the code is happy
    return 0.0;
  }
}

