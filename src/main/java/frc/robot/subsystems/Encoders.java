package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

public class Encoders extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public AbsoluteEncoder frontLeftMove;
  public AbsoluteEncoder frontLeftTurn;
  public AbsoluteEncoder frontRightMove;
  public AbsoluteEncoder frontRightTurn;
  public AbsoluteEncoder backLeftMove;
  public AbsoluteEncoder backLeftTurn;
  public AbsoluteEncoder backRightMove;
  public AbsoluteEncoder backRightTurn;
  //for calculating the distance moved
  public double frontLeftPosition;
  public double frontRightPosition;
  public double backLeftPosition;
  public double backRightPosition;
  public double distanceMoved;
  //for calculating the degrees turned
  public double distanceRotated;
  public double deltaRadians;
  //storing value of current bearing of each wheel
  public double frontLeftBearing;
  public double frontRightBearing;
  public double backLeftBearing;
  public double backRightBearing;
  
  /**
   * Creates an encoder subsystem to manage the encoders of the robot
   * This only works for swerve drives. If you use a different set of encoders for the swerve drive
   * you only need to modify the class of the encoder parameters
   * 
   * @param frontLeftMove the encoder of the Front Left Movement Encoder
   * @param frontLeftTurn the encoder of the Front Left Turning Encoder
   * @param frontRightTurn the encoder of the Front Right Turning Encoder
   * @param frontRightMove the encoder of the Front Right Movement Encoder
   * @param backLeftMove the encoder of the Back Left Movement Encoder
   * @param backLeftTurn the encoder of the Back Left Turning Encoder
   * @param backRightMove the encoder of he Back Right Movement Encoder
   * @param backRightTurn the encoder of the Back Right Turning Encoder
   * @param movementPerRotation the distance that the robot moves for every rotation of the Movement Encoder
   */
  public Encoders(AbsoluteEncoder frontLeftMove, AbsoluteEncoder frontLeftTurn, AbsoluteEncoder frontRightTurn, AbsoluteEncoder frontRightMove, AbsoluteEncoder backLeftMove, AbsoluteEncoder backLeftTurn, AbsoluteEncoder backRightMove, AbsoluteEncoder backRightTurn, double movementPerRotation) {
    this.frontLeftMove = frontLeftMove;
    this.frontLeftTurn = frontLeftTurn;
    this.frontRightTurn = frontRightTurn;
    this.frontRightMove = frontRightMove;
    this.backLeftMove = backLeftMove;
    this.backLeftTurn = backLeftTurn;
    this.backRightMove = backRightMove;
    this.backRightTurn = backRightTurn;

    //the movement motors use movement per rotation because 1 rotation for the moveement motor doesn't mean that 
    //the robot moves forward by precisely the circumference of the motor
    frontLeftMove.setPositionConversionFactor(movementPerRotation);
    frontRightMove.setPositionConversionFactor(movementPerRotation);
    backLeftMove.setPositionConversionFactor(movementPerRotation);
    backRightMove.setPositionConversionFactor(movementPerRotation);
    frontLeftBearing = 0;
    frontRightBearing = 0;
    backLeftBearing = 0;
    backRightBearing = 0;
    frontLeftPosition=0;
    frontRightPosition=0;
    backLeftPosition=0;
    backRightPosition=0;
  }
  
  /**
   * This gets the distanceMoved by each  since the last time this function was called
   * @param motorNum the number of the motor you want to check
   * @return the distance Moved since the last time this function was called
   */
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
  
  /**
   * This returns the currrent bearing of any one of the Turn Encoders in radians
   * The range is from 0 to 2 PI radians
   * @param encoder the TurnEncoder you would like to check
   * @return the bearing of the encoder from 0 to 2PI Radians
   */
  public double motorTurned(TurnEncoder encoder){
    switch (encoder){
      case FRONT_LEFT:
        //return frontLeft degrees turned
        distanceRotated = frontLeftTurn.getPosition();
        SmartDashboard.putNumber("FrontLeft Rotations",distanceRotated);
        //using radians
        frontLeftBearing =  distanceRotated*2*Math.PI;
        //if the is more than 2pi, I am resetting it down
        frontLeftBearing = frontLeftBearing%(Math.PI*2);
        return frontLeftBearing;
      case FRONT_RIGHT:
        //return frontRight degrees turned
        distanceRotated = frontRightTurn.getPosition();
        SmartDashboard.putNumber("FrontLeft Rotations",distanceRotated);
        //using radians
        frontRightBearing =  distanceRotated*2*Math.PI;
        //if the is more than 2pi, I am resetting it down
        frontRightBearing = frontRightBearing%(Math.PI*2);
        return frontRightBearing;
      case BACK_LEFT: 
        //return backLeft degrees turned
        distanceRotated = backLeftTurn.getPosition();
        SmartDashboard.putNumber("FrontLeft Rotations",distanceRotated);
        //using radians
        backLeftBearing =  distanceRotated*2*Math.PI;
        //if the is more than 2pi, I am resetting it down
        backLeftBearing = backLeftBearing%(Math.PI*2);
        return backLeftBearing;
      case BACK_RIGHT:    
        //return backRight degrees turned
        distanceRotated = backRightTurn.getPosition();
        SmartDashboard.putNumber("FrontLeft Rotations",distanceRotated);
        //using radians
        backRightBearing =  distanceRotated*2*Math.PI;
        //if the is more than 2pi, I am resetting it down
        backRightBearing = backRightBearing%(Math.PI*2);
        return backRightBearing;
    }
    //this should never happen, but just so the code is happy
    return 0.0;
  }
}