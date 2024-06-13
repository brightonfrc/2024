package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;




public class Motors extends SubsystemBase {
  public static CANSparkMax frontLeftMove;
  public static CANSparkMax frontLeftTurn;
  public static CANSparkMax frontRightMove;
  public static CANSparkMax frontRightTurn;
  public static CANSparkMax backLeftMove;
  public static CANSparkMax backLeftTurn;
  public static CANSparkMax backRightMove;
  public static CANSparkMax backRightTurn;
  
  /**
   * This creates the Motors subsystem to manage what power the motors are set at. 
   * This only works for a swerve drive.
   * @param FrontLeftMove the Front Left Movement Motor
   * @param FrontLeftTurn the Front Left Turning Motor
   * @param FrontRightMove the Front Right Movement Motor
   * @param FrontRightTurn the Front Right Turning Motor
   * @param BackLeftMove the Back Left Movement Motor
   * @param BackLeftTurn the Back Left Turning Motor
   * @param BackRightMove the Back Right Movement Motor
   * @param BackRightTurn the Back Right Turning Motor
   */
  public Motors(CANSparkMax FrontLeftMove, CANSparkMax FrontLeftTurn, CANSparkMax FrontRightMove, CANSparkMax FrontRightTurn, CANSparkMax BackLeftMove, CANSparkMax BackLeftTurn, CANSparkMax BackRightMove, CANSparkMax BackRightTurn) {
    frontLeftMove=FrontLeftMove;
    frontLeftTurn=FrontLeftTurn;
    frontRightMove=FrontRightMove;
    frontRightTurn=FrontRightTurn;
    backLeftMove=BackLeftMove;
    backLeftTurn=BackLeftTurn;
    backRightMove=BackRightMove;
    backRightTurn=BackRightTurn;

    //setting all move motors to follow the Front Left motors
    frontRightMove.follow(FrontLeftMove);
    backLeftMove.follow(FrontLeftMove);
  }
  /**
   * This sets the move motors to go at a certain power
   * @param PIDoutput The power that you want to set the motors at, ideally controller by a 
   * PID controller. Only takes from range -1 to 1. 
   */
  public void setMoveMotors(double PIDoutput){
    frontLeftMove.set(PIDoutput);
    //backRightMotor is inverted
    backRightMove.set(PIDoutput);
  }

  public enum TurnMotor {
    FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
  }
  /**
   * This sets the turn motors to go at a certain power
   * @param PIDOutput The power that you want to set the motors at, ideally controller by a 
   * PID controller. Only takes from range -1 to 1. 
   * @param motor The Turn motor which you would like to set. 
   */
  public void setTurnMotors(double PIDOutput, TurnMotor motor){
    switch (motor)
    {  
      case FRONT_LEFT:
        frontLeftTurn.set(PIDOutput);
      case FRONT_RIGHT:  
        frontRightTurn.set(PIDOutput);
      case BACK_LEFT:
        backLeftTurn.set(PIDOutput);
      case BACK_RIGHT:
      backRightTurn.set(PIDOutput);
    }
  }
}
