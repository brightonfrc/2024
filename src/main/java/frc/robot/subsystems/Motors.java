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
  public void setMoveMotors(double PIDoutput){
    frontLeftMove.set(PIDoutput);
    //backRightMotor is inverted
    backRightMove.set(PIDoutput);
  }

  public enum TurnMotor {
    FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
  }

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
