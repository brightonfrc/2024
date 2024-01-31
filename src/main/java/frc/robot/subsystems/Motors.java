package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;




public class Motors extends SubsystemBase {
  public static Talon frontLeftMove;
  public static Talon frontLeftTurn;
  public static Talon frontRightMove;
  public static Talon frontRightTurn;
  public static Talon backLeftMove;
  public static Talon backLeftTurn;
  public static Talon backRightMove;
  public static Talon backRightTurn;

  public Motors(Talon FrontLeftMove, Talon FrontLeftTurn, Talon FrontRightMove, Talon FrontRightTurn, Talon BackLeftMove, Talon BackLeftTurn, Talon BackRightMove, Talon BackRightTurn) {
    frontLeftMove=FrontLeftMove;
    frontLeftTurn=FrontLeftTurn;
    frontRightMove=FrontRightMove;
    frontRightTurn=FrontRightTurn;
    backLeftMove=BackLeftMove;
    backLeftTurn=BackLeftTurn;
    backRightMove=BackRightMove;
    backRightTurn=BackRightTurn;
  }
  public void setMoveMotors(double PIDoutput){
    frontLeftMove.set(PIDoutput);
    frontRightMove.set(PIDoutput);
    backLeftMove.set(PIDoutput);
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
