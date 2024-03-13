package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Motors;
import frc.robot.subsystems.Motors.TurnMotor;
import frc.robot.subsystems.Encoders;
import frc.robot.subsystems.Encoders.TurnEncoder;
import frc.robot.subsystems.Encoders.MoveEncoder;
import frc.robot.Constants.PIDConstants;

/** A command that makes the robot drive forwards a certain distance. */
public class DriveForwards extends Command {

  private final Motors m_motors;
  private final Encoders m_encoders;
  private final double m_targetDistance;
  private final double m_targetBearing;

  private PIDController robotMovementController;
  private double totalDistanceMoved;
  private double lastDistanceMoved;
  private boolean endCommand;

  private PIDController frontLeftBearingController;
  private PIDController frontRightBearingController;
  private PIDController backLeftBearingController;
  private PIDController backRightBearingController;
  private double currentBearing;
  


  /**
   * Creates a new DriveForwards command.
   *
   * @param motors The motors subsystem used by this command.
   * @param encoders The encoders subsystem used by this command.
   * @param targetDistance The distance in encoder ticks or other units to drive forwards.
   * @param targetBearing The bearing that the robot is supposed to move towards.
   */
  public DriveForwards(Motors motors, Encoders encoders, double targetDistance, double targetBearing) {
    m_motors = motors;
    m_encoders = encoders;
    m_targetDistance = targetDistance;
    m_targetBearing = targetBearing;
    addRequirements(m_motors, m_encoders);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset encoders if needed
    // m_encoders.FrontLeftMove.reset();
    // m_encoders.FrontRightMove.reset();
    // m_encoders.BackLeftMove.reset();
    // m_encoders.BackRightMove.reset();
    frontLeftBearingController= new PIDController(PIDConstants.kDrivetrainP, PIDConstants.kDrivetrainI, PIDConstants.kDrivetrainD);
    //allowing 1 degree of variation
    frontLeftBearingController.setTolerance(Math.PI/180);
    frontLeftBearingController.setSetpoint(m_targetBearing);
    frontLeftBearingController.enableContinuousInput(0, 2*Math.PI);

    frontRightBearingController= new PIDController(PIDConstants.kDrivetrainP, PIDConstants.kDrivetrainI, PIDConstants.kDrivetrainD);
    //allowing 1 degree of variation
    frontRightBearingController.setTolerance(Math.PI/180);
    frontRightBearingController.setSetpoint(m_targetBearing);
    frontRightBearingController.enableContinuousInput(0, 2*Math.PI);

    backLeftBearingController= new PIDController(PIDConstants.kDrivetrainP, PIDConstants.kDrivetrainI, PIDConstants.kDrivetrainD);
    //allowing 1 degree of variation
    backLeftBearingController.setTolerance(Math.PI/180);
    backLeftBearingController.setSetpoint(m_targetBearing);
    backLeftBearingController.enableContinuousInput(0, 2*Math.PI);

    backRightBearingController= new PIDController(PIDConstants.kDrivetrainP, PIDConstants.kDrivetrainI, PIDConstants.kDrivetrainD);
    //allowing 1 degree of variation
    backRightBearingController.setTolerance(Math.PI/180);
    if (m_targetBearing<Math.PI){
      //preventing the setpoints from being more than 2PI
      backRightBearingController.setSetpoint(m_targetBearing+Math.PI);
    }
    else{
      backRightBearingController.setSetpoint(m_targetBearing-Math.PI);
    }
    backRightBearingController.enableContinuousInput(0, 2*Math.PI);

    robotMovementController= new PIDController(PIDConstants.kRobotMoveP, PIDConstants.kRobotMoveI, PIDConstants.kRobotMoveD);
    robotMovementController.setSetpoint(m_targetDistance);
    //saying that 1cm of error is acceptable
    robotMovementController.setTolerance(0.01);
    totalDistanceMoved=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentBearing=m_encoders.motorTurned(TurnEncoder.FRONT_LEFT);
    SmartDashboard.putNumber("FL Bearing", currentBearing);
    m_motors.setTurnMotors(frontLeftBearingController.calculate(currentBearing), TurnMotor.FRONT_LEFT);

    currentBearing=m_encoders.motorTurned(TurnEncoder.FRONT_RIGHT);
    SmartDashboard.putNumber("FR Bearing", currentBearing);
    m_motors.setTurnMotors(frontRightBearingController.calculate(currentBearing), TurnMotor.FRONT_RIGHT);

    currentBearing=m_encoders.motorTurned(TurnEncoder.BACK_LEFT);
    SmartDashboard.putNumber("BL Bearing", currentBearing);
    m_motors.setTurnMotors(backLeftBearingController.calculate(currentBearing), TurnMotor.BACK_LEFT);

    currentBearing=m_encoders.motorTurned(TurnEncoder.BACK_RIGHT);
    SmartDashboard.putNumber("BR Bearing", currentBearing);
    m_motors.setTurnMotors(backRightBearingController.calculate(currentBearing), TurnMotor.BACK_RIGHT);

    if (frontLeftBearingController.atSetpoint()&&
    frontRightBearingController.atSetpoint()&&
    backLeftBearingController.atSetpoint()&&
    backRightBearingController.atSetpoint()){
      //preventing the code from running until the turn motors are ready  
      // getting the average distance moved
      lastDistanceMoved+=m_encoders.getDistanceMoved(MoveEncoder.FRONT_LEFT);
      lastDistanceMoved+=m_encoders.getDistanceMoved(MoveEncoder.FRONT_RIGHT);
      lastDistanceMoved+=m_encoders.getDistanceMoved(MoveEncoder.BACK_LEFT);
      lastDistanceMoved+=m_encoders.getDistanceMoved(MoveEncoder.BACK_RIGHT);
      lastDistanceMoved=lastDistanceMoved/4;
      totalDistanceMoved+=lastDistanceMoved;
      SmartDashboard.putNumber("currentDistance",totalDistanceMoved);
      m_motors.setMoveMotors(robotMovementController.calculate(totalDistanceMoved));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the motors
    m_motors.setMoveMotors(0);
    m_motors.setTurnMotors(0, TurnMotor.FRONT_LEFT);
    m_motors.setTurnMotors(0, TurnMotor.FRONT_RIGHT);
    m_motors.setTurnMotors(0, TurnMotor.BACK_LEFT);
    m_motors.setTurnMotors(0, TurnMotor.BACK_RIGHT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return robotMovementController.atSetpoint();
  }
}
