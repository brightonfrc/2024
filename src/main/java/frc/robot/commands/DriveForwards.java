package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Motors;
import frc.robot.subsystems.Encoders;
import frc.robot.subsystems.Encoders.MoveEncoder;
import frc.robot.Constants.PIDConstants;

/** A command that makes the robot drive forwards a certain distance. */
public class DriveForwards extends Command {

  private final Motors m_motors;
  private final Encoders m_encoders;
  private final double m_targetDistance;

  private PIDController robotMovementController;
  private double totalDistanceMoved;
  private double lastDistanceMoved;
  private boolean endCommand;


  /**
   * Creates a new DriveForwards command.
   *
   * @param motors The motors subsystem used by this command.
   * @param encoders The encoders subsystem used by this command.
   * @param targetDistance The distance in encoder ticks or other units to drive forwards.
   */
  public DriveForwards(Motors motors, Encoders encoders, double targetDistance) {
    m_motors = motors;
    m_encoders = encoders;
    m_targetDistance = targetDistance;
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

    robotMovementController= new PIDController(PIDConstants.kRobotMoveP, PIDConstants.kRobotMoveI, PIDConstants.kRobotMoveD);
    robotMovementController.setSetpoint(m_targetDistance);
    //saying that 1cm of error is acceptable
    robotMovementController.setTolerance(0.01);
    totalDistanceMoved=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(robotMovementController.atSetpoint()){
      endCommand=true;
    }
    // getting the average distance moved
    lastDistanceMoved+=m_encoders.getDistanceMoved(MoveEncoder.FRONT_LEFT);
    lastDistanceMoved+=m_encoders.getDistanceMoved(MoveEncoder.FRONT_RIGHT);
    lastDistanceMoved+=m_encoders.getDistanceMoved(MoveEncoder.BACK_LEFT);
    lastDistanceMoved+=m_encoders.getDistanceMoved(MoveEncoder.BACK_RIGHT);
    lastDistanceMoved=lastDistanceMoved/4;
    totalDistanceMoved+=lastDistanceMoved;
    m_motors.setMoveMotors(robotMovementController.calculate(totalDistanceMoved));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the motors
    m_motors.setMoveMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand;
  }
}
