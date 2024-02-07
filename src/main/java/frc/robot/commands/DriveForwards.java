package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Motors;
import frc.robot.subsystems.Encoders;

/** A command that makes the robot drive forwards a certain distance. */
public class DriveForwards extends Command {

  private final Motors m_motors;
  private final Encoders m_encoders;
  private final double m_targetDistance;
  private final double m_speed;

  /**
   * Creates a new DriveForwards command.
   *
   * @param motors The motors subsystem used by this command.
   * @param encoders The encoders subsystem used by this command.
   * @param targetDistance The distance in encoder ticks or other units to drive forwards.
   * @param speed The speed at which the robot should drive forwards.
   */
  public DriveForwards(Motors motors, Encoders encoders, double targetDistance, double speed) {
    m_motors = motors;
    m_encoders = encoders;
    m_targetDistance = targetDistance;
    m_speed = speed;
    addRequirements(m_motors);
    addRequirements(m_encoders);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset encoders if needed
    // m_encoders.FrontLeftMove.reset();
    // m_encoders.FrontRightMove.reset();
    // m_encoders.BackLeftMove.reset();
    // m_encoders.BackRightMove.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drive forwards at the specified speed
    m_motors.setMoveMotors(m_speed);
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
    // Calculate the current distance traveled by any motor
    double currentDistance = m_encoders.getDistanceMoved(1); // Using front left encoder for example
    return currentDistance >= m_targetDistance;
  }
}
