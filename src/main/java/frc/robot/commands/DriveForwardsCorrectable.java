package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Motors;
import frc.robot.subsystems.Encoders;

/** A command that makes the robot drive forwards a certain distance while maintaining a straight path. */
public class DriveForwardsCorrectable extends Command {

    private final Motors m_motors;
    private final Encoders m_encoders;
    private final double m_targetDistance;
    private final double m_speed;
    private final double m_kP = 0.1; // Proportional constant for correction

    private double m_initialLeftDistance;
    //private double m_initialRightDistance;

    /**
     * Creates a new DriveForwards command.
     *
     * @param motors The motors subsystem used by this command.
     * @param encoders The encoders subsystem used by this command.
     * @param targetDistance The distance in encoder ticks or other units to drive forwards.
     * @param speed The speed at which the robot should drive forwards.
     */
    public DriveForwardsCorrectable(Motors motors, Encoders encoders, double targetDistance, double speed) {
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
        // Record initial encoder distances
        m_initialLeftDistance = m_encoders.getDistanceMoved(1); // Using front left encoder for example
        //m_initialRightDistance = m_encoders.getDistanceMoved(2); // Using front right encoder for example
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Get current encoder distances
        double currentLeftDistance = m_encoders.getDistanceMoved(1); // Using front left encoder for example
        double currentRightDistance = m_encoders.getDistanceMoved(2); // Using front right encoder for example

        // Calculate error in distances
        double error = currentLeftDistance - currentRightDistance;

        // Calculate correction
        double correction = error * m_kP;

        // Adjust speed for left and right sides
        double leftSpeed = m_speed - correction;
        double rightSpeed = m_speed + correction;

        // Set motor speeds
        m_motors.setMoveMotors(leftSpeed); // Set left motor speed
        m_motors.setMoveMotors(rightSpeed); // Set right motor speed
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      // Stop the motors by setting their speeds to 0
      m_motors.setMoveMotors(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Calculate the average distance traveled
        double currentLeftDistance = m_encoders.getDistanceMoved(1); // Using front left encoder for example
        double currentRightDistance = m_encoders.getDistanceMoved(2); // Using front right encoder for example
        double averageDistance = (currentLeftDistance + currentRightDistance) / 2.0;

        // Check if the average distance exceeds the target distance
        return Math.abs(averageDistance - m_initialLeftDistance) >= m_targetDistance;
    }
}
