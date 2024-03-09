// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Motors;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.Constants.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class MoveToAmp extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Motors motors;
  private final DistanceSensor distanceSensor;

  private PIDController robotMovementController;

  private double kPRobot;
  private double kIRobot;
  private double kDRobot;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveToAmp(Motors motors, DistanceSensor distanceSensor) {
    this.motors=motors;
    this.distanceSensor= distanceSensor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motors,distanceSensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kPRobot=PIDConstants.kPRobotMove;
    kIRobot=PIDConstants.kIRobotMove;
    kDRobot=PIDConstants.kDRobotMove;

    robotMovementController=new PIDController(kPRobot, kIRobot, kDRobot);
    //allowing for a tolerance of 1 cm
    robotMovementController.setTolerance(1);
    //I am setting the setpoint to 30 because that is the minimum detection distance of the sensor
    robotMovementController.setSetpoint(30);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Moving to Amp", true);
    distanceSensor.upddateDistances();
    SmartDashboard.putNumber("current Distance", distanceSensor.getDistance());
    motors.setMoveMotors(robotMovementController.calculate(distanceSensor.getDistance()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return robotMovementController.atSetpoint();
  }
}
