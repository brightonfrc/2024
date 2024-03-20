// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import frc.robot.Constants.SwerveDriveCommandConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.math.controller.PIDController;

/** An example command that uses an example subsystem. */
public class SwerveDrive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem drive;
  private final CommandPS4Controller controller;

  private PIDController bearingController;
  private double maxRotationRate;
  private double kP;
  private double kI;
  private double kD;
  private double bearingTolerance;

  /**
   * Creates a new SwerveDrive that rotates the chassis to the bearing set by the right joystick.
   *
   * @param m_robotDrive The drive subsystem used by this command.
   */
  public SwerveDrive(DriveSubsystem m_robotDrive, CommandPS4Controller controller) {
    drive = m_robotDrive;
    this.controller=controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP=SwerveDriveCommandConstants.kP;
    kI=SwerveDriveCommandConstants.kI;
    kD=SwerveDriveCommandConstants.kD;
    bearingTolerance=SwerveDriveCommandConstants.bearingTolerance;

    maxRotationRate=SwerveDriveCommandConstants.maxRotationRate;
    bearingController= new PIDController(kP, kI, kD);
    bearingController.setTolerance(Math.PI/180*bearingTolerance);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
