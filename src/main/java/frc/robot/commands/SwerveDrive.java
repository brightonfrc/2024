// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Gyroscope;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveDriveCommandConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class SwerveDrive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem drive;
  private final Gyroscope gyro;
  private final CommandPS4Controller controller;

  private PIDController bearingController;
  private double maxRotationRate;
  private double kP;
  private double kI;
  private double kD;
  private double bearingTolerance;
  private double bearingGoal;
  private double previousBearingGoal;

  /**
   * Creates a new SwerveDrive that rotates the chassis to the bearing set by the right joystick.
   *
   * @param m_robotDrive The drive subsystem used by this command.
   */
  public SwerveDrive(DriveSubsystem m_robotDrive,Gyroscope gyroscope, CommandPS4Controller controller) {
    drive = m_robotDrive;
    gyro = gyroscope;
    this.controller=controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive,gyroscope);
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
    bearingController.enableContinuousInput(0, 2*Math.PI);
    bearingController.setTolerance(Math.PI/180*bearingTolerance);
    previousBearingGoal=0;
    bearingController.setSetpoint(previousBearingGoal);
    SmartDashboard.putBoolean("SwerveDriveActive", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    bearingGoal= Math.atan2(controller.getRightX(),controller.getRightY());
    if (bearingGoal<0){
      //setting it to range 0 to 2pi
      bearingGoal+=Math.PI*2;
    }
    SmartDashboard.putNumber("JoystickRadians", bearingGoal);
    if (Math.abs(bearingGoal-previousBearingGoal)>Math.PI/180){
      //if the difference in bearing is more than 1 degree
      previousBearingGoal=bearingGoal;
      bearingController.reset();
      bearingController.setSetpoint(previousBearingGoal);        
    }
    SmartDashboard.putNumber("setpoint", bearingController.getSetpoint());
    SmartDashboard.putNumber("bearing", gyro.getBearing());
    SmartDashboard.putNumber("PID output", bearingController.calculate(gyro.getBearing()));
    drive.drive(MathUtil.applyDeadband(controller.getLeftY(), OIConstants.kDriveDeadband) * 0.3, MathUtil.applyDeadband(controller.getLeftX(), OIConstants.kDriveDeadband) * 0.3, bearingController.calculate(gyro.getBearing())*maxRotationRate, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
