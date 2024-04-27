// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Status2Subsystem;

public class SetStatus2 extends Command {
  private Status2Subsystem debugSubsystem;
  private long startTimeMillis;
  private String statusMessage;
  private long durationMillis;

  /** Creates a new SetDebugStatus. */
  public SetStatus2(Status2Subsystem debugSubsystem, String statusMessage, long durationMillis) {
    this.debugSubsystem = debugSubsystem;
    this.statusMessage = statusMessage;
    this.durationMillis = durationMillis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(debugSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startTimeMillis = System.currentTimeMillis(); 
    this.debugSubsystem.setStatus(statusMessage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.debugSubsystem.clearStatus();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    long elapsedTimeMillis = System.currentTimeMillis() - startTimeMillis;
    return (elapsedTimeMillis >= durationMillis);
  }
}
