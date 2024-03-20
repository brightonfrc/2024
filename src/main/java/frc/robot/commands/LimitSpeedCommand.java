// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.SwerveDriveCommandConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class LimitSpeedCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private boolean endCommand;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LimitSpeedCommand() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Command worked",true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SwerveDriveCommandConstants.limiterActive){
        //if limiter active is true, set it to false
        SwerveDriveCommandConstants.limiterActive=false;
    }
    else{
        SwerveDriveCommandConstants.limiterActive=true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand;
  }
}
