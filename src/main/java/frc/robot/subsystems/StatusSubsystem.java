// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StatusSubsystem extends SubsystemBase {
  /** Creates a new DebugSubsystem. */
  public StatusSubsystem() {}
  
  /**
   * Set the shuffleboard status to be shown to the driver
   * @param status human readable status
   */
  public void setStatus(String status) {
    SmartDashboard.putString("Debug/Status", status);
  }

  /**
   * Set the shuffleboard status to an empty string
   */
  public void clearStatus() {
    setStatus("");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
