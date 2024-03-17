// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkMax motor;
  /**
   * Creates a new Intake.
   * @param motor The NEO motor powering the intake.
   */
  public Intake(CANSparkMax motor) {
    this.motor = motor;
    SmartDashboard.putNumber("Intake/Speed", 0);
  }

  /**
   * Runs the motor at the given speed.
   * @param speed The speed of the motor, from 0 to 1.
   */
  public void runMotor(double speed) {
    motor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: Add actions
    runMotor(SmartDashboard.getNumber("Intake/Speed", 0.1));
    SmartDashboard.putNumber("Intake/Current", motor.getOutputCurrent());
  }
}
