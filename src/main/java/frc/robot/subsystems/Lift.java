// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

public class Lift extends SubsystemBase {
  /** Creates a new Lift. */
  private final CANSparkMax motor = new CANSparkMax(0,MotorType.kBrushless);
  private final AbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  
  
  public Lift() {
    motor.setIdleMode(IdleMode.kBrake);
  }

  public void setClimbMotor(double speed) {
    motor.set(speed);
  }

  public double getRotations() {
    return encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}

