// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Motors;
import frc.robot.subsystems.Motors.TurnMotor;
import frc.robot.subsystems.Encoders;
import frc.robot.subsystems.Encoders.TurnEncoder;
import frc.robot.subsystems.Gyroscope;
import frc.robot.Constants.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoTurnDegrees extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Motors motors;
  private final Encoders encoders;
  private final Gyroscope gyro;
  private double angleGoal;

  private double kP;
  private double kI;
  private double kD;
  private PIDController frontLeftBearingController;
  private PIDController frontRightBearingController;
  private PIDController backLeftBearingController;
  private PIDController backRightBearingController;

  private double kPRobotTurn;
  private double kIRobotTurn;
  private double kDRobotTurn;
  private PIDController robotBearingController;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoTurnDegrees(Motors motors, Encoders encoders, Gyroscope gyro, double angleGoal) {
    this.motors=motors;
    this.encoders=encoders;
    this.gyro=gyro;
    this.angleGoal=angleGoal;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motors,encoders,gyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP=PIDConstants.kDrivetrainP;
    kI=PIDConstants.kDrivetrainI;
    kD=PIDConstants.kDrivetrainD;

    kPRobotTurn=PIDConstants.kRobotTurnP;
    kIRobotTurn=PIDConstants.kRobotTurnI;
    kDRobotTurn=PIDConstants.kRobotTurnD;

    frontLeftBearingController.setP(kP);
    frontLeftBearingController.setI(kI);
    frontLeftBearingController.setD(kD);
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
