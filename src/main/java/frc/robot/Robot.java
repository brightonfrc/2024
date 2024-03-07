// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.subsystems.Motors;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Ports;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private final CANSparkMax frontLeftMove= new CANSparkMax(Ports.kDriveFrontLeftMove,MotorType.kBrushless);
  private final CANSparkMax frontLeftTurn= new CANSparkMax(Ports.kDriveFrontLeftTurn,MotorType.kBrushless);
  private final CANSparkMax frontRightMove= new CANSparkMax(Ports.kDriveFrontRightMove,MotorType.kBrushless);
  private final CANSparkMax frontRightTurn= new CANSparkMax(Ports.kDriveFrontRightTurn,MotorType.kBrushless);
  private final CANSparkMax backLeftMove= new CANSparkMax(Ports.kDriveBackLeftMove,MotorType.kBrushless);
  private final CANSparkMax backLeftTurn = new CANSparkMax(Ports.kDriveBackLeftTurn,MotorType.kBrushless);
  private final CANSparkMax backRightMove= new CANSparkMax(Ports.kDriveBackRightMove,MotorType.kBrushless);
  private final CANSparkMax backRightTurn= new CANSparkMax(Ports.kDriveBackRightTurn,MotorType.kBrushless);
  private final AbsoluteEncoder frontLeftMoveEncoder = frontLeftMove.getAbsoluteEncoder(Type.kDutyCycle);
  private final AbsoluteEncoder frontRightMoveEncoder = frontRightMove.getAbsoluteEncoder(Type.kDutyCycle);
  private final AbsoluteEncoder backLeftMoveEncoder = backLeftMove.getAbsoluteEncoder(Type.kDutyCycle);
  private final AbsoluteEncoder backRightMoveEncoder = backRightMove.getAbsoluteEncoder(Type.kDutyCycle);

  private final Motors motors = new Motors(frontLeftMove, frontLeftTurn, frontRightMove, frontRightTurn, backLeftMove, backLeftTurn, backRightMove, backRightTurn);
  private PIDController movementController= new PIDController(0.1, 0.0, 0.01);
  private double currentRotations;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    //I will find the distance travelled in 10 rotations, then divide by 10
    movementController.setSetpoint(10);
    frontRightMove.follow(frontLeftMove);
    backLeftMove.follow(frontLeftMove);
    backRightMove.follow(frontLeftMove);
    //this is code I will run to deterine distance per rotation
    
  }


  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    currentRotations+=frontLeftMoveEncoder.getPosition();
    currentRotations+=frontRightMoveEncoder.getPosition();
    currentRotations+=backLeftMoveEncoder.getPosition();
    currentRotations+=backRightMoveEncoder.getPosition();
    currentRotations=currentRotations/4;
    SmartDashboard.putNumber("current Rotations", currentRotations);
    SmartDashboard.putNumber("PID output", movementController.calculate(currentRotations));
    frontLeftMove.set(movementController.calculate(currentRotations));
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
