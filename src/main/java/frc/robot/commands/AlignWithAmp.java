// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Motors;
import frc.robot.subsystems.Motors.TurnMotor;
import frc.robot.subsystems.Encoders;
import frc.robot.subsystems.Encoders.TurnEncoder;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.Constants.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlignWithAmp extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Encoders encoders;
  private final Motors motors;
  private final DistanceSensor distanceSensor;
  private PIDController bearingControllerFrontLeft;
  private PIDController bearingControllerFrontRight;
  private PIDController bearingControllerBackLeft;
  private PIDController bearingControllerBackRight;
  private PIDController robotBearingController;

  public double kP;
  public double kI;
  public double kD;

  public double kPRobot;
  public double kIRobot;
  public double kDRobot;

  private double relativeBearing;
  private double motorBearing;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignWithAmp(Encoders encoders, Motors motors, DistanceSensor distanceSensor) {
    this.encoders=encoders;
    this.motors=motors;
    this.distanceSensor= distanceSensor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motors,encoders,distanceSensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP=PIDConstants.kDrivetrainP;
    kI=PIDConstants.kDrivetrainI;
    kD=PIDConstants.kDrivetrainD;
    bearingControllerFrontLeft=new PIDController(kP, kI, kD);
    bearingControllerFrontLeft.enableContinuousInput(0, 2*Math.PI);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerFrontLeft.setTolerance(Math.PI/180);
    bearingControllerFrontLeft.setSetpoint(Math.PI/4);

    bearingControllerFrontRight=new PIDController(kP, kI, kD);
    bearingControllerFrontRight.enableContinuousInput(0, 2*Math.PI);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerFrontRight.setTolerance(Math.PI/180);
    bearingControllerFrontRight.setSetpoint(Math.PI*3/4);

    bearingControllerBackLeft=new PIDController(kP, kI, kD);
    bearingControllerBackLeft.enableContinuousInput(0, 2*Math.PI);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerBackLeft.setTolerance(Math.PI/180);
    bearingControllerBackLeft.setSetpoint(Math.PI*5/4);

    bearingControllerBackRight=new PIDController(kP, kI, kD);
    bearingControllerBackRight.enableContinuousInput(0, 2*Math.PI);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerBackRight.setTolerance(Math.PI/180);
    bearingControllerBackRight.setSetpoint(Math.PI*7/4);

    kPRobot=PIDConstants.kPRobotTurn;
    kIRobot=PIDConstants.kIRobotTurn;
    kDRobot=PIDConstants.kDRobotTurn;
    robotBearingController= new PIDController(kPRobot, kIRobot, kDRobot);
    robotBearingController.enableContinuousInput(0, 2*Math.PI);
    robotBearingController.setTolerance(Math.PI/180);
    robotBearingController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Command chain active", true);
    if 
      (bearingControllerFrontLeft.atSetpoint()&&
      bearingControllerFrontRight.atSetpoint()&&
      bearingControllerBackLeft.atSetpoint()&&
      bearingControllerBackRight.atSetpoint())
    {
      SmartDashboard.putBoolean("Turning mode", true);
      distanceSensor.upddateDistances();
      relativeBearing=distanceSensor.getAngle();
      //if the output is negative, I will convert to its next positive equivalent
      if (relativeBearing<0){
        relativeBearing+=Math.PI*2;        
      }
      SmartDashboard.putNumber("Relative bearing", relativeBearing);
      motors.setMoveMotors(robotBearingController.calculate(relativeBearing));
    }
    else{
      SmartDashboard.putBoolean("Turning mode", false);

      motorBearing=encoders.motorTurned(TurnEncoder.FRONT_LEFT);
      motors.setTurnMotors(bearingControllerFrontLeft.calculate(motorBearing), TurnMotor.FRONT_LEFT);

      motorBearing=encoders.motorTurned(TurnEncoder.FRONT_RIGHT);
      motors.setTurnMotors(bearingControllerFrontRight.calculate(motorBearing), TurnMotor.FRONT_RIGHT);

      motorBearing=encoders.motorTurned(TurnEncoder.BACK_LEFT);
      motors.setTurnMotors(bearingControllerBackLeft.calculate(motorBearing), TurnMotor.BACK_LEFT);

      motorBearing=encoders.motorTurned(TurnEncoder.BACK_RIGHT);
      motors.setTurnMotors(bearingControllerBackRight.calculate(motorBearing), TurnMotor.BACK_RIGHT);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return robotBearingController.atSetpoint();
  }
}
