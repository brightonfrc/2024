// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Encoders;
import frc.robot.subsystems.Encoders.TurnEncoder;
import frc.robot.subsystems.Motors;
import frc.robot.subsystems.Motors.TurnMotor;
import frc.robot.subsystems.TagDetector;
import frc.robot.Constants.AprilTags;

import javax.swing.text.html.HTML.Tag;

import edu.wpi.first.apriltag.AprilTag;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlignAmpShot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Encoders encoders;
  private final Motors motors;
  private final TagDetector tagDetector;

  private PIDController bearingControllerFrontLeft;
  private PIDController bearingControllerFrontRight;
  private PIDController bearingControllerBackLeft;
  private PIDController bearingControllerBackRight;
  private PIDController robotBearingController;
  private PIDController robotMovementController;

  public ShuffleboardTab tab;
  public GenericEntry P;
  public GenericEntry I;
  public GenericEntry D;
  //for the motors
  public double kP;
  public double kI;
  public double kD;
  //for the robot
  public double kPRobotTurn;
  public double kIRobotTurn;
  public double kDRobotTurn;
  public double kPRobotMove;
  public double kIRobotMove;
  public double kDRobotMove;

  public int tag;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignAmpShot(Encoders encoders, Motors motors, TagDetector tagDetector) {
    this.encoders=encoders;
    this.motors=motors;
    this.tagDetector=tagDetector;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(encoders,motors,tagDetector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tab= Shuffleboard.getTab("kPMotor");
    P=tab.add("Motor P constant",0).getEntry();
    kP=P.getDouble(0);
    tab= Shuffleboard.getTab("kIMotor");
    I=tab.add("Motor I constant",0).getEntry();
    kI=I.getDouble(0);
    tab= Shuffleboard.getTab("kDMotor");
    D=tab.add("Motor D constant",0).getEntry();
    kD=D.getDouble(0);

    tab= Shuffleboard.getTab("kPRobotTurn");
    P=tab.add("Robot P constant",0).getEntry();
    kPRobotTurn=P.getDouble(0);
    tab= Shuffleboard.getTab("kIRobotTurn");
    I=tab.add("Robot I constant",0).getEntry();
    kIRobotTurn=I.getDouble(0);
    tab= Shuffleboard.getTab("kDRobotTurn");
    D=tab.add("Robot D constant",0).getEntry();
    kDRobotTurn=D.getDouble(0);

    tab= Shuffleboard.getTab("kPRobotMove");
    P=tab.add("Robot P constant",0).getEntry();
    kDRobotMove=P.getDouble(0);
    tab= Shuffleboard.getTab("kIRobotMove");
    I=tab.add("Robot I constant",0).getEntry();
    kIRobotMove=I.getDouble(0);
    tab= Shuffleboard.getTab("kDRobotMove");
    D=tab.add("Robot D constant",0).getEntry();
    kPRobotMove=D.getDouble(0);

    // first the turn motors need to get ready for turning
    bearingControllerFrontLeft=new PIDController(kP, kI, kD);
    bearingControllerFrontLeft.enableContinuousInput(0, 2*Math.PI);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerFrontLeft.setTolerance(Math.PI/360);

    bearingControllerFrontRight=new PIDController(kP, kI, kD);
    bearingControllerFrontRight.enableContinuousInput(0, 2*Math.PI);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerFrontRight.setTolerance(Math.PI/360);

    bearingControllerBackLeft=new PIDController(kP, kI, kD);
    bearingControllerBackLeft.enableContinuousInput(0, 2*Math.PI);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerBackLeft.setTolerance(Math.PI/360);

    bearingControllerBackRight=new PIDController(kP, kI, kD);
    bearingControllerBackRight.enableContinuousInput(0, 2*Math.PI);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerBackRight.setTolerance(Math.PI/360);

    //the PID controller for the robot itself after turning mode has been achieved
    robotBearingController= new PIDController(kPRobotTurn, kIRobotTurn, kDRobotTurn);
    robotBearingController.enableContinuousInput(0, 2*Math.PI);
    robotBearingController.setTolerance(Math.PI/360);

    //the PID controller for the robot so that it can move into position
    robotMovementController= new PIDController(kDRobotMove, kIRobotMove, kDRobotMove);
    //temporarily setting the movement controller to 1cm. 
    robotMovementController.setTolerance(0.01);

    tag=AprilTags.ampNum;
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
