// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Encoders;
import frc.robot.subsystems.Encoders.TurnEncoder;
import frc.robot.subsystems.Motors;
import frc.robot.subsystems.Motors.TurnMotor;
import frc.robot.subsystems.TagDetector;
import frc.robot.Constants.AmpAprilTag;
import frc.robot.Constants.PIDConstants;

import javax.swing.text.StyleContext.SmallAttributeSet;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlignWithAprilTag extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Encoders encoders;
  private final Motors motors;
  private final TagDetector tagDetector;

  private PIDController bearingControllerFrontLeft;
  private PIDController bearingControllerFrontRight;
  private PIDController bearingControllerBackLeft;
  private PIDController bearingControllerBackRight;
  private PIDController robotBearingController;

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

  public int tag;
  public Mat image;
  public double angleDiff;

  public boolean firstIteration;
  public boolean foundTurningDirection;
  public double previousAngleDiff;
  public boolean inverseDirection;
  public double orientationTime;
  public boolean orientationEnter;

  public boolean endCommand;
  public double currentBearing;



  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignWithAprilTag(Encoders encoders, Motors motors, TagDetector tagDetector) {
    //remember to add the camera to this command. 

    this.encoders=encoders;
    this.motors=motors;
    this.tagDetector=tagDetector;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(encoders,motors,tagDetector);
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

    //the setpoint is set in a way that will cause the robot to enter turning mode, 
    //but it does not necessarily have to be used. 
    bearingControllerFrontLeft=new PIDController(kP, kI, kD);
    bearingControllerFrontLeft.enableContinuousInput(0, 2*Math.PI);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerFrontLeft.setTolerance(Math.PI/360);
    bearingControllerFrontLeft.setSetpoint(Math.PI/4);

    bearingControllerFrontRight=new PIDController(kP, kI, kD);
    bearingControllerFrontRight.enableContinuousInput(0, 2*Math.PI);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerFrontRight.setTolerance(Math.PI/360);
    bearingControllerFrontRight.setSetpoint(Math.PI/4*3);

    bearingControllerBackLeft=new PIDController(kP, kI, kD);
    bearingControllerBackLeft.enableContinuousInput(0, 2*Math.PI);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerBackLeft.setTolerance(Math.PI/360);
    bearingControllerBackLeft.setSetpoint(Math.PI/4*5);

    bearingControllerBackRight=new PIDController(kP, kI, kD);
    bearingControllerBackRight.enableContinuousInput(0, 2*Math.PI);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerBackRight.setTolerance(Math.PI/360);
    bearingControllerBackRight.setSetpoint(Math.PI/4*7);;

    //the PID controller for the robot itself after turning mode has been achieved
    robotBearingController= new PIDController(kPRobotTurn, kIRobotTurn, kDRobotTurn);
    robotBearingController.enableContinuousInput(0, 2*Math.PI);
    robotBearingController.setTolerance(Math.PI/360);

    
    tag=AmpAprilTag.ampNum;
    //giving it a dummy value
    angleDiff=10;
    firstIteration=true;
    orientationEnter=true;
    SmartDashboard.putBoolean("Entered align Command", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //take a picture
    if (tagDetector.updateDetection(image, tag)==false){
      //essntially, if an image is not found, stop this command
      tab=Shuffleboard.getTab("AmpShot Command");
      tab.add("Error","Image not found");
    }
    angleDiff=tagDetector.angleDiff(tagDetector.distanceToTag());
    if (firstIteration){
      previousAngleDiff=angleDiff;
      firstIteration=false;
    }
    if (angleDiff>Math.PI/360){
      
      //allowing an error of 1 degrees. 
      if(bearingControllerFrontLeft.atSetpoint() ||
      bearingControllerFrontRight.atSetpoint() ||
      bearingControllerBackLeft.atSetpoint() ||
      bearingControllerBackRight.atSetpoint()){  
        
        if(foundTurningDirection){
          robotBearingController.setSetpoint(0);
          if (inverseDirection){
            motors.setMoveMotors(-1*robotBearingController.calculate(angleDiff));
          } else {
            motors.setMoveMotors(robotBearingController.calculate(angleDiff));
          }
        }  
        else{
          //turn right for 0.5s to check if the robot should turn anticlockwise or clockwise. 
          if (orientationEnter){
            orientationTime=System.currentTimeMillis();
            orientationEnter=false;
          }
          motors.setMoveMotors(0.5);
          if (System.currentTimeMillis()-orientationTime>500){
            //essentially, if this is a second iteration, then check how setting move motors affected
            //the angle difference
            if (previousAngleDiff<angleDiff){
              inverseDirection=true;
              foundTurningDirection=true;
            }
          }
        }
      }
      else{
        //enter turning mode
        currentBearing=encoders.motorTurned(TurnEncoder.FRONT_LEFT);
        motors.setTurnMotors(bearingControllerFrontLeft.calculate(currentBearing), TurnMotor.FRONT_LEFT);
    
        currentBearing=encoders.motorTurned(TurnEncoder.FRONT_RIGHT);
        motors.setTurnMotors(bearingControllerFrontRight.calculate(currentBearing), TurnMotor.FRONT_RIGHT);

        currentBearing=encoders.motorTurned(TurnEncoder.BACK_LEFT);
        motors.setTurnMotors(bearingControllerBackLeft.calculate(currentBearing), TurnMotor.BACK_LEFT);
      
        currentBearing=encoders.motorTurned(TurnEncoder.BACK_RIGHT);
        motors.setTurnMotors(bearingControllerBackRight.calculate(currentBearing), TurnMotor.BACK_RIGHT);
      }
    }
    else{
      endCommand=true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motors.setTurnMotors(0, TurnMotor.FRONT_LEFT);
    motors.setTurnMotors(0,TurnMotor.FRONT_RIGHT);
    motors.setTurnMotors(0, TurnMotor.BACK_LEFT);
    motors.setTurnMotors(0, TurnMotor.BACK_RIGHT);
    motors.setMoveMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand;
  }
}
