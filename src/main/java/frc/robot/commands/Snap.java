package frc.robot.commands;

import frc.robot.subsystems.Encoders;
import frc.robot.subsystems.Encoders.TurnEncoder;
import frc.robot.subsystems.Motors;
import frc.robot.subsystems.Motors.TurnMotor;
import frc.robot.subsystems.Gyroscope;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.SnapConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import java.lang.Math;

/** An example command that uses an example subsystem. */
public class Snap extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Motors motors;
  private final Encoders encoders;
  private final Gyroscope gyro;
  private final Joystick stick;
  
  private PIDController bearingControllerFrontLeft;
  private PIDController bearingControllerFrontRight;
  private PIDController bearingControllerBackLeft;
  private PIDController bearingControllerBackRight;
  private PIDController robotBearingController;

  public ShuffleboardTab tab;
  //for the motors
  public double kP;
  public double kI;
  public double kD;
  //for the robot
  public double kPRobot;
  public double kIRobot;
  public double kDRobot;

  private double joystickBearing;
  private double currentBearing;
  private double snapGoal;
  private double previousSnapGoal;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Snap(Motors motors, Encoders encoders, Gyroscope gyro, Joystick stick) {
    this.motors = motors;
    this.encoders= encoders;
    this.gyro=gyro;
    this.stick= stick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motors, encoders, gyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP=PIDConstants.snapMotorP;
    kI=PIDConstants.snapMotorI;
    kD=PIDConstants.snapMotorD;

    kPRobot=PIDConstants.kRobotTurningP;
    kIRobot=PIDConstants.kRobotTurningI;
    kDRobot=PIDConstants.kRobotTurningD;
    
    // first the turn motors need to get ready for turning
    bearingControllerFrontLeft=new PIDController(kP, kI, kD);
    bearingControllerFrontLeft.enableContinuousInput(0, 2*Math.PI);
    //setting tolerance to +=2.5 degree (radians equivalent)
    bearingControllerFrontLeft.setTolerance(Math.PI/180*SnapConstants.snapAngleTolerance);
    bearingControllerFrontLeft.setSetpoint(Math.PI*7/4);

    bearingControllerFrontRight=new PIDController(kP, kI, kD);
    bearingControllerFrontRight.enableContinuousInput(0, 2*Math.PI);
    //setting tolerance to +=2.5 degree (radians equivalent)
    bearingControllerFrontRight.setTolerance(Math.PI/180*SnapConstants.snapAngleTolerance);
    bearingControllerFrontRight.setSetpoint(Math.PI*5/4);

    bearingControllerBackLeft=new PIDController(kP, kI, kD);
    bearingControllerBackLeft.enableContinuousInput(0, 2*Math.PI);
    //setting tolerance to +=2.5 degree (radians equivalent)
    bearingControllerBackLeft.setTolerance(Math.PI/180*SnapConstants.snapAngleTolerance);
    bearingControllerBackLeft.setSetpoint(Math.PI*1/4);

    bearingControllerBackRight=new PIDController(kP, kI, kD);
    bearingControllerBackRight.enableContinuousInput(0, 2*Math.PI);
    //setting tolerance to +=2.5 degree (radians equivalent)
    bearingControllerBackRight.setTolerance(Math.PI/180*SnapConstants.snapAngleTolerance);
    bearingControllerBackRight.setSetpoint(Math.PI*7/4);

    //the PID controller for the robot itself after turning mode has been achieved
    robotBearingController= new PIDController(kPRobot, kIRobot, kDRobot);
    robotBearingController.enableContinuousInput(0, 2*Math.PI);
    robotBearingController.setTolerance(Math.PI/180*SnapConstants.snapBearingTolerance);

    //setting all the motors to zero
    motors.setTurnMotors(0, TurnMotor.FRONT_LEFT);
    motors.setTurnMotors(0, TurnMotor.FRONT_RIGHT);
    motors.setTurnMotors(0, TurnMotor.BACK_LEFT);
    motors.setTurnMotors(0, TurnMotor.BACK_RIGHT);
    motors.setMoveMotors(0);
    SmartDashboard.putBoolean("Command active", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if any of the motors are not ready to turn, this code won't run
    if (
      bearingControllerFrontLeft.atSetpoint()==false ||
      bearingControllerFrontRight.atSetpoint()==false ||
      bearingControllerBackLeft.atSetpoint()==false ||
      bearingControllerBackRight.atSetpoint()==false){
      currentBearing=encoders.motorTurned(TurnEncoder.FRONT_LEFT);
      SmartDashboard.putNumber("FL Bearing", currentBearing);
      motors.setTurnMotors(bearingControllerFrontLeft.calculate(currentBearing), TurnMotor.FRONT_LEFT);
    
      currentBearing=encoders.motorTurned(TurnEncoder.FRONT_RIGHT);
      SmartDashboard.putNumber("FR Bearing", currentBearing);
      motors.setTurnMotors(bearingControllerFrontRight.calculate(currentBearing), TurnMotor.FRONT_RIGHT);
      
      currentBearing=encoders.motorTurned(TurnEncoder.BACK_LEFT);
      SmartDashboard.putNumber("BL Bearing", currentBearing);
      motors.setTurnMotors(bearingControllerBackLeft.calculate(currentBearing), TurnMotor.BACK_LEFT);
      
      
      currentBearing=encoders.motorTurned(TurnEncoder.BACK_RIGHT);
      SmartDashboard.putNumber("BR Bearing", currentBearing);
      motors.setTurnMotors(bearingControllerBackRight.calculate(currentBearing), TurnMotor.BACK_RIGHT);

      SmartDashboard.putBoolean("FL ready",bearingControllerFrontLeft.atSetpoint());
      SmartDashboard.putBoolean("FR ready", bearingControllerFrontRight.atSetpoint());
      SmartDashboard.putBoolean("BL ready", bearingControllerBackLeft.atSetpoint());
      SmartDashboard.putBoolean("BR ready", bearingControllerBackRight.atSetpoint());
    }
    else
    {
      joystickBearing=stick.getDirectionRadians();
      SmartDashboard.putNumber("JoystickBearing", joystickBearing);
      //converting the joystickBearing to range 0 to 2pi
      if(joystickBearing<0){
        joystickBearing+=2*Math.PI;
      }
      //essentially I won't bother to update this unless the bearing difference is more than 45 degrees
      if (Math.abs(joystickBearing-previousSnapGoal)>Math.PI/4){  
        // coding the data from 1 to 4
        joystickBearing= joystickBearing/Math.PI*2;
        // rounding to closest integer
        int estimate= (int) Math.round(joystickBearing);
        if (estimate==4){
          //the PIDController doesn't appreciate snapping between 0 and 4, which are essentially the same thing
          estimate=0;
        }
        joystickBearing=estimate;
        //adding this because I got a funny feeling about the fact that it is rounded to a long
        SmartDashboard.putNumber("approximation", estimate);
        //converting back to radians
        snapGoal=joystickBearing*Math.PI/2;
        //updating the new goal if the joystick is moved
        robotBearingController.reset();
        robotBearingController.setSetpoint(snapGoal);
        previousSnapGoal=snapGoal;
      }
      SmartDashboard.putNumber("current Bearing", gyro.getBearing());
      SmartDashboard.putNumber("PID setpoint", robotBearingController.getSetpoint());
      SmartDashboard.putNumber("PID output", robotBearingController.calculate(gyro.getBearing()));
      motors.setMoveMotors(robotBearingController.calculate(gyro.getBearing())*0.5);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Command active", false);
    motors.setTurnMotors(0, TurnMotor.FRONT_LEFT);
    motors.setTurnMotors(0,TurnMotor.FRONT_RIGHT);
    motors.setTurnMotors(0, TurnMotor.BACK_LEFT);
    motors.setTurnMotors(0, TurnMotor.BACK_RIGHT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
