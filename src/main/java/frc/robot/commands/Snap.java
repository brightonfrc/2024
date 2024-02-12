package frc.robot.commands;

import frc.robot.subsystems.Encoders;
import frc.robot.subsystems.Encoders.TurnEncoder;
import frc.robot.subsystems.Motors;
import frc.robot.subsystems.Motors.TurnMotor;
import frc.robot.subsystems.Gyroscope;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;

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
  public GenericEntry P;
  public GenericEntry I;
  public GenericEntry D;
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
    tab= Shuffleboard.getTab("kPMotor");
    P=tab.add("Motor P constant",0).getEntry();
    kP=P.getDouble(0);
    tab= Shuffleboard.getTab("kIMotor");
    I=tab.add("Motor I constant",0).getEntry();
    kI=I.getDouble(0);
    tab= Shuffleboard.getTab("kDMotor");
    D=tab.add("Motor D constant",0).getEntry();
    kD=D.getDouble(0);

    tab= Shuffleboard.getTab("kPRobot");
    P=tab.add("Robot P constant",0).getEntry();
    kPRobot=P.getDouble(0);
    tab= Shuffleboard.getTab("kIRobot");
    I=tab.add("Robot I constant",0).getEntry();
    kIRobot=I.getDouble(0);
    tab= Shuffleboard.getTab("kDRobot");
    D=tab.add("Robot D constant",0).getEntry();
    kDRobot=D.getDouble(0);

    // first the turn motors need to get ready for turning
    bearingControllerFrontLeft=new PIDController(kP, kI, kD);
    bearingControllerFrontLeft.enableContinuousInput(0, 2*Math.PI);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerFrontLeft.setTolerance(Math.PI/360);
    bearingControllerFrontLeft.setSetpoint(Math.PI/4);

    bearingControllerFrontRight=new PIDController(kP, kI, kD);
    bearingControllerFrontRight.enableContinuousInput(0, 2*Math.PI);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerFrontRight.setTolerance(Math.PI/360);
    bearingControllerFrontRight.setSetpoint(Math.PI*3/4);

    bearingControllerBackLeft=new PIDController(kP, kI, kD);
    bearingControllerBackLeft.enableContinuousInput(0, 2*Math.PI);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerBackLeft.setTolerance(Math.PI/360);
    bearingControllerBackLeft.setSetpoint(Math.PI*5/4);

    bearingControllerBackRight=new PIDController(kP, kI, kD);
    bearingControllerBackRight.enableContinuousInput(0, 2*Math.PI);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerBackRight.setTolerance(Math.PI/360);
    bearingControllerBackRight.setSetpoint(Math.PI*7/4);

    //the PID controller for the robot itself after turning mode has been achieved
    robotBearingController= new PIDController(kPRobot, kIRobot, kDRobot);
    robotBearingController.enableContinuousInput(0, 2*Math.PI);
    robotBearingController.setTolerance(Math.PI/360);
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
      motors.setTurnMotors(bearingControllerFrontLeft.calculate(currentBearing), TurnMotor.FRONT_LEFT);
    
      currentBearing=encoders.motorTurned(TurnEncoder.FRONT_RIGHT);
      motors.setTurnMotors(bearingControllerFrontRight.calculate(currentBearing), TurnMotor.FRONT_RIGHT);

      currentBearing=encoders.motorTurned(TurnEncoder.BACK_LEFT);
      motors.setTurnMotors(bearingControllerBackLeft.calculate(currentBearing), TurnMotor.BACK_LEFT);
      
      currentBearing=encoders.motorTurned(TurnEncoder.BACK_RIGHT);
      motors.setTurnMotors(bearingControllerBackRight.calculate(currentBearing), TurnMotor.BACK_RIGHT);
    }
    else
    {
      //robot ready to rotate
      joystickBearing=stick.getDirectionRadians();

      if (Math.abs(joystickBearing-previousSnapGoal)>Math.PI/4){  
        // coding the data from 1 to 4
        joystickBearing= joystickBearing/Math.PI*2;
        // rounding to closest integer
        int estimate=(int) joystickBearing;
        joystickBearing=estimate;
        //converting back to radians
        snapGoal=joystickBearing*Math.PI/2;
        //updating the new goal if the joystick is moved
        robotBearingController.reset();
        robotBearingController.setSetpoint(snapGoal);
        previousSnapGoal=snapGoal;
      }
      motors.setMoveMotors(robotBearingController.calculate(gyro.getBearing()));
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
