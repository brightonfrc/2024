package frc.robot.commands;

import frc.robot.subsystems.Encoders;
import frc.robot.subsystems.Encoders.TurnEncoder;
import frc.robot.subsystems.Motors;
import frc.robot.subsystems.Motors.TurnMotor;
import frc.robot.subsystems.Gyroscope;
import frc.robot.Constants.PIDConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/** An example command that uses an example subsystem. */
public class Drive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Encoders encoders;
  private final Motors motors;
  private final Gyroscope gyro;
  private final Joystick joystick;
  private PIDController bearingControllerFrontLeft;
  private PIDController bearingControllerFrontRight;
  private PIDController bearingControllerBackLeft;
  private PIDController bearingControllerBackRight;

  public double kP;
  public double kI;
  public double kD;

  public ShuffleboardTab tab;
  // public GenericEntry P;
  // public GenericEntry I;
  // public GenericEntry D;
  



  private double currentBearing;
  private double previousBearingGoal;
  private double fieldOrientOffset;
  private double joystickBearing;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Drive(Motors motors, Encoders encoders, Gyroscope gyro, Joystick stick) {
    this.encoders=encoders;
    this.motors=motors;
    joystick=stick;
    this.gyro=gyro;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motors,encoders,gyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //tuning the PID constants

    // tab= Shuffleboard.getTab("kP");
    // P=tab.add("P constant",0).getEntry();
    kP=PIDConstants.kDrivetrainP;
    // tab= Shuffleboard.getTab("kI");
    // I=tab.add("I constant",0).getEntry();
    kI=PIDConstants.kDrivetrainI;
    // tab= Shuffleboard.getTab("kD");
    // D=tab.add("D constant",0).getEntry();
    kD=PIDConstants.kDrivetrainD;

    bearingControllerFrontLeft=new PIDController(kP, kI, kD);
    bearingControllerFrontLeft.enableContinuousInput(0, Math.PI*2);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerFrontLeft.setTolerance(Math.PI/180);

    bearingControllerFrontRight=new PIDController(kP, kI, kD);
    bearingControllerFrontRight.enableContinuousInput(0, Math.PI*2);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerFrontRight.setTolerance(Math.PI/180);

    bearingControllerBackLeft=new PIDController(kP, kI, kD);
    bearingControllerBackLeft.enableContinuousInput(0, Math.PI*2);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerBackLeft.setTolerance(Math.PI/180);

    bearingControllerBackRight=new PIDController(kP, kI, kD);
    bearingControllerBackRight.enableContinuousInput(0, Math.PI*2);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerBackRight.setTolerance(Math.PI/180);
    motors.setMoveMotors(0);
    motors.setTurnMotors(0,TurnMotor.FRONT_LEFT);
    motors.setTurnMotors(0,TurnMotor.FRONT_RIGHT);
    motors.setTurnMotors(0,TurnMotor.BACK_LEFT);
    motors.setTurnMotors(0,TurnMotor.BACK_RIGHT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("joystick Magnitude", joystick.getMagnitude());
    //Adding Pi to the joystick output so that it is within the range 0 to 2 pi
    joystickBearing=joystick.getDirectionRadians();
    fieldOrientOffset=gyro.getBearing();
    joystickBearing-=fieldOrientOffset;
    if (joystickBearing<0){
      joystickBearing+=Math.PI*2;
    }
    SmartDashboard.putNumber("joystick/bearing",joystickBearing);
    SmartDashboard.putNumber("gyro offset", fieldOrientOffset);
    //I won't update the new bearing unless it is 1 degree different to the old bearing. 
    if(Math.abs(previousBearingGoal-joystickBearing)>Math.PI/360)  
    {
      //updating the new goal if the joystick is moved
      previousBearingGoal=joystickBearing;
      bearingControllerFrontLeft.setSetpoint(previousBearingGoal);
      bearingControllerFrontLeft.reset();
      bearingControllerFrontRight.setSetpoint(previousBearingGoal);
      bearingControllerFrontRight.reset();
      bearingControllerBackLeft.setSetpoint(previousBearingGoal);
      bearingControllerBackLeft.reset();
      bearingControllerBackRight.setSetpoint(previousBearingGoal);
      bearingControllerBackRight.reset();
    }
    currentBearing=encoders.motorTurned(TurnEncoder.FRONT_LEFT);
    motors.setTurnMotors(bearingControllerFrontLeft.calculate(currentBearing)*0.5, TurnMotor.FRONT_LEFT);
    SmartDashboard.putNumber("Front left Bearing", currentBearing);
  
    currentBearing=encoders.motorTurned(TurnEncoder.FRONT_RIGHT);
    motors.setTurnMotors(bearingControllerFrontRight.calculate(currentBearing)*0.5, TurnMotor.FRONT_RIGHT);
    SmartDashboard.putNumber("Front right Bearing", currentBearing);
  
    currentBearing=encoders.motorTurned(TurnEncoder.BACK_LEFT);
    motors.setTurnMotors(bearingControllerBackLeft.calculate(currentBearing)*0.5, TurnMotor.BACK_LEFT);
    SmartDashboard.putNumber("Back left Bearing", currentBearing);
  
    currentBearing=encoders.motorTurned(TurnEncoder.BACK_RIGHT);
    motors.setTurnMotors(bearingControllerBackRight.calculate(currentBearing)*0.5, TurnMotor.BACK_RIGHT);
    SmartDashboard.putNumber("Back right Bearing", currentBearing);
  
    //setting the speed, but at 0.1 scale to ensure no one dies
    motors.setMoveMotors(joystick.getMagnitude()*0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motors.setMoveMotors(0);
    motors.setTurnMotors(0, TurnMotor.FRONT_LEFT);
    motors.setTurnMotors(0, TurnMotor.FRONT_RIGHT);
    motors.setTurnMotors(0, TurnMotor.BACK_LEFT);
    motors.setTurnMotors(0, TurnMotor.BACK_RIGHT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

