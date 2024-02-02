package frc.robot.commands;

import frc.robot.subsystems.Encoders;
import frc.robot.subsystems.Encoders.TurnEncoder;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Motors;
import frc.robot.subsystems.Motors.TurnMotor;
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
  private final Joystick stick;
  
  private PIDController bearingControllerFrontLeft;
  private PIDController bearingControllerFrontRight;
  private PIDController bearingControllerBackLeft;
  private PIDController bearingControllerBackRight;

  public double kP;
  public double kI;
  public double kD;
  public ShuffleboardTab tab;
  public GenericEntry P;
  public GenericEntry I;
  public GenericEntry D;

  private double joystickBearing;
  private double currentBearing;
  private double snapGoal;
  private double previousSnapGoal;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Snap(Motors motors, Encoders encoders, Joystick stick) {
    this.motors = motors;
    this.encoders= encoders;
    this.stick= stick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motors, encoders);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    bearingControllerFrontLeft=new PIDController(kP, kI, kD);
    bearingControllerFrontLeft.enableContinuousInput(0, 6.283185307);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerFrontLeft.setTolerance(0.01745329251);

    bearingControllerFrontRight=new PIDController(kP, kI, kD);
    bearingControllerFrontRight.enableContinuousInput(0, 6.283185307);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerFrontRight.setTolerance(0.01745329251);

    bearingControllerBackLeft=new PIDController(kP, kI, kD);
    bearingControllerBackLeft.enableContinuousInput(0, 6.283185307);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerBackLeft.setTolerance(0.01745329251);

    bearingControllerBackRight=new PIDController(kP, kI, kD);
    bearingControllerBackRight.enableContinuousInput(0, 6.283185307);
    //setting tolerance to +=1 degree (radians equivalent)
    bearingControllerBackRight.setTolerance(0.01745329251);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
//tuning the PID constants
    tab= Shuffleboard.getTab("kP");
    P=tab.add("P constant",0).getEntry();
    kP=P.getDouble(0);
    tab= Shuffleboard.getTab("kI");
    I=tab.add("I constant",0).getEntry();
    kI=I.getDouble(0);
    tab= Shuffleboard.getTab("kD");
    D=tab.add("D constant",0).getEntry();
    kD=D.getDouble(0);

    joystickBearing=stick.getDirectionRadians();
    // coding the data from 1 to 4
    joystickBearing= joystickBearing*0.63661977236;
    //converting back to radians
    snapGoal=Math.round(joystickBearing)*1.570796326;
    if (snapGoal!=previousSnapGoal){
      //updating the new goal if the joystick is moved
      bearingControllerFrontLeft.setSetpoint(snapGoal);
      bearingControllerFrontLeft.reset();
      bearingControllerFrontRight.setSetpoint(snapGoal);
      bearingControllerFrontRight.reset();
      bearingControllerBackLeft.setSetpoint(snapGoal);
      bearingControllerBackLeft.reset();
      bearingControllerBackRight.setSetpoint(snapGoal);
      bearingControllerBackRight.reset();
      previousSnapGoal=snapGoal;
    }
    currentBearing=encoders.motorTurned(TurnEncoder.FRONT_LEFT);
    motors.setTurnMotors(bearingControllerFrontLeft.calculate(currentBearing), TurnMotor.FRONT_LEFT);
  
    currentBearing=encoders.motorTurned(TurnEncoder.FRONT_RIGHT);
    motors.setTurnMotors(bearingControllerFrontLeft.calculate(currentBearing), TurnMotor.FRONT_RIGHT);

    currentBearing=encoders.motorTurned(TurnEncoder.BACK_LEFT);
    motors.setTurnMotors(bearingControllerFrontLeft.calculate(currentBearing), TurnMotor.BACK_LEFT);
    
    currentBearing=encoders.motorTurned(TurnEncoder.BACK_RIGHT);
    motors.setTurnMotors(bearingControllerFrontLeft.calculate(currentBearing), TurnMotor.BACK_RIGHT);
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
