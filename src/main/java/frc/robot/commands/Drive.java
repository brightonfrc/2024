package frc.robot.commands;

import frc.robot.subsystems.Encoders;
import frc.robot.subsystems.Motors;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
/** An example command that uses an example subsystem. */
public class Drive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Encoders encoders;
  private final Motors motors;
  private final Joystick joystick;
  private PIDController bearingControllerFrontLeft;
  private PIDController bearingControllerFrontRight;
  private PIDController bearingControllerBackLeft;
  private PIDController bearingControllerBackRight;

  //find a way to update the PID values
  public double kP;
  public double kI;
  public double kD;
  private double currentBearing;
  private double previousBearingGoal;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Drive(Motors motors, Encoders encoders, Joystick stick) {
    this.encoders=encoders;
    this.motors=motors;
    joystick=stick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motors,encoders);
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
    if(previousBearingGoal!=joystick.getDirectionRadians())  
    {
      //updating the new goal if the joystick is moved
      previousBearingGoal=joystick.getDirectionRadians();
      bearingControllerFrontLeft.setSetpoint(previousBearingGoal);
      bearingControllerFrontLeft.reset();
      bearingControllerFrontRight.setSetpoint(previousBearingGoal);
      bearingControllerFrontRight.reset();
      bearingControllerBackLeft.setSetpoint(previousBearingGoal);
      bearingControllerBackLeft.reset();
      bearingControllerBackRight.setSetpoint(previousBearingGoal);
      bearingControllerBackRight.reset();
    }
    currentBearing=encoders.motorTurned(1);
    motors.setTurnMotors(bearingControllerFrontLeft.calculate(currentBearing),1);
  
    currentBearing=encoders.motorTurned(2);
    motors.setTurnMotors(bearingControllerFrontLeft.calculate(currentBearing),2);

    currentBearing=encoders.motorTurned(3);
    motors.setTurnMotors(bearingControllerFrontLeft.calculate(currentBearing),3);
    
    currentBearing=encoders.motorTurned(4);
    motors.setTurnMotors(bearingControllerFrontLeft.calculate(currentBearing),4);
    //setting the speed, but at 0.5 scale to ensure no one dies
    motors.setMoveMotors(joystick.getMagnitude()*0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motors.setMoveMotors(0);
    motors.setTurnMotors(0, 1);
    motors.setTurnMotors(0, 2);
    motors.setTurnMotors(0, 3);
    motors.setTurnMotors(0, 4);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

