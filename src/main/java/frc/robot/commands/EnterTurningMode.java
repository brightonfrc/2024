// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Encoders;
import frc.robot.subsystems.Encoders.TurnEncoder;
import frc.robot.subsystems.Motors;
import frc.robot.subsystems.Motors.TurnMotor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class EnterTurningMode extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Encoders encoders;
    private final Motors motors;

    private PIDController bearingControllerFrontLeft;
    private PIDController bearingControllerFrontRight;
    private PIDController bearingControllerBackLeft;
    private PIDController bearingControllerBackRight;
    
    public double kP;
    public double kI;
    public double kD;

    public double currentBearing;
    public boolean endCommand;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public EnterTurningMode(Encoders encoders, Motors motors) {
    this.encoders=encoders;
    this.motors=motors;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(encoders,motors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    endCommand=false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand;
  }
}
