// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
// here is where you put all your commands and subsystems;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.commands.Snap;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.EjectNote;
import frc.robot.commands.FireAmp;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Encoders;
import frc.robot.subsystems.Motors;
import frc.robot.subsystems.Gyroscope;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.Ports;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  // the video says to define the motors within the motors subsystem, but I will just define it here so I know where all my 
  // variables are
  
  // remember to configure the acutal channels
  private final CANSparkMax frontLeftMove= new CANSparkMax(Ports.kDriveFrontLeftMove,MotorType.kBrushless);
  private final CANSparkMax frontLeftTurn= new CANSparkMax(Ports.kDriveFrontLeftTurn,MotorType.kBrushless);
  private final CANSparkMax frontRightMove= new CANSparkMax(Ports.kDriveFrontRightMove,MotorType.kBrushless);
  private final CANSparkMax frontRightTurn= new CANSparkMax(Ports.kDriveFrontRightTurn,MotorType.kBrushless);
  private final CANSparkMax backLeftMove= new CANSparkMax(Ports.kDriveBackLeftMove,MotorType.kBrushless);
  private final CANSparkMax backLeftTurn = new CANSparkMax(Ports.kDriveBackLeftTurn,MotorType.kBrushless);
  private final CANSparkMax backRightMove= new CANSparkMax(Ports.kDriveBackRightMove,MotorType.kBrushless);
  private final CANSparkMax backRightTurn= new CANSparkMax(Ports.kDriveBackRightTurn,MotorType.kBrushless);
  private final AbsoluteEncoder frontLeftMoveEncoder = frontLeftMove.getAbsoluteEncoder(Type.kDutyCycle);
  private final AbsoluteEncoder frontLeftTurnEncoder = frontLeftTurn.getAbsoluteEncoder(Type.kDutyCycle);
  private final AbsoluteEncoder frontRightMoveEncoder = frontRightMove.getAbsoluteEncoder(Type.kDutyCycle);
  private final AbsoluteEncoder frontRightTurnEncoder = frontRightTurn.getAbsoluteEncoder(Type.kDutyCycle);
  private final AbsoluteEncoder backLeftMoveEncoder = backLeftMove.getAbsoluteEncoder(Type.kDutyCycle);
  private final AbsoluteEncoder backLeftTurnEncoder = backLeftTurn.getAbsoluteEncoder(Type.kDutyCycle);
  private final AbsoluteEncoder backRightMoveEncoder = backRightMove.getAbsoluteEncoder(Type.kDutyCycle);
  private final AbsoluteEncoder backRightTurnEncoder = backRightTurn.getAbsoluteEncoder(Type.kDutyCycle);
  private final AHRS gyro = new AHRS(I2C.Port.kMXP);
  private final VictorSPX intakeMotor = new VictorSPX(Ports.kIntakeMotorPort);
  private final VictorSPX leftShooterMotor = new VictorSPX(Ports.kLeftShooterMotor);
  private final VictorSPX rightShooterMotor = new VictorSPX(Ports.kRightShooterMotor);


  private final Motors motors= new Motors(frontLeftMove, frontLeftTurn, frontRightMove, frontRightTurn, backLeftMove, backLeftTurn, backRightMove, backRightTurn);
  //motor radius is configured in mm and distance per rotation is still unkown
  private final Encoders encoders= new Encoders(frontLeftMoveEncoder, frontLeftTurnEncoder, frontRightTurnEncoder, frontRightMoveEncoder, backLeftMoveEncoder, backLeftTurnEncoder, backRightMoveEncoder, backRightTurnEncoder, MotorConstants.movementPerRotation);
  private final Gyroscope gyroscope = new Gyroscope(gyro);
  private final Intake intake = new Intake(intakeMotor);
  private final Shooter shooter = new Shooter(leftShooterMotor, rightShooterMotor);
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  
  private CommandPS4Controller controller = new CommandPS4Controller(OperatorConstants.kDriverControllerPort);




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    defaultCommands();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {


    //snap command is triggered by holding down the circle button. 
    controller.circle().whileTrue(new Snap(motors, encoders, gyroscope, controller));

    controller.L1().whileTrue(new IntakeNote(intake));
    controller.R1().whileTrue(new EjectNote(intake));

    controller.L2().whileTrue(new FireAmp(shooter));
  }
  private void defaultCommands(){
    motors.setDefaultCommand(new Drive(motors,encoders,gyroscope,controller));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
