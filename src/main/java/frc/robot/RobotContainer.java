// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// here is where you put all your commands and subsystems;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Encoders;
import frc.robot.subsystems.Motors;
import frc.robot.subsystems.Gyroscope;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // the video says to define the motors within the motors subsystem, but I will just define it here so I know where all my 
  // variables are
  
  // remember to configure the acutal channels
  private final CANSparkMax FrontLeftMove= new CANSparkMax(0,MotorType.kBrushless);
  private final CANSparkMax FrontLeftTurn= new CANSparkMax(0,MotorType.kBrushless);
  private final CANSparkMax FrontRightMove= new CANSparkMax(0,MotorType.kBrushless);
  private final CANSparkMax FrontRightTurn= new CANSparkMax(0,MotorType.kBrushless);
  private final CANSparkMax BackLeftMove= new CANSparkMax(0,MotorType.kBrushless);
  private final CANSparkMax BackLeftTurn = new CANSparkMax(0,MotorType.kBrushless);
  private final CANSparkMax BackRightMove= new CANSparkMax(0,MotorType.kBrushless);
  private final CANSparkMax BackRightTurn= new CANSparkMax(0,MotorType.kBrushless);
  private final Encoder frontLeftMove = new Encoder(0, 0);
  private final Encoder frontLeftTurn = new Encoder(0,0);
  private final Encoder frontRightMove = new Encoder(0, 0);
  private final Encoder frontRightTurn = new Encoder(0,0);
  private final Encoder backLeftMove = new Encoder(0, 0);
  private final Encoder backLeftTurn = new Encoder(0,0);
  private final Encoder backRightMove = new Encoder(0, 0);
  private final Encoder backRightTurn = new Encoder(0,0);
  private final AHRS gyro = new AHRS(I2C.Port.kMXP);


  private final Motors motors= new Motors(FrontLeftMove, FrontLeftTurn, FrontRightMove, FrontRightTurn, BackLeftMove, BackLeftTurn, BackRightMove, BackRightTurn);
  //motor radius is configured in mm and distance per pulse is still unkown
  private final Encoders encoders= new Encoders(frontLeftMove, frontLeftTurn, frontRightTurn, frontRightMove, backLeftMove, backLeftTurn, backRightMove, backRightTurn, 38.1, 0);
  private final Gyroscope gyroscope = new Gyroscope(gyro);
  // remember to set the joystick port
  private Joystick stick = new Joystick(0);
  



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

  
  }
  private void defaultCommands(){
    motors.setDefaultCommand(new Drive(motors,encoders,gyroscope,stick));
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
