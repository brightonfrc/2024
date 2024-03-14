// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.subsystems.Encoders;
// import frc.robot.subsystems.Encoders.TurnEncoder;
// import frc.robot.subsystems.Motors;
// import frc.robot.subsystems.Motors.TurnMotor;
// import frc.robot.subsystems.TagDetector;
// import frc.robot.Constants.AmpAprilTag;
// import frc.robot.Constants.PIDConstants;


// import org.opencv.core.Mat;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;

// /** An example command that uses an example subsystem. */
// public class MoveToAmp extends Command {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final Encoders encoders;
//   private final Motors motors;
//   private final TagDetector tagDetector;

//   private PIDController bearingControllerFrontLeft;
//   private PIDController bearingControllerFrontRight;
//   private PIDController bearingControllerBackLeft;
//   private PIDController bearingControllerBackRight;
//   private PIDController robotMovementController;

//   public ShuffleboardTab tab;
//   public GenericEntry P;
//   public GenericEntry I;
//   public GenericEntry D;
//   //for the motors
//   public double kP;
//   public double kI;
//   public double kD;
//   //for the robot
//   public double kPRobotMove;
//   public double kIRobotMove;
//   public double kDRobotMove;

//   public int tag;
//   public Mat image;
//   //the left-right offset required to get to the april tag
//   public double horizontalOffset;
//   //the forward-backward offset required to get to the april tag. 
//   public double verticalOffset;
//   //the bearing that the motors need to face
//   public double motorGoal;
//   //the diagonal distance that needs to be traversed. 
//   public double offsetMagnitude;
//   public boolean endCommand;
//   public boolean firstAngleIteration;
//   public boolean firstMovementIteration;
//   public double currentBearing;



//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public MoveToAmp(Encoders encoders, Motors motors, TagDetector tagDetector) {
//     //remember to add the camera to this command. 

//     this.encoders=encoders;
//     this.motors=motors;
//     this.tagDetector=tagDetector;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(encoders,motors,tagDetector);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     kP=PIDConstants.kDrivetrainP;
//     kI=PIDConstants.kDrivetrainI;
//     kD=PIDConstants.kDrivetrainD;

//     kPRobotMove=PIDConstants.kRobotMoveP;
//     kIRobotMove=PIDConstants.kRobotMoveI;
//     kDRobotMove=PIDConstants.kRobotMoveD;

//     bearingControllerFrontLeft=new PIDController(kP, kI, kD);
//     bearingControllerFrontLeft.enableContinuousInput(0, 2*Math.PI);
//     //setting tolerance to +=1 degree (radians equivalent)
//     bearingControllerFrontLeft.setTolerance(Math.PI/360);

//     bearingControllerFrontRight=new PIDController(kP, kI, kD);
//     bearingControllerFrontRight.enableContinuousInput(0, 2*Math.PI);
//     //setting tolerance to +=1 degree (radians equivalent)
//     bearingControllerFrontRight.setTolerance(Math.PI/360);

//     bearingControllerBackLeft=new PIDController(kP, kI, kD);
//     bearingControllerBackLeft.enableContinuousInput(0, 2*Math.PI);
//     //setting tolerance to +=1 degree (radians equivalent)
//     bearingControllerBackLeft.setTolerance(Math.PI/360);

//     bearingControllerBackRight=new PIDController(kP, kI, kD);
//     bearingControllerBackRight.enableContinuousInput(0, 2*Math.PI);
//     //setting tolerance to +=1 degree (radians equivalent)
//     bearingControllerBackRight.setTolerance(Math.PI/360);

//     //the PID controller for the robot so that it can move into position
//     robotMovementController= new PIDController(kDRobotMove, kIRobotMove, kDRobotMove);
//     //temporarily setting the movement controller to 1cm. 
//     robotMovementController.setTolerance(0.01);

//     tag=AmpAprilTag.ampNum;
//     firstAngleIteration=true;
//     firstMovementIteration=true;
//     SmartDashboard.putBoolean("Entered move Command", true);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     //take a picture
//     if (tagDetector.updateDetection(image, tag)==false){
//       //essntially, if an image is not found, stop this command
//       tab=Shuffleboard.getTab("AmpShot Command");
//       tab.add("Error","Image not found");
//     }
    
//     horizontalOffset=tagDetector.distanceFromCenter();
//     verticalOffset=tagDetector.distanceToTag()-AmpAprilTag.optimalDistance;
//     if (firstAngleIteration){
//       //in theory, if the vertical offset is zero, this could cause problems, so I am defualting it to 1 cm.
//       if (verticalOffset==0)
//       {
//         verticalOffset=0.01;
//       }
//       motorGoal=Math.atan(horizontalOffset/verticalOffset);
//       bearingControllerFrontLeft.setSetpoint(motorGoal);
//       bearingControllerFrontRight.setSetpoint(motorGoal);
//       bearingControllerBackLeft.setSetpoint(motorGoal);
//       bearingControllerBackRight.setSetpoint(motorGoal);
//       firstAngleIteration=false;
//     }
//     if (bearingControllerFrontLeft.atSetpoint() ||
//     bearingControllerFrontRight.atSetpoint() ||
//     bearingControllerBackLeft.atSetpoint() ||
//     bearingControllerBackRight.atSetpoint()){
//       if (firstMovementIteration){
//         robotMovementController.setSetpoint(AmpAprilTag.optimalDistance);
//         firstMovementIteration=false;
//       }  
//       if (robotMovementController.atSetpoint()){
//         endCommand=true;
//       }
//       //pythagoras
//       offsetMagnitude=Math.sqrt(Math.pow(horizontalOffset,2)+Math.pow(verticalOffset,2));
//       motors.setMoveMotors(robotMovementController.calculate(offsetMagnitude));
//     }
//     else{
//       currentBearing=encoders.motorTurned(TurnEncoder.FRONT_LEFT);
//       motors.setTurnMotors(bearingControllerFrontLeft.calculate(currentBearing), TurnMotor.FRONT_LEFT);
    
//       currentBearing=encoders.motorTurned(TurnEncoder.FRONT_RIGHT);
//       motors.setTurnMotors(bearingControllerFrontRight.calculate(currentBearing), TurnMotor.FRONT_RIGHT);

//       currentBearing=encoders.motorTurned(TurnEncoder.BACK_LEFT);
//       motors.setTurnMotors(bearingControllerBackLeft.calculate(currentBearing), TurnMotor.BACK_LEFT);
      
//       currentBearing=encoders.motorTurned(TurnEncoder.BACK_RIGHT);
//       motors.setTurnMotors(bearingControllerBackRight.calculate(currentBearing), TurnMotor.BACK_RIGHT);
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     motors.setTurnMotors(0, TurnMotor.FRONT_LEFT);
//     motors.setTurnMotors(0,TurnMotor.FRONT_RIGHT);
//     motors.setTurnMotors(0, TurnMotor.BACK_LEFT);
//     motors.setTurnMotors(0, TurnMotor.BACK_RIGHT);
//     motors.setMoveMotors(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return endCommand;
//   }
// }