// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Ports {
    public static int kDriveFrontLeftMove = 1;
    public static int kDriveFrontLeftTurn = 2;
    public static int kDriveFrontRightMove = 3;
    public static int kDriveFrontRightTurn = 4;
    public static int kDriveBackLeftMove = 5;
    public static int kDriveBackLeftTurn = 6;
    public static int kDriveBackRightMove = 7;
    public static int kDriveBackRightTurn = 8;
  }

  public static class MotorConstants{
    public static double movementPerRotation=400;
  }
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class PIDConstants {
    public static final double kDrivetrainP = 0.2;
    public static final double kDrivetrainI = 0.0;
    public static final double kDrivetrainD = 0.0;
    public static final double kPRobotTurn = 0.2;
    public static final double kIRobotTurn = 0.0;
    public static final double kDRobotTurn = 0.0;
    public static final double kPRobotMove = 0.2;
    public static final double kIRobotMove = 0.0;
    public static final double kDRobotMove = 0.0;
  }

  public static class DistanceSensorConstants {
    public static final int AnalogChannel1=0;
    public static final int AnalogChannel2=0;
    //all measurements are in cm
    public static final double distanceScalar=0.125;
    public static final double distanceBetweenSensors=50;
  }
}
