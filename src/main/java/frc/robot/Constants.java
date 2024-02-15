// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
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
    public static double distancePerRotation=10.0;
    public static double movementPerRotation=10.0;
  }
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int snapButton=1;
    public static final int AmpShotButton=2;
  }

  public static class PIDConstants {
    public static final double kDrivetrainP = 1.0;
    public static final double kDrivetrainI = 0.0;
    public static final double kDrivetrainD = 0.0;
  }
  public static class AprilTags{
    //this stores the settings for the april tag detector config and the families to be detected by the detector
    //remember to actually configure these
    public static final int numThreads=1;
    public static final float quadDecimate=2;
    public static final float quadSigma=0;
    //6.5 inches to mm
    public static final double normalTagLength=165.1;
    //the number of pixels the border of the april tag will have at 1m. 
    public static final double pixelsAtMeter=400;
    //the X coordinate of the center when the april tag is aligned with the camera. 
    public static final double centerXCoordinate=300;
    //https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html
    public static final String family="36h11";
  }
  public static class AmpAprilTag{
    //remember to actually fill this in with the correct april tags
    public static final int ampNum=0;
    //the optimal distance at which the note can be inserted into the amp in m
    public static final double optimalDistance=0.1;
  }
}
