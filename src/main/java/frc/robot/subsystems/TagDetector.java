// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.AprilTags;

import edu.wpi.first.apriltag.AprilTagDetector;


import org.opencv.core.Mat;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class TagDetector extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public final AprilTagDetector detector;
  public final AprilTagDetector.Config config;
  public AprilTagDetection detection;

  
  public double bottomLeft;
  public double bottomRight;
  public double topRight;
  public double topLeft;
  public double observedLength;
  public double angleDiff;
  public TagDetector(AprilTagDetector Detector, AprilTagDetector.Config configuration) {
    detector=Detector;
    detector.addFamily(AprilTags.family);
    config=configuration;
    config.numThreads=AprilTags.numThreads;
    config.quadDecimate=AprilTags.quadDecimate;
    config.quadSigma=AprilTags.quadSigma;
    detector.setConfig(config);
  }
  public boolean updateDetection(Mat image, int tagID){
    //this code inherently assumes that there is only 1 april tag on screen at any given point in 
    //time, which I suspect will be true for most cases
    if (detector.detect(image)!=null){
      detection=detector.detect(image)[0];
      if (tagID==detection.getId()){
        //essentially return true if the correct april tag is found
        return true;
      }
      else{
        return false;
      }
    }
    return false;
  }
  public double angleDiff(){
    // this code is incapable of telling if the aprilTag is rotated clockwise or anticlockwise but
    // it can calculate roughly how much it should turn. 
    
    //these are the X coordinates of the april tag corners. We don't need
    //to care about the Y coordinates as we can safely assume that the robot's height
    //remains somewhat constant.
    bottomLeft=detection.getCornerX(0);
    bottomRight=detection.getCornerX(1);
    topRight=detection.getCornerX(2);
    topLeft=detection.getCornerX(3);
    //calculating the average difference in the X coordinates of the corners
    observedLength=Math.abs(bottomLeft-bottomRight);
    observedLength+=Math.abs(topLeft-topRight);
    observedLength=observedLength/2*AprilTags.pixelScaler;
    angleDiff=Math.acos(observedLength/AprilTags.normalTagLength);
    return angleDiff;
  }
  public double pixelToHeightRatio(){
    //these are the Y coordinates of the paril tag corners. 
    //this is because the camera is roughly level with the april tag, so we don't
    //have to worry about angle offset.. 
    bottomLeft=detection.getCornerY(0);
    bottomRight=detection.getCornerY(1);
    topRight=detection.getCornerY(2);
    topLeft=detection.getCornerY(3);
    observedLength=Math.abs(bottomLeft-bottomRight);
    observedLength+=Math.abs(topLeft-topRight);
    observedLength=observedLength/2*AprilTags.pixelScaler;
    return observedLength/AprilTags.normalTagLength;
  }
}
