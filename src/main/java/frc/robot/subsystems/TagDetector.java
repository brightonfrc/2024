// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.AprilTags;

import edu.wpi.first.apriltag.AprilTagDetector;


import org.opencv.core.Mat;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class TagDetector extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public final AprilTagDetector detector;
  public final AprilTagDetector.Config config;
  public AprilTagDetection detection;
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

}
