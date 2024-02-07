package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;



public class Gyroscope extends SubsystemBase {
  public AHRS gyro;
  public double convertedBearing;
  public Gyroscope(AHRS Gyro){
    gyro=Gyro;
  }
  public double getBearing(){
    //returns bearing in radians
    convertedBearing= gyro.getAngle()%360;
    convertedBearing=convertedBearing/180*Math.PI;
    return convertedBearing;
  }
}

