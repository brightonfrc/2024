package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import com.kauailabs.navx.frc.AHRS;




public class Gyroscope extends SubsystemBase {
  public AHRS gyro;
  public double convertedBearing;
  public Gyroscope(){
    gyro= new AHRS(I2C.Port.kMXP);
  }
  public double getBearing(){
    //returns bearing in radians
    convertedBearing= gyro.getAngle()%360;
    if (convertedBearing<0){
      convertedBearing+=360;
    }
    convertedBearing=convertedBearing/180*Math.PI;
    return convertedBearing;
  }
}