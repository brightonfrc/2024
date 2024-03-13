package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;



public class Gyroscope extends SubsystemBase {
  public AHRS gyro;
  public double convertedBearing;
  /**
  * This creates the Gyroscope subsystem to manage the bearing of the robot
  * @param Gyro the AHRS object representing the gyro
  */
  public Gyroscope(AHRS Gyro){
    gyro=Gyro;
  }
  /**
   * This returns the bearing of the robot
   * @return Returns the bearing of the robot from range 0 to 2PI radians. 
   */
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

