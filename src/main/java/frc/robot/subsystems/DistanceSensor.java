package frc.robot.subsystems;
import frc.robot.Constants.DistanceSensorConstants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DistanceSensor extends SubsystemBase{
    private double voltageScaleFactor;
    private double distanceScalar;
    private double distance1;
    private double distance2;
    private double distance;
    private AnalogInput distanceSensor1;
    private AnalogInput distanceSensor2;
    public DistanceSensor(){
        distanceSensor1= new AnalogInput(DistanceSensorConstants.AnalogChannel1);
        distanceSensor2= new AnalogInput(DistanceSensorConstants.AnalogChannel2);
        voltageScaleFactor=DistanceSensorConstants.distanceScalar;
    }
    public void upddateDistances(double currentVoltage){
        //current voltage fetched using 
        //RobotController.getVoltage5V();
        voltageScaleFactor=5/currentVoltage;
        distance1= distanceSensor1.getValue()*voltageScaleFactor*distanceScalar;
        distance2= distanceSensor2.getValue()*voltageScaleFactor*distanceScalar;
        //do note that if the distance is less than 0.3m or more than 5m, it is represented within 0.3m and 5m
    }
    public boolean checkAligned(){
        //allowing 1cm of difference
        if (Math.abs(distance1-distance2)<0.01){
            return true;
        }
        else{
            return false;
        }
    }
    public double getAngle(){
        return 0.0;
    }
}
