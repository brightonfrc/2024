package frc.robot.subsystems;
import frc.robot.Constants.DistanceSensorConstants;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DistanceSensor extends SubsystemBase{
    private double voltageScaleFactor;
    private double distanceScalar;
    private double currentVoltage;
    private double distance1;
    private double distance2;

    private double distance;

    private double difference;

    private AnalogInput distanceSensor1;
    private AnalogInput distanceSensor2;
    public DistanceSensor(){
        distanceSensor1= new AnalogInput(DistanceSensorConstants.AnalogChannel1);
        distanceSensor2= new AnalogInput(DistanceSensorConstants.AnalogChannel2);
        voltageScaleFactor=DistanceSensorConstants.distanceScalar;
    }
    public void upddateDistances(){
        //every time you want to use any of the methods, this must be called first
        currentVoltage=RobotController.getVoltage5V();
        voltageScaleFactor=5/currentVoltage;
        distance1= distanceSensor1.getValue()*voltageScaleFactor*distanceScalar;
        distance2= distanceSensor2.getValue()*voltageScaleFactor*distanceScalar;
        //do note that if the distance is less than 30cm or more than 500cm, it is represented within 30cm and 500cm
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
    public double getDistance(){
        distance=distance1+distance2;
        distance=distance/2;
        return distance;
    }
    public double getAngle(){
        //I'm assuming that distanceSensor1 is the sensor on the right
        difference=distance1-distance2;
        //funny thing is that theoretically, the bearing relative to the wall must be within -90 and 90.
        //so I only need to use atan instead of atan2
        return Math.atan(difference/DistanceSensorConstants.distanceBetweenSensors);
    }
}
