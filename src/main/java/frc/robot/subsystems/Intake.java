package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;




public class Intake extends SubsystemBase {
  public VictorSPX leftIntake;
  public Intake(VictorSPX intakeMotor){
    leftIntake=intakeMotor;
  }
  public void runIntake(double input){
    SmartDashboard.putNumber("Intake Running", input);
    leftIntake.set(ControlMode.PercentOutput, input);
    
  }
}
