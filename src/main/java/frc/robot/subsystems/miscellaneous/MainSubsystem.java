package frc.robot.subsystems.miscellaneous;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MainSubsystem extends SubsystemBase{

  private final SparkMax cMotor = new SparkMax(Constants.CANIDConstants.CoralIntake_CANID, SparkLowLevel.MotorType.kBrushless); //change brush/brushed


 //coral
 public void endEffector(double speed){
    cMotor.set(speed);
 }
}
