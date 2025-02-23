package frc.robot.subsystems.miscellaneous;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
    private SparkMax climbMotor = new SparkMax(Constants.CANIDConstants.ClimMotorA_CANID, SparkLowLevel.MotorType.kBrushless);
    private SparkMax climbAngle = new SparkMax(Constants.CANIDConstants.ClimMotorB_CANID, SparkLowLevel.MotorType.kBrushless);

    private double maxPos = Constants.PositionRestraintConstants.climbMaxPos;
    private double minPos = Constants.PositionRestraintConstants.climbMinPos;

    private AbsoluteEncoder climbAngleEncoder = climbAngle.getAbsoluteEncoder();

    private PIDController climbPID = new PIDController(0, 0, 0);

    //preset poistioning
    public void raiseClimbAngle(double targetPos){
        if(targetPos > minPos && targetPos < maxPos){
            double currentPos = climbAngleEncoder.getPosition();
            double power = climbPID.calculate(currentPos, targetPos);
    
            if(power > 1){
                power = 1;
            } else if (power < -1){
                power = -1;
            }
    
            climbAngle.set(power);
        } else {
            climbAngle.stopMotor();
        }
    }
    //manual positioning
    public void changeClimbAngle(double speed){
        double currPosition = climbAngleEncoder.getPosition();
        if(currPosition < maxPos && currPosition > minPos ){
            climbAngle.set(speed);
        } else {
            climbAngle.stopMotor();
        }
    }
    //manual climb
    public void climb(){
        climbMotor.set(0.35);
    }
    //data
    public double getArmPos(){
        return climbAngleEncoder.getPosition();
    }
}