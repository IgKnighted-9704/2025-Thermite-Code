package frc.robot.subsystems.miscellaneous;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX armTalon = new TalonFX(Constants.CANIDConstants.Arm_CANID);

    private final PIDController armPID = new PIDController(0,0,0); // tune PID values

    private final double minArmAngle = Constants.PositionRestraintConstants.armMinPos; //min arm angle
    private final double maxArmAngle = Constants.PositionRestraintConstants.armMaxPos ; //max arm angle

    //preset positioning 
    public void raiseArm2pos(double targetPos){
        if(targetPos < maxArmAngle && targetPos > minArmAngle ){
            double currentPos = armTalon.getPosition().getValueAsDouble();
            double speed = armPID.calculate(currentPos, targetPos);
        
            if(speed > 1){
                speed = 1;
            } else if (speed < -1) {
                speed = -1;
            }
        
            armTalon.set(speed);
        } else {
            armTalon.stopMotor();
        }
     }
     //manual positioning
     public void changeArm(double speed){
        double currPosition = armTalon.getPosition().getValueAsDouble();
        if(currPosition < maxArmAngle && currPosition > minArmAngle ){
            armTalon.set(speed);
        } else {
            armTalon.stopMotor();
        }
    }
    //data
    public double getArmPos(){
        return armTalon.getPosition().getValueAsDouble();
    }
}