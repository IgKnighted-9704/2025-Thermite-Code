package frc.robot.subsystems.miscellaneous;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.Constants;

public class ElevatorSubsystem extends ArmSubsystem{
        private final TalonFX elevatorTalonA = new TalonFX(Constants.CANIDConstants.ElevatorA_CANID);
        private final TalonFX elevatorTalonB = new TalonFX(Constants.CANIDConstants.ElevatorB_CANID);

    private final PIDController elevatorPID = new PIDController(0,0,0); // tune PID values

    private final double minHeight = Constants.PositionRestraintConstants.armMinPos; //min arm angle
    private final double maxHeight = Constants.PositionRestraintConstants.armMaxPos ; //max arm angle

    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0,0,0);

    //preset positioning 
    public void raiseElevatorPos(double targetPos){
        if(targetPos < maxHeight && targetPos > minHeight){
            double currentPos = elevatorTalonA.getPosition().getValueAsDouble();
            elevatorPID.setSetpoint(targetPos);
            double speed = elevatorPID.calculate(currentPos);
            if(speed > 1){
                speed = 1;
            } else if (speed < -1) {
                speed = -1;
            }
            
            elevatorTalonA.setVoltage(speed + elevatorFeedforward.calculate(speed));
            elevatorTalonB.set(speed + elevatorFeedforward.calculate(speed));
        } else {
            elevatorTalonA.stopMotor();
            elevatorTalonA.stopMotor();;
        }
     }
     //manual positioning
     public void changeElevator(double speed){
        double currPosition = elevatorTalonA.getPosition().getValueAsDouble();
        if(currPosition < maxHeight || currPosition > minHeight ){
            elevatorTalonA.set(speed);
            elevatorTalonB.set(speed);
        } else {
            elevatorTalonA.stopMotor();
            elevatorTalonB.stopMotor();
        }
    }
    //data
    public double getHeight(){
        return elevatorTalonA.getPosition().getValueAsDouble();
    }
}
