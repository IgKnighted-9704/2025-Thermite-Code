package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntakeSubsystem extends SubsystemBase {

    // Pivot motor (CIM in brushed mode)
    private final SparkMax pivotMotor = new SparkMax(
            Constants.AlgaeIntakeConstants.PIVOT_MOTOR_ID,
            MotorType.kBrushed);

    // Absolute encoder (duty cycle)
    private final AbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();

    // PID controller for pivot
    private final PIDController pivotPID = new PIDController(
            Constants.AlgaeIntakeConstants.PIVOT_kP,
            Constants.AlgaeIntakeConstants.PIVOT_kI,
            Constants.AlgaeIntakeConstants.PIVOT_kD);

    // Intake motor (NEO, brushless)
    private final SparkMax intakeMotor = new SparkMax(
            Constants.AlgaeIntakeConstants.INTAKE_MOTOR_ID,
            MotorType.kBrushless);

    // Desired pivot angle and a flag to enable/disable PID
    private double desiredPivotAngle = 0.0;
    private boolean pivotPIDEnabled = false;

    public AlgaeIntakeSubsystem() {
        // motor settings
    }

    // Set the pivot to the intake angle and enable PID
    public void setPivotToIntake() {
        desiredPivotAngle = Constants.AlgaeIntakeConstants.PIVOT_INTAKE_ANGLE;
        pivotPIDEnabled = true;
    }

    // Disable PID and stop the motor so the pivot can fall
    public void stopPivot() {
        pivotPIDEnabled = false;
        pivotMotor.stopMotor();
    }

    // Spin the intake forward
    public void intakeForward() {
        intakeMotor.set(Constants.AlgaeIntakeConstants.INTAKE_SPEED);
    }

    // Spin the intake backward
    public void intakeReverse() {
        intakeMotor.set(-Constants.AlgaeIntakeConstants.INTAKE_SPEED);
    }

    // Stop the intake motor
    public void intakeStop() {
        intakeMotor.set(0.0);
    }

    // Get the current pivot angle (for logging or debug)
    public double getPivotAngle() {
        return pivotEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // If PID is enabled, run the pivot motor toward desiredPivotAngle
        if (pivotPIDEnabled) {
            // Make sure we're within min/max angles
            if (desiredPivotAngle >= Constants.AlgaeIntakeConstants.PIVOT_MIN_ANGLE
                    && desiredPivotAngle <= Constants.AlgaeIntakeConstants.PIVOT_MAX_ANGLE) {

                double currentAngle = pivotEncoder.getPosition();
                double power = pivotPID.calculate(currentAngle, desiredPivotAngle);

                // Clamp power to [-1, 1]
                if (power > 1) {
                    power = 1;
                } else if (power < -1) {
                    power = -1;
                }
                pivotMotor.set(power);
            } else {
                // Out of range: just stop for safety
                pivotMotor.stopMotor();
            }
        }
        // If pivotPIDEnabled is false, we do nothing here,
        // so the pivot motor doesn't hold position.
    }
}
