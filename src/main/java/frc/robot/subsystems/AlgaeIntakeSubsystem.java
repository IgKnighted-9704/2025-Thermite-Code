package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntakeSubsystem extends SubsystemBase {

    // This motor controls the pivot using a CIM in brushed mode
    private final SparkMax pivotMotor =
            new SparkMax(Constants.AlgaeIntakeConstants.PIVOT_MOTOR_ID, MotorType.kBrushed);

    // Absolute encoder for measuring the pivot angle
    private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

    // PID controller for precise pivot control
    private final PIDController pivotPID = new PIDController(
            Constants.AlgaeIntakeConstants.PIVOT_kP, Constants.AlgaeIntakeConstants.PIVOT_kI,
            Constants.AlgaeIntakeConstants.PIVOT_kD);

    // Brushless NEO motor that drives the intake
    private final SparkMax intakeMotor =
            new SparkMax(Constants.AlgaeIntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

    // Holds the target pivot angle and whether PID is active
    private double desiredPivotAngle = 0.0;
    private boolean pivotPIDEnabled = true;

    public AlgaeIntakeSubsystem() {
        // Configure motor settings here if needed
        pivotEncoder.setPosition((0));
    }

    // Moves the pivot to the defined intake angle and turns on the PID
    public void setPivotToIntake() {
        desiredPivotAngle = Constants.AlgaeIntakeConstants.PIVOT_INTAKE_ANGLE;
        pivotPIDEnabled = true;
    }

    // Turns off the PID and stops the pivot motor, letting it fall freely
    public void stopPivot() {
        desiredPivotAngle = 0.0;
        pivotPIDEnabled = true;
    }

    // Runs the intake forward at the set speed
    public void intakeForward() {
        intakeMotor.set(Constants.AlgaeIntakeConstants.INTAKE_SPEED);
    }

    // Reverses the intake at the same speed
    public void intakeReverse() {
        intakeMotor.set(-Constants.AlgaeIntakeConstants.INTAKE_SPEED);
    }

    // Stops the intake motor
    public void intakeStop() {
        intakeMotor.set(0.0);
    }

    // Returns the current pivot angle (useful for logging or debugging)
    public double getPivotAngle() {
        return pivotEncoder.getPosition() * 360;
    }

    @Override
    public void periodic() {
        Constants.AlgaeIntakeConstants.PIVOT_kP =
                SmartDashboard.getNumber("Algae Pivot kP", Constants.AlgaeIntakeConstants.PIVOT_kP);
        Constants.AlgaeIntakeConstants.PIVOT_kI =
                SmartDashboard.getNumber("Algae Pivot kI", Constants.AlgaeIntakeConstants.PIVOT_kI);
        Constants.AlgaeIntakeConstants.PIVOT_kD =
                SmartDashboard.getNumber("Algae Pivot kD", Constants.AlgaeIntakeConstants.PIVOT_kD);

        SmartDashboard.putNumber("Algae Pivot kP", Constants.AlgaeIntakeConstants.PIVOT_kP);
        SmartDashboard.putNumber("Algae Pivot kI", Constants.AlgaeIntakeConstants.PIVOT_kI);
        SmartDashboard.putNumber("Algae Pivot kD", Constants.AlgaeIntakeConstants.PIVOT_kD);

        // Update the PID controller with new constants
        pivotPID.setPID(Constants.AlgaeIntakeConstants.PIVOT_kP,
                Constants.AlgaeIntakeConstants.PIVOT_kI, Constants.AlgaeIntakeConstants.PIVOT_kD);

        // ----------------------------------------------
        // 2) Report current and desired pivot angles back to SmartDashboard
        // ----------------------------------------------
        SmartDashboard.putNumber("Algae Pivot Current Angle", getPivotAngle());
        SmartDashboard.putNumber("Algae Pivot Desired Angle", desiredPivotAngle);

        // If the pivot PID is active, drive the pivot to the target angle
    //     if (pivotPIDEnabled) {
    //         // Check if the desired pivot angle is within the allowed range
    //         if (desiredPivotAngle >= Constants.AlgaeIntakeConstants.PIVOT_MIN_ANGLE
    //                 && desiredPivotAngle <= Constants.AlgaeIntakeConstants.PIVOT_MAX_ANGLE) {

    //             double currentAngle = pivotEncoder.getPosition();
    //             double power = pivotPID.calculate(currentAngle, desiredPivotAngle);

    //             // Limit motor power to the range [-1, 1]
    //             if (power > 1) {
    //                 power = 1;
    //             } else if (power < -1) {
    //                 power = -1;
    //             }
    //             pivotMotor.set(power / 4);
    //         } else {
    //             // If the angle is out of range, stop the motor for safety
    //             pivotMotor.stopMotor();
    //         }
    //     }
    //     // If pivotPIDEnabled is false, we do nothing to control the pivot,
    //     // allowing it to move freely.
     }
}
