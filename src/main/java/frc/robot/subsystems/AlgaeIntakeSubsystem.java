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

    // This brushed CIM motor adjusts the pivot for the intake mechanism
    private final SparkMax pivotMotor =
            new SparkMax(Constants.AlgaeIntakeConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);

    private final SparkMax climbMotor2 =
            new SparkMax(Constants.ClimbConstants.CLIMB_MOTOR_B_ID, MotorType.kBrushed);

    // Using a relative encoder here to measure the pivot's angle
    private final AbsoluteEncoder pivotEncoder = climbMotor2.getAbsoluteEncoder();

    // PID controller to handle precise pivot angle adjustments
    private final PIDController pivotPID = new PIDController(
            Constants.AlgaeIntakeConstants.PIVOT_kP, Constants.AlgaeIntakeConstants.PIVOT_kI,
            Constants.AlgaeIntakeConstants.PIVOT_kD);

    // Brushless NEO motor dedicated to running the intake
    private final SparkMax intakeMotor =
            new SparkMax(Constants.AlgaeIntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

    // Variables for the target pivot angle and whether PID is active
    private double desiredPivotAngle = 0.0;
    private boolean pivotPIDEnabled = true;

    public AlgaeIntakeSubsystem() {
        // Set the initial encoder position (change if needed in the future)
        // pivotEncoder.setPosition(0);
    }

    // Moves the pivot to our defined intake angle and activates the PID controller
    public void setPivotToIntake() {
        desiredPivotAngle = Constants.AlgaeIntakeConstants.PIVOT_INTAKE_ANGLE;
        pivotPIDEnabled = true;
    }

    // Stops the pivot by zeroing out the desired angle and leaving PID enabled
    public void stopPivot() {
        desiredPivotAngle = 0.0;
        pivotPIDEnabled = true;
    }

    // Spins the intake forward at the configured speed
    public void intakeForward() {
        intakeMotor.set(Constants.AlgaeIntakeConstants.INTAKE_SPEED);
    }

    // Reverses the intake at the same speed
    public void intakeReverse() {
        intakeMotor.set(-Constants.AlgaeIntakeConstants.INTAKE_SPEED);
    }

    // Completely stops the intake mechanism
    public void intakeStop() {
        intakeMotor.set(0.0);
    }

    // Provides the pivot angle for logging or troubleshooting
    public double getPivotAngle() {
        return pivotEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // Pull potential new PID values from the SmartDashboard
        Constants.AlgaeIntakeConstants.PIVOT_kP =
                SmartDashboard.getNumber("Algae Pivot kP", Constants.AlgaeIntakeConstants.PIVOT_kP);
        Constants.AlgaeIntakeConstants.PIVOT_kI =
                SmartDashboard.getNumber("Algae Pivot kI", Constants.AlgaeIntakeConstants.PIVOT_kI);
        Constants.AlgaeIntakeConstants.PIVOT_kD =
                SmartDashboard.getNumber("Algae Pivot kD", Constants.AlgaeIntakeConstants.PIVOT_kD);

        // Push current PID values back to the dashboard for visibility
        SmartDashboard.putNumber("Algae Pivot kP", Constants.AlgaeIntakeConstants.PIVOT_kP);
        SmartDashboard.putNumber("Algae Pivot kI", Constants.AlgaeIntakeConstants.PIVOT_kI);
        SmartDashboard.putNumber("Algae Pivot kD", Constants.AlgaeIntakeConstants.PIVOT_kD);

        // Update our PID controller with any new constants
        pivotPID.setPID(Constants.AlgaeIntakeConstants.PIVOT_kP,
                Constants.AlgaeIntakeConstants.PIVOT_kI, Constants.AlgaeIntakeConstants.PIVOT_kD);

        // Display the pivot’s current and desired angles on the dashboard
        SmartDashboard.putNumber("Algae Pivot Current Angle", getPivotAngle());
        SmartDashboard.putNumber("Algae Pivot Desired Angle", desiredPivotAngle);

        // If the pivot PID is active, use it to reach the target angle
        // if (pivotPIDEnabled) {
        //     // Make sure the desired angle is within our allowed range
        //     if (desiredPivotAngle >= Constants.AlgaeIntakeConstants.PIVOT_MIN_ANGLE
        //             && desiredPivotAngle <= Constants.AlgaeIntakeConstants.PIVOT_MAX_ANGLE) {

        //         double currentAngle = pivotEncoder.getPosition();
        //         double power = pivotPID.calculate(currentAngle, desiredPivotAngle);

        //         // Ensure the motor power doesn’t exceed ±1
        //         if (power > 1) {
        //             power = 1;
        //         } else if (power < -1) {
        //             power = -1;
        //         }
        //         pivotMotor.set(-power / 4);
        //     } else {
        //         // Stop the pivot if the desired angle is out of range
        //         pivotMotor.stopMotor();
        //     }
        // }
        // If pivotPIDEnabled is false, we do nothing here, so the pivot is free-moving.
    }
}
