package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

    // Main climb motor (e.g. spool/winch)
    private final SparkMax climbMotor = new SparkMax(
            Constants.ClimbConstants.CLIMB_MOTOR_A_ID,
            MotorType.kBrushed);

    // Pivot motor with absolute encoder (both brushed)
    private final SparkMax climbAngle = new SparkMax(
            Constants.ClimbConstants.CLIMB_MOTOR_B_ID,
            MotorType.kBrushed);

    private final AbsoluteEncoder climbAngleEncoder = climbAngle.getAbsoluteEncoder();

    // PID for pivot angle
    private final PIDController climbPID = new PIDController(
            Constants.ClimbConstants.CLIMB_kP,
            Constants.ClimbConstants.CLIMB_kI,
            Constants.ClimbConstants.CLIMB_kD);

    // Desired pivot angle and whether PID is enabled
    private double desiredAngle = 0.0;
    private boolean pidEnabled = false;

    public ClimbSubsystem() {
        // Optional: set idle modes or current limits here
        // For example:
        // climbMotor.setIdleMode(IdleMode.kBrake);
        // climbAngle.setIdleMode(IdleMode.kBrake);
    }

    // Moves pivot to a preset climb angle, enabling PID to hold it
    public void setPivotToClimbAngle() {
        desiredAngle = Constants.ClimbConstants.CLIMB_HOLD_ANGLE;
        pidEnabled = true;
    }

    // Manually move pivot, disabling PID so it won't fight the operator
    public void changeClimbAngle(double speed) {
        pidEnabled = false; // turn off PID while in manual control
        double currPos = climbAngleEncoder.getPosition();
        if (currPos < Constants.ClimbConstants.CLIMB_MAX_POS
                && currPos > Constants.ClimbConstants.CLIMB_MIN_POS) {
            climbAngle.set(speed);
        } else {
            climbAngle.stopMotor();
        }
    }

    // Spins the main climb motor (e.g., spool or winch)
    public void climb() {
        climbMotor.set(0.35);
    }

    public void stopClimbMotor() {
        climbMotor.stopMotor();
    }

    // Returns the current pivot angle for logging/debug
    public double getArmPos() {
        return climbAngleEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // If PID is enabled, hold the pivot at desiredAngle
        if (pidEnabled) {
            // Only do PID if angle is within valid range
            if (desiredAngle >= Constants.ClimbConstants.CLIMB_MIN_POS
                    && desiredAngle <= Constants.ClimbConstants.CLIMB_MAX_POS) {
                double current = climbAngleEncoder.getPosition();
                double power = climbPID.calculate(current, desiredAngle);

                // Clamp power to [-1, 1]
                if (power > 1) {
                    power = 1;
                } else if (power < -1) {
                    power = -1;
                }
                climbAngle.set(power);
            } else {
                // If out of range, stop the pivot
                climbAngle.stopMotor();
            }
        }
        // If pidEnabled is false, we do nothing here;
        // the pivot is either manual or stopped.
    }
}
