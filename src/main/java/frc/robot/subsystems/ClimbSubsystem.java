package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

    // Main climb motor
    private final SparkMax climbMotor =
            new SparkMax(Constants.ClimbConstants.CLIMB_MOTOR_A_ID, MotorType.kBrushed);

    // Pivot motor (brushed) equipped with an absolute encoder
    private final SparkMax climbAngle =
            new SparkMax(Constants.ClimbConstants.CLIMB_MOTOR_B_ID, MotorType.kBrushed);

    private final AbsoluteEncoder climbAngleEncoder = climbAngle.getAbsoluteEncoder();

    // PID controller that manages pivot angle
    private final PIDController climbPID = new PIDController(Constants.ClimbConstants.CLIMB_kP,
            Constants.ClimbConstants.CLIMB_kI, Constants.ClimbConstants.CLIMB_kD);

    // Tracks the target pivot angle and whether the PID is actively holding it
    private double desiredAngle = 0.0;
    private boolean pidEnabled = false;

    public ClimbSubsystem() {
        // Optionally set idle modes or current limits here, for example:
        // climbMotor.setIdleMode(IdleMode.kBrake);
        // climbAngle.setIdleMode(IdleMode.kBrake);
    }

    // Moves the pivot to a preset angle and enables the PID to hold that position
    public void setPivotToClimbAngle() {
        desiredAngle = Constants.ClimbConstants.CLIMB_HOLD_ANGLE;
        pidEnabled = true;
    }

    // Manually adjusts the pivot angle
    public void changeClimbAngle(double speed) {
        pidEnabled = false; // turn off PID during manual control
        double currPos = climbAngleEncoder.getPosition();

        if (currPos < Constants.ClimbConstants.CLIMB_MAX_POS
                && currPos > Constants.ClimbConstants.CLIMB_MIN_POS) {
            climbAngle.set(speed);
        } else {
            climbAngle.stopMotor();
        }
    }

    // Runs the main climb motor at a set speed
    public void climb() {
        climbMotor.set(0.35);
    }

    // Immediately stops the climb motor
    public void stopClimbMotor() {
        climbMotor.stopMotor();
    }

    // Returns the current pivot angle for logging or diagnostics
    public double getArmPos() {
        return climbAngleEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // If PID is enabled, hold the pivot at the desired angle
        if (pidEnabled) {
            // Only run PID if the desired angle is within valid limits
            if (desiredAngle >= Constants.ClimbConstants.CLIMB_MIN_POS
                    && desiredAngle <= Constants.ClimbConstants.CLIMB_MAX_POS) {

                double currentAngle = climbAngleEncoder.getPosition();
                double output = climbPID.calculate(currentAngle, desiredAngle);

                // Clamp motor output between [-1, 1]
                if (output > 1) {
                    output = 1;
                } else if (output < -1) {
                    output = -1;
                }
                climbAngle.set(output);
            } else {
                // Stop if we're outside the acceptable range
                climbAngle.stopMotor();
            }
        }
        // If PID is disabled, we do nothing here and rely on manual controls or a
        // stopped motor.
    }
}
