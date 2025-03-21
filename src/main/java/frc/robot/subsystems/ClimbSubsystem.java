package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

    // Primary climb motor (brushed)
    private final SparkMax climbMotor =
            new SparkMax(Constants.ClimbConstants.CLIMB_MOTOR_A_ID, MotorType.kBrushed);

    // Secondary pivot motor (brushed) with an absolute encoder
    private final SparkMax climbMotor2 =
            new SparkMax(Constants.ClimbConstants.CLIMB_MOTOR_B_ID, MotorType.kBrushed);

    public ClimbSubsystem() {
        // Could configure current limits or idle modes here if desired
    }

    // // Moves the pivot to a preset angle and enables the PID to hold that position
    // public void setPivotToClimbAngle() {
    // desiredAngle = Constants.ClimbConstants.CLIMB_HOLD_ANGLE;
    // pidEnabled = true;
    // }

    // // Manually adjusts the pivot angle
    // public void changeClimbAngle(double speed) {
    // pidEnabled = false; // turn off PID during manual control
    // double currPos = climbAngleEncoder.getPosition();

    // if (currPos < Constants.ClimbConstants.CLIMB_MAX_POS
    // && currPos > Constants.ClimbConstants.CLIMB_MIN_POS) {
    // climbAngle.set(speed);
    // } else {
    // climbAngle.stopMotor();
    // }
    // }

    /**
     * Runs both climb motors at a speed based on the difference between right and left triggers.
     * Speeds are clamped between -1 and 1.
     */
    public void climb(double leftTrigger, double rightTrigger) {
        double speed = rightTrigger - leftTrigger;
        // Constrain speed to [-1, 1]
        speed = Math.max(-1.0, Math.min(1.0, speed));
        double volts = 12 * speed;
        climbMotor.setVoltage(volts);
        climbMotor2.setVoltage(volts);
    }

    /** Immediately stops both climb motors. */
    public void stopClimbMotor() {
        climbMotor.stopMotor();
        climbMotor2.stopMotor();
    }

    // // Returns the current pivot angle (for logging or diagnostics)
    // public double getArmPos() {
    // return climbAngleEncoder.getPosition();
    // }

    @Override
    public void periodic() {
        // // If PID was enabled, we would hold the pivot at the desired angle here.
        // // If it's disabled, we rely on manual or halted behavior.
        // if (pidEnabled) {
        // // Only run PID if the desired angle is within valid limits
        // if (desiredAngle >= Constants.ClimbConstants.CLIMB_MIN_POS
        // && desiredAngle <= Constants.ClimbConstants.CLIMB_MAX_POS) {

        // double currentAngle = climbAngleEncoder.getPosition();
        // double output = climbPID.calculate(currentAngle, desiredAngle);

        // // Constrain output between -1 and 1
        // if (output > 1) {
        // output = 1;
        // } else if (output < -1) {
        // output = -1;
        // }
        // climbAngle.set(output);
        // } else {
        // // Stop if we're outside the acceptable range
        // climbAngle.stopMotor();
        // }
        // }
    }
}
