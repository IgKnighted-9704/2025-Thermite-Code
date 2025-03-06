package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
// --- Rev Robotics "Spark" classes ---
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmElevatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * Subsystem handling:
 * - Elevator (motion profiling)
 * - Arm (PID)
 * - End effector (intake)
 *
 * Demonstrates:
 * - Automatic funnel-intake logic (stops on motor stall).
 * - Collision constraints (arm/elevator).
 * - Tilt-lowering logic (if drivebase pitch is too high).
 */
public class ArmElevatorEndEffectorSubsystem extends SubsystemBase {

    // Elevator
    private final TalonFX elevatorMotorA;
    private final TalonFX elevatorMotorB;
    private final ProfiledPIDController elevatorController;
    private final ElevatorFeedforward elevatorFF;

    // Arm
    private final TalonFX armMotor;
    private final PIDController armPID;

    // Intake
    private final SparkMax intakeMotor; // The new SparkMax object
    private final RelativeEncoder intakeEncoder; // For measuring RPM

    // Desired setpoints (in real units)
    private double desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
    private double desiredElevatorInches = ArmElevatorConstants.ELEVATOR_STOW_INCHES;

    // If you have a drivebase with pitch:
    private final SwerveSubsystem drivebase;

    // --- Intake Logic ---
    private boolean autoIntakeActive = false;
    private boolean manualIntakeActive = false;

    /**
     * Constructor
     * 
     * @param drivebase The drivebase instance for measuring pitch (if available).
     */
    public ArmElevatorEndEffectorSubsystem(SwerveSubsystem drivebase) {
        this.drivebase = drivebase;

        // Elevator motors
        elevatorMotorA = new TalonFX(ArmElevatorConstants.ELEVATOR_MOTOR_A_ID);
        elevatorMotorB = new TalonFX(ArmElevatorConstants.ELEVATOR_MOTOR_B_ID);
        // Configure if needed (e.g., follower mode, neutral modes, etc.)

        // Arm motor
        armMotor = new TalonFX(ArmElevatorConstants.ARM_MOTOR_ID);

        // SparkMax for the intake
        intakeMotor = new SparkMax(ArmElevatorConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

        // Get the built-in relative encoder for the intake motor
        intakeEncoder = intakeMotor.getEncoder();

        // Elevator motion-profiled PID
        elevatorController = new ProfiledPIDController(
                ArmElevatorConstants.ELEVATOR_kP,
                ArmElevatorConstants.ELEVATOR_kI,
                ArmElevatorConstants.ELEVATOR_kD,
                new TrapezoidProfile.Constraints(
                        ArmElevatorConstants.ELEVATOR_MAX_VEL,
                        ArmElevatorConstants.ELEVATOR_MAX_ACC));
        elevatorController.setGoal(ArmElevatorConstants.ELEVATOR_STOW_INCHES);

        // Elevator feedforward
        elevatorFF = new ElevatorFeedforward(
                ArmElevatorConstants.ELEV_kS,
                ArmElevatorConstants.ELEV_kG,
                ArmElevatorConstants.ELEV_kV,
                ArmElevatorConstants.ELEV_kA);

        // Arm PID
        armPID = new PIDController(
                ArmElevatorConstants.ARM_kP,
                ArmElevatorConstants.ARM_kI,
                ArmElevatorConstants.ARM_kD);
    }

    // --------------------------------------------------
    // Preset Positions
    // --------------------------------------------------
    public void intakePosition() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_INTAKE_DEG;
        desiredElevatorInches = ArmElevatorConstants.ELEVATOR_INTAKE_INCHES;
        autoIntakeActive = true;
    }

    public void stowElevator() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
        desiredElevatorInches = ArmElevatorConstants.ELEVATOR_STOW_INCHES;
        autoIntakeActive = false;
    }

    public void goToLevel1Position() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL1_DEG;
        desiredElevatorInches = ArmElevatorConstants.ELEVATOR_LEVEL1_INCHES;
        autoIntakeActive = false;
    }

    public void goToLevel2Position() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL2_DEG;
        desiredElevatorInches = ArmElevatorConstants.ELEVATOR_LEVEL2_INCHES;
        autoIntakeActive = false;
    }

    public void goToLevel3Position() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL3_DEG;
        desiredElevatorInches = ArmElevatorConstants.ELEVATOR_LEVEL3_INCHES;
        autoIntakeActive = false;
    }

    public void goToLevel4Position() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL4_DEG;
        desiredElevatorInches = ArmElevatorConstants.ELEVATOR_LEVEL4_INCHES;
        autoIntakeActive = false;
    }

    // --------------------------------------------------
    // Manual Intake
    // --------------------------------------------------
    /** Manually start intake, only if in funnel position. Otherwise do nothing. */
    public void startManualIntake() {
        manualIntakeActive = true;
        autoIntakeActive = false;
    }

    /** Stops any intake (auto or manual). */
    public void stopIntake() {
        manualIntakeActive = false;
        autoIntakeActive = false;
        intakeMotor.set(0.0);
    }

    // --------------------------------------------------
    // Accessors
    // --------------------------------------------------
    public double getArmAngleDegrees() {
        // Example: TalonFX integrated sensor => .getPosition().getValueAsDouble() is
        // "ticks"
        double armTicks = armMotor.getPosition().getValueAsDouble();
        return armTicks / ArmElevatorConstants.ARM_TICKS_PER_DEG;
    }

    public double getElevatorHeightInches() {
        double elevTicks = elevatorMotorA.getPosition().getValueAsDouble();
        return elevTicks / ArmElevatorConstants.ELEV_TICKS_PER_INCH;
    }

    /** Current intake motor velocity (in RPM). */
    private double getIntakeRPM() {
        return intakeEncoder.getVelocity();
    }

    // --------------------------------------------------
    // PERIODIC
    // --------------------------------------------------
    @Override
    public void periodic() {
        double currentArmDeg = getArmAngleDegrees();
        double currentElevInch = getElevatorHeightInches();

        // 1) Tilt safety
        if (drivebase != null) {
            double pitchDeg = Math.abs(drivebase.getPitch().getDegrees());
            if (pitchDeg > ArmElevatorConstants.TILT_THRESHOLD_DEG) {
                desiredElevatorInches = ArmElevatorConstants.ELEVATOR_MIN_INCHES;
            }
        }

        // 2) Collision avoidance
        // a) If arm behind the robot, ensure elevator in safe range
        if (desiredArmAngleDeg < 0.0) {
            if (desiredElevatorInches < ArmElevatorConstants.ELEV_BEHIND_SAFE_MIN_INCHES
                    || desiredElevatorInches > ArmElevatorConstants.ELEV_BEHIND_SAFE_MAX_INCHES) {
                desiredElevatorInches = ArmElevatorConstants.ELEV_BEHIND_SAFE_MIN_INCHES;
            }
        }
        // b) If elevator going fully down, arm must be stowed
        if (desiredElevatorInches <= ArmElevatorConstants.ELEVATOR_MIN_INCHES + 0.1) {
            if (!isArmInTolerance(ArmElevatorConstants.ARM_STOW_DEG, ArmElevatorConstants.ARM_TOLERANCE_DEG)) {
                desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
            }
        }

        // 3) Clamp setpoints
        double finalArmDeg = clamp(
                desiredArmAngleDeg,
                ArmElevatorConstants.ARM_MIN_DEG,
                ArmElevatorConstants.ARM_MAX_DEG);
        double finalElevInch = clamp(
                desiredElevatorInches,
                ArmElevatorConstants.ELEVATOR_MIN_INCHES,
                ArmElevatorConstants.ELEVATOR_MAX_INCHES);

        // 4) Elevator motion-profile
        elevatorController.setGoal(finalElevInch);
        double elevPIDOut = elevatorController.calculate(currentElevInch);
        double elevFFVolts = elevatorFF.calculate(elevatorController.getSetpoint().velocity);
        double elevatorVolts = elevPIDOut + elevFFVolts;

        // 5) Arm PID
        double armOutput = armPID.calculate(currentArmDeg, finalArmDeg);

        // 6) Command motors
        elevatorMotorA.setVoltage(elevatorVolts);
        elevatorMotorB.setVoltage(elevatorVolts);
        armMotor.set(armOutput);

        // 7) Intake control logic
        if (manualIntakeActive) {
            // If user manually started intake, only run if at funnel position:
            boolean atFunnelArm = isArmInTolerance(ArmElevatorConstants.ARM_INTAKE_DEG,
                    ArmElevatorConstants.ARM_TOLERANCE_DEG);
            boolean atFunnelElev = isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_INTAKE_INCHES,
                    ArmElevatorConstants.ELEVATOR_TOLERANCE_INCH);
            if (atFunnelArm && atFunnelElev) {
                intakeMotor.set(ArmElevatorConstants.INTAKE_SPEED);
            } else {
                // Not in funnel => no intake
                intakeMotor.set(0.0);
            }
        } else if (autoIntakeActive) {
            // Auto-intake if in funnel
            boolean atFunnelArm = isArmInTolerance(ArmElevatorConstants.ARM_INTAKE_DEG,
                    ArmElevatorConstants.ARM_TOLERANCE_DEG);
            boolean atFunnelElev = isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_INTAKE_INCHES,
                    ArmElevatorConstants.ELEVATOR_TOLERANCE_INCH);
            if (atFunnelArm && atFunnelElev) {
                intakeMotor.set(ArmElevatorConstants.INTAKE_SPEED);

                // Stop if the motor "stalls" (RPM too low)
                if (Math.abs(getIntakeRPM()) < ArmElevatorConstants.INTAKE_STOPPED_RPM) {
                    stopIntake(); // also resets autoIntakeActive/manualIntakeActive
                }
            } else {
                // Not in funnel => no intake
                intakeMotor.set(0.0);
            }
        } else {
            // If neither auto nor manual is active => off
            intakeMotor.set(0.0);
        }
    }

    // --------------------------------------------------
    // Private Helpers
    // --------------------------------------------------
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private boolean isArmInTolerance(double targetDeg, double tolDeg) {
        return Math.abs(getArmAngleDegrees() - targetDeg) <= tolDeg;
    }

    private boolean isElevatorInTolerance(double targetInches, double tolInches) {
        return Math.abs(getElevatorHeightInches() - targetInches) <= tolInches;
    }
}
