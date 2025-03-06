package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmElevatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ArmElevatorEndEffectorSubsystem extends SubsystemBase {

    // Elevator
    private final TalonFX elevatorMotorA;
    private final TalonFX elevatorMotorB;
    private final ProfiledPIDController elevatorController;
    private final ElevatorFeedforward elevatorFF;

    // Arm
    private final TalonFX armMotor;
    private final PIDController armPID;

    // Funnel / Intake
    private final SparkMax intakeMotor;
    private final RelativeEncoder intakeEncoder;

    // Desired positions
    private double desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
    private double desiredElevInches = ArmElevatorConstants.ELEVATOR_STOW_INCHES;

    private final SwerveSubsystem drivebase;

    // Intake modes
    private boolean autoIntakeActive = false;
    private boolean manualIntakeActive = false;

    // Track setpoints before tilt forces elevator down
    private double storedArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
    private double storedElevInches = ArmElevatorConstants.ELEVATOR_STOW_INCHES;
    private boolean wasTilted = false;

    public ArmElevatorEndEffectorSubsystem(SwerveSubsystem drivebase) {
        this.drivebase = drivebase;

        // Elevator
        elevatorMotorA = new TalonFX(ArmElevatorConstants.ELEVATOR_MOTOR_A_ID);
        elevatorMotorB = new TalonFX(ArmElevatorConstants.ELEVATOR_MOTOR_B_ID);

        elevatorController = new ProfiledPIDController(
                ArmElevatorConstants.ELEVATOR_kP,
                ArmElevatorConstants.ELEVATOR_kI,
                ArmElevatorConstants.ELEVATOR_kD,
                new TrapezoidProfile.Constraints(
                        ArmElevatorConstants.ELEVATOR_MAX_VEL,
                        ArmElevatorConstants.ELEVATOR_MAX_ACC));
        elevatorController.setGoal(ArmElevatorConstants.ELEVATOR_STOW_INCHES);

        elevatorFF = new ElevatorFeedforward(
                ArmElevatorConstants.ELEV_kS,
                ArmElevatorConstants.ELEV_kG,
                ArmElevatorConstants.ELEV_kV,
                ArmElevatorConstants.ELEV_kA);

        // Arm
        armMotor = new TalonFX(ArmElevatorConstants.ARM_MOTOR_ID);
        armPID = new PIDController(
                ArmElevatorConstants.ARM_kP,
                ArmElevatorConstants.ARM_kI,
                ArmElevatorConstants.ARM_kD);

        // Funnel intake
        intakeMotor = new SparkMax(ArmElevatorConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
    }

    // -------------------------------
    // Position Presets
    // -------------------------------
    public void funnelPosition() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_FUNNEL_DEG;
        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES;
        autoIntakeActive = true;
    }

    public void stowElevator() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
        desiredElevInches = ArmElevatorConstants.ELEVATOR_STOW_INCHES;
        autoIntakeActive = false;
    }

    public void goToLevel1Position() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL1_DEG;
        desiredElevInches = ArmElevatorConstants.ELEVATOR_LEVEL1_INCHES;
        autoIntakeActive = false;
    }

    public void goToLevel2Position() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL2_DEG;
        desiredElevInches = ArmElevatorConstants.ELEVATOR_LEVEL2_INCHES;
        autoIntakeActive = false;
    }

    public void goToLevel3Position() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL3_DEG;
        desiredElevInches = ArmElevatorConstants.ELEVATOR_LEVEL3_INCHES;
        autoIntakeActive = false;
    }

    public void goToLevel4Position() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL4_DEG;
        desiredElevInches = ArmElevatorConstants.ELEVATOR_LEVEL4_INCHES;
        autoIntakeActive = false;
    }

    // -------------------------------
    // Funnel / Intake
    // -------------------------------
    public void startManualIntake() {
        manualIntakeActive = true;
        autoIntakeActive = false;
    }

    public void stopIntake() {
        manualIntakeActive = false;
        autoIntakeActive = false;
        intakeMotor.set(0.0);
    }

    // -------------------------------
    // Readouts
    // -------------------------------
    public double getArmAngleDegrees() {
        double armTicks = armMotor.getPosition().getValueAsDouble();
        return armTicks / ArmElevatorConstants.ARM_TICKS_PER_DEG;
    }

    public double getElevatorHeightInches() {
        double elevTicks = elevatorMotorA.getPosition().getValueAsDouble();
        return elevTicks / ArmElevatorConstants.ELEV_TICKS_PER_INCH;
    }

    private double getIntakeRPM() {
        return intakeEncoder.getVelocity();
    }

    // -------------------------------
    // Periodic Updates
    // -------------------------------
    @Override
    public void periodic() {
        double currentArmDeg = getArmAngleDegrees();
        double currentElevInch = getElevatorHeightInches();

        // Check if we're tilted and force elevator down if needed
        if (drivebase != null) {
            double pitchDeg = Math.abs(drivebase.getPitch().getDegrees());
            boolean isTiltedNow = (pitchDeg > ArmElevatorConstants.TILT_THRESHOLD_DEG);

            if (isTiltedNow) {
                // If we weren't tilted before, store our current desired positions
                if (!wasTilted) {
                    storedArmAngleDeg = desiredArmAngleDeg;
                    storedElevInches = desiredElevInches;
                }
                // Force elevator down while tilted
                desiredElevInches = ArmElevatorConstants.ELEVATOR_MIN_INCHES;
            } else {
                // If we just went back to level, restore the old setpoints
                if (wasTilted) {
                    desiredArmAngleDeg = storedArmAngleDeg;
                    desiredElevInches = storedElevInches;
                }
            }
            wasTilted = isTiltedNow;
        }

        // Collision checks
        double allowedArmMin = ArmElevatorConstants.ARM_MIN_DEG;
        double allowedArmMax = ArmElevatorConstants.ARM_MAX_DEG;
        double allowedElevMin = ArmElevatorConstants.ELEVATOR_MIN_INCHES;
        double allowedElevMax = ArmElevatorConstants.ELEVATOR_MAX_INCHES;

        // 1) Inside robot logic
        boolean wantsInsideRobot = (desiredArmAngleDeg < 0.0);
        double insideRobotMin = ArmElevatorConstants.ELEV_FUNNEL_SAFE_MIN_INCHES;
        double insideRobotMax = ArmElevatorConstants.ELEV_FUNNEL_SAFE_MAX_INCHES;

        if (wantsInsideRobot) {
            double origElev = desiredElevInches;
            double newElev = clamp(origElev, insideRobotMin, insideRobotMax);
            if (Math.abs(newElev - origElev) > 0.001) {
                allowedArmMin = Math.max(allowedArmMin, 0.0);
            }
            desiredElevInches = newElev;
        } else {
            boolean armIsInsideRobot = (currentArmDeg < 0.0);
            boolean elevatorOutOfInsideRobot = (desiredElevInches < insideRobotMin)
                    || (desiredElevInches > insideRobotMax);

            if (armIsInsideRobot && elevatorOutOfInsideRobot) {
                double origElev = desiredElevInches;
                double newElev = clamp(origElev, insideRobotMin, insideRobotMax);
                desiredElevInches = newElev;
                allowedArmMin = Math.max(allowedArmMin, 0.0);
            }
        }

        // 2) Elevator-down logic
        boolean wantsElevDown = (desiredElevInches <= (ArmElevatorConstants.ELEVATOR_MIN_INCHES + 0.01));
        boolean armOutsideStow = !isArmInTolerance(
                ArmElevatorConstants.ARM_STOW_DEG,
                ArmElevatorConstants.ARM_STOW_TOLERANCE_DEG);

        if (wantsElevDown && armOutsideStow) {
            allowedElevMin = Math.max(allowedElevMin, currentElevInch);
        }

        // 3) Arm-out logic
        boolean armWantsOutOfStow = (Math.abs(desiredArmAngleDeg) > ArmElevatorConstants.ARM_STOW_TOLERANCE_DEG);
        boolean elevatorTooLow = (desiredElevInches < ArmElevatorConstants.ELEVATOR_SAFE_LOWER_THRESHOLD);

        if (armWantsOutOfStow && elevatorTooLow) {
            double origElev = desiredElevInches;
            double newElev = clamp(origElev, ArmElevatorConstants.ELEVATOR_SAFE_LOWER_THRESHOLD, allowedElevMax);
            if (Math.abs(newElev - origElev) > 0.001) {
                allowedArmMin = -ArmElevatorConstants.ARM_STOW_TOLERANCE_DEG;
                allowedArmMax = ArmElevatorConstants.ARM_STOW_TOLERANCE_DEG;
            }
            desiredElevInches = newElev;
        }

        // Final clamp
        double finalArmDeg = clamp(desiredArmAngleDeg, allowedArmMin, allowedArmMax);
        double finalElevInch = clamp(desiredElevInches, allowedElevMin, allowedElevMax);

        // Elevator control
        elevatorController.setGoal(finalElevInch);
        double elevOut = elevatorController.calculate(currentElevInch);
        double elevFF = elevatorFF.calculate(elevatorController.getSetpoint().velocity);
        double elevVolts = elevOut + elevFF;

        // Arm control
        double armOutput = armPID.calculate(currentArmDeg, finalArmDeg);

        elevatorMotorA.setVoltage(elevVolts);
        elevatorMotorB.setVoltage(elevVolts);
        armMotor.set(armOutput);

        // Intake logic
        if (manualIntakeActive) {
            boolean atFunnelArm = isArmInTolerance(ArmElevatorConstants.ARM_FUNNEL_DEG,
                    ArmElevatorConstants.ARM_TOLERANCE_DEG);
            boolean atFunnelElev = isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES,
                    ArmElevatorConstants.ELEVATOR_TOLERANCE_INCH);

            if (atFunnelArm && atFunnelElev) {
                intakeMotor.set(ArmElevatorConstants.INTAKE_SPEED);
            } else {
                intakeMotor.set(0.0);
            }
        } else if (autoIntakeActive) {
            boolean atFunnelArm = isArmInTolerance(ArmElevatorConstants.ARM_FUNNEL_DEG,
                    ArmElevatorConstants.ARM_TOLERANCE_DEG);
            boolean atFunnelElev = isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES,
                    ArmElevatorConstants.ELEVATOR_TOLERANCE_INCH);

            if (atFunnelArm && atFunnelElev) {
                intakeMotor.set(ArmElevatorConstants.INTAKE_SPEED);

                if (Math.abs(getIntakeRPM()) < ArmElevatorConstants.INTAKE_STOPPED_RPM) {
                    stopIntake();
                }
            } else {
                intakeMotor.set(0.0);
            }
        } else {
            intakeMotor.set(0.0);
        }
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    private boolean isArmInTolerance(double targetDeg, double toleranceDeg) {
        return Math.abs(getArmAngleDegrees() - targetDeg) <= toleranceDeg;
    }

    private boolean isElevatorInTolerance(double targetIn, double toleranceIn) {
        return Math.abs(getElevatorHeightInches() - targetIn) <= toleranceIn;
    }
}
