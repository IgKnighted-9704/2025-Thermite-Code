package frc.robot.subsystems;

import java.util.List;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ArmElevatorConstants;
import frc.robot.Constants.ReefConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.Constants;

public class ArmElevatorEndEffectorSubsystem extends SubsystemBase {

    // Two krakens for the elevator, controlled by a profiled PID controller
    // and a feedforward
    private final TalonFX elevatorMotorA;
    private final TalonFX elevatorMotorB;
    private final ProfiledPIDController elevatorController;
    private ElevatorFeedforward elevatorFF;

    // One kraken motor for the arm, using a standard PID controller
    private final TalonFX armMotor;
    private final PIDController armPID;

    // Funnel/intake mechanism powered by a SparkMax (brushless) and its encoder
    private final SparkMax intakeMotor;
    private final RelativeEncoder intakeEncoder;

    // ───────────────────────────────────────────────
    // absolute encoders for arm & elevator
    // ───────────────────────────────────────────────
    private final SparkAbsoluteEncoder armAbsEnc;

    // Target positions (arm angle and elevator height)
    private double desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
    private double desiredElevInches = ArmElevatorConstants.ELEVATOR_STOW_INCHES;

    // Reference to the swerve drive, mainly for pitch measurement
    private final SwerveSubsystem drivebase;

    // Intake control modes
    private boolean autoIntakeActive = false;
    private boolean manualIntakeActive = false;

    // Stores the last known positions in case the robot tilts and we need to revert
    private double storedArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
    private double storedElevInches = ArmElevatorConstants.ELEVATOR_STOW_INCHES;
    private boolean wasTilted = false;

    public ArmElevatorEndEffectorSubsystem(SwerveSubsystem drivebase) {
        this.drivebase = drivebase;

        // Elevator components
        elevatorMotorA = new TalonFX(ArmElevatorConstants.ELEVATOR_MOTOR_A_ID);
        elevatorMotorB = new TalonFX(ArmElevatorConstants.ELEVATOR_MOTOR_B_ID);

        elevatorController = new ProfiledPIDController(ArmElevatorConstants.ELEVATOR_kP,
                ArmElevatorConstants.ELEVATOR_kI, ArmElevatorConstants.ELEVATOR_kD,
                new TrapezoidProfile.Constraints(ArmElevatorConstants.ELEVATOR_MAX_VEL,
                        ArmElevatorConstants.ELEVATOR_MAX_ACC));
        elevatorController.setGoal(ArmElevatorConstants.ELEVATOR_STOW_INCHES);

        elevatorFF =
                new ElevatorFeedforward(ArmElevatorConstants.ELEV_kS, ArmElevatorConstants.ELEV_kG,
                        ArmElevatorConstants.ELEV_kV, ArmElevatorConstants.ELEV_kA);

        // Arm components
        armMotor = new TalonFX(ArmElevatorConstants.ARM_MOTOR_ID);
        armPID = new PIDController(ArmElevatorConstants.ARM_kP, ArmElevatorConstants.ARM_kI,
                ArmElevatorConstants.ARM_kD);

        // Intake mechanism
        intakeMotor = new SparkMax(ArmElevatorConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
        armAbsEnc = intakeMotor.getAbsoluteEncoder();
    }

    // -------------------------------
    // Preset Positions
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

    // Score positions (new)
    public void goToLevel1ScorePosition() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL1_DEG;
        desiredElevInches = ArmElevatorConstants.ELEVATOR_LEVEL1_SCORE_INCHES;
        autoIntakeActive = false;
    }

    public void goToLevel2ScorePosition() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL2_DEG;
        desiredElevInches = ArmElevatorConstants.ELEVATOR_LEVEL2_SCORE_INCHES;
        autoIntakeActive = false;
    }

    public void goToLevel3ScorePosition() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL3_DEG;
        desiredElevInches = ArmElevatorConstants.ELEVATOR_LEVEL3_SCORE_INCHES;
        autoIntakeActive = false;
    }

    public void goToLevel4ScorePosition() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL4_DEG;
        desiredElevInches = ArmElevatorConstants.ELEVATOR_LEVEL4_SCORE_INCHES;
        autoIntakeActive = false;
    }

    // -----------------------------------------------------------------------
    // Intake (Manual & Auto)
    // -----------------------------------------------------------------------
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
    // Sensor Readouts
    // -------------------------------
    public double getArmAngleDegrees() {
        return armAbsEnc.getPosition() * ArmElevatorConstants.ARM_ABS_ENC_RATIO;
    }

    public double getElevatorHeightInches() {
        double elevTicks = (elevatorMotorA.getPosition().getValueAsDouble()
                + elevatorMotorB.getPosition().getValueAsDouble()) / 2.0;
        return elevTicks / ArmElevatorConstants.ELEV_TICKS_PER_INCH;
    }

    private double getIntakeRPM() {
        return intakeEncoder.getVelocity();
    }

    // ------------------------------------------------------------------------
    // Reef Score Command (new code)
    // Slows for last 0.1m and checks elevator up/down states.
    // ------------------------------------------------------------------------
    public Command createReefScoreCommand(boolean leftBranch, int level) {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
        List<Integer> allianceTags =
                (alliance == DriverStation.Alliance.Red) ? ReefConstants.REEF_RED_IDS
                        : ReefConstants.REEF_BLUE_IDS;

        int visibleTag = findVisibleReefTag(allianceTags);
        if (visibleTag < 0) {
            return Commands
                    .print("[ReefScore] No recognized reef tag is visible for our alliance!");
        }

        double yOffset = leftBranch ? -ReefConstants.BRANCH_OFFSET_METERS
                : ReefConstants.BRANCH_OFFSET_METERS;

        Transform2d approachOffset =
                new Transform2d(new Translation2d(-ReefConstants.APPROACH_X_OFFSET_METERS, yOffset),
                        new Rotation2d());
        Pose2d reefContactPose = Vision.getAprilTagPose(visibleTag, approachOffset);

        Command elevatorNormalCmd;
        Command elevatorScoreCmd;
        switch (level) {
            case 4:
                elevatorNormalCmd = Commands.runOnce(this::goToLevel4Position, this);
                elevatorScoreCmd = Commands.runOnce(this::goToLevel4ScorePosition, this);
                break;
            case 3:
                elevatorNormalCmd = Commands.runOnce(this::goToLevel3Position, this);
                elevatorScoreCmd = Commands.runOnce(this::goToLevel3ScorePosition, this);
                break;
            case 2:
                elevatorNormalCmd = Commands.runOnce(this::goToLevel2Position, this);
                elevatorScoreCmd = Commands.runOnce(this::goToLevel2ScorePosition, this);
                break;
            default: // covers level 1
                elevatorNormalCmd = Commands.runOnce(this::goToLevel1Position, this);
                elevatorScoreCmd = Commands.runOnce(this::goToLevel1ScorePosition, this);
                break;
        }

        Command waitForNormalPos = Commands.waitUntil(() -> isElevatorInTolerance(desiredElevInches,
                ArmElevatorConstants.ELEVATOR_TOLERANCE_INCH));
        Command waitForScorePos = Commands.waitUntil(() -> isElevatorInTolerance(desiredElevInches,
                ArmElevatorConstants.ELEVATOR_TOLERANCE_INCH));

        Command elevatorStow = Commands.runOnce(this::stowElevator, this);
        Command waitForStowPos = Commands
                .waitUntil(() -> isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_STOW_INCHES,
                        ArmElevatorConstants.ELEVATOR_TOLERANCE_INCH));

        Command driveUp = drivebase.twoStepApproach(reefContactPose, 0.1);

        Pose2d retreatPose = reefContactPose.transformBy(new Transform2d(
                new Translation2d(-ReefConstants.RETREAT_DISTANCE_METERS, 0.0), new Rotation2d()));
        Command backAway = drivebase.driveToPose(retreatPose);

        return Commands.sequence(elevatorNormalCmd, waitForNormalPos, driveUp, elevatorScoreCmd,
                waitForScorePos, elevatorStow, waitForStowPos, backAway);
    }

    private int findVisibleReefTag(List<Integer> validTagIds) {
        for (Vision.Cameras cam : Vision.Cameras.values()) {
            var bestResultOpt = cam.getBestResult();
            if (bestResultOpt.isEmpty()) {
                continue;
            }
            var bestTarget = bestResultOpt.get().getBestTarget();
            if (bestTarget != null) {
                int fid = bestTarget.getFiducialId();
                if (validTagIds.contains(fid)) {
                    return fid;
                }
            }
        }
        return -1;
    }

    public void setManualElevatorSpeed(double leftTrigger, double rightTrigger) {
        double speed = rightTrigger - leftTrigger;
        // Clamp speed to [-1,1]
        speed = Math.max(-1.0, Math.min(1.0, speed));
        double volts = 6.0 * speed;
        elevatorMotorA.setVoltage(-volts);
        elevatorMotorB.setVoltage(volts);
    }

    // -------------------------------
    // Main Periodic Update
    // -------------------------------
    @Override
    public void periodic() {
        // ------------------------------------------------
        // 1) Read updated values from the SmartDashboard
        // ------------------------------------------------
        ArmElevatorConstants.ELEVATOR_kP =
                SmartDashboard.getNumber("Elevator kP", ArmElevatorConstants.ELEVATOR_kP);
        ArmElevatorConstants.ELEVATOR_kI =
                SmartDashboard.getNumber("Elevator kI", ArmElevatorConstants.ELEVATOR_kI);
        ArmElevatorConstants.ELEVATOR_kD =
                SmartDashboard.getNumber("Elevator kD", ArmElevatorConstants.ELEVATOR_kD);
        ArmElevatorConstants.ELEVATOR_MAX_VEL = SmartDashboard.getNumber("Elevator MaxVelocity",
                ArmElevatorConstants.ELEVATOR_MAX_VEL);
        ArmElevatorConstants.ELEVATOR_MAX_ACC = SmartDashboard.getNumber("Elevator MaxAccel",
                ArmElevatorConstants.ELEVATOR_MAX_ACC);

        ArmElevatorConstants.ARM_kP =
                SmartDashboard.getNumber("Arm kP", ArmElevatorConstants.ARM_kP);
        ArmElevatorConstants.ARM_kI =
                SmartDashboard.getNumber("Arm kI", ArmElevatorConstants.ARM_kI);
        ArmElevatorConstants.ARM_kD =
                SmartDashboard.getNumber("Arm kD", ArmElevatorConstants.ARM_kD);

        // Feedforward constants
        ArmElevatorConstants.ELEV_kS =
                SmartDashboard.getNumber("Elev kS", ArmElevatorConstants.ELEV_kS);
        ArmElevatorConstants.ELEV_kG =
                SmartDashboard.getNumber("Elev kG", ArmElevatorConstants.ELEV_kG);
        ArmElevatorConstants.ELEV_kV =
                SmartDashboard.getNumber("Elev kV", ArmElevatorConstants.ELEV_kV);
        ArmElevatorConstants.ELEV_kA =
                SmartDashboard.getNumber("Elev kA", ArmElevatorConstants.ELEV_kA);

        // Update the feedforward object with new constants
        elevatorFF =
                new ElevatorFeedforward(ArmElevatorConstants.ELEV_kS, ArmElevatorConstants.ELEV_kG,
                        ArmElevatorConstants.ELEV_kV, ArmElevatorConstants.ELEV_kA);

        // Also read & update additional constants
        ArmElevatorConstants.ARM_ABS_ENC_RATIO = SmartDashboard.getNumber("Arm Abs Encoder Ratio",
                ArmElevatorConstants.ARM_ABS_ENC_RATIO);
        ArmElevatorConstants.ELEV_TICKS_PER_INCH = SmartDashboard.getNumber("Elev Ticks per Inch",
                ArmElevatorConstants.ELEV_TICKS_PER_INCH);

        // ------------------------------------------------
        // 2) Apply updated constants to controllers, etc.
        // ------------------------------------------------
        elevatorController.setP(ArmElevatorConstants.ELEVATOR_kP);
        elevatorController.setI(ArmElevatorConstants.ELEVATOR_kI);
        elevatorController.setD(ArmElevatorConstants.ELEVATOR_kD);
        elevatorController.setConstraints(new TrapezoidProfile.Constraints(
                ArmElevatorConstants.ELEVATOR_MAX_VEL, ArmElevatorConstants.ELEVATOR_MAX_ACC));

        armPID.setP(ArmElevatorConstants.ARM_kP);
        armPID.setI(ArmElevatorConstants.ARM_kI);
        armPID.setD(ArmElevatorConstants.ARM_kD);

        // ------------------------------------------------
        // 3) Continuously send the updated values back to SmartDashboard
        // ------------------------------------------------
        SmartDashboard.putNumber("Elevator kP", ArmElevatorConstants.ELEVATOR_kP);
        SmartDashboard.putNumber("Elevator kI", ArmElevatorConstants.ELEVATOR_kI);
        SmartDashboard.putNumber("Elevator kD", ArmElevatorConstants.ELEVATOR_kD);
        SmartDashboard.putNumber("Elevator MaxVelocity", ArmElevatorConstants.ELEVATOR_MAX_VEL);
        SmartDashboard.putNumber("Elevator MaxAccel", ArmElevatorConstants.ELEVATOR_MAX_ACC);

        SmartDashboard.putNumber("Arm kP", ArmElevatorConstants.ARM_kP);
        SmartDashboard.putNumber("Arm kI", ArmElevatorConstants.ARM_kI);
        SmartDashboard.putNumber("Arm kD", ArmElevatorConstants.ARM_kD);

        SmartDashboard.putNumber("Elev kS", ArmElevatorConstants.ELEV_kS);
        SmartDashboard.putNumber("Elev kG", ArmElevatorConstants.ELEV_kG);
        SmartDashboard.putNumber("Elev kV", ArmElevatorConstants.ELEV_kV);
        SmartDashboard.putNumber("Elev kA", ArmElevatorConstants.ELEV_kA);

        SmartDashboard.putNumber("Arm Abs Encoder Ratio", ArmElevatorConstants.ARM_ABS_ENC_RATIO);
        SmartDashboard.putNumber("Elev Ticks per Inch", ArmElevatorConstants.ELEV_TICKS_PER_INCH);

        // Same for ELEVATOR_MAX_INCHES if you have it

        SmartDashboard.putNumber("Arm Angle (Deg)", getArmAngleDegrees());
        SmartDashboard.putNumber("Elevator Height (In)", getElevatorHeightInches());
        SmartDashboard.putNumber("Arm Desired Position", desiredArmAngleDeg);
        SmartDashboard.putNumber("Elevator Desired Position", desiredElevInches);

        double currentArmDeg = getArmAngleDegrees();
        double currentElevInch = getElevatorHeightInches();

        // // Check for pitch angle from the drivebase and adjust if the robot is tilted
        // if (drivebase != null) {
        // double pitchDeg = Math.abs(drivebase.getPitch().getDegrees());
        // boolean isTiltedNow = (pitchDeg > ArmElevatorConstants.TILT_THRESHOLD_DEG);

        // if (isTiltedNow) {
        // // If this is a new tilt event, record current setpoints
        // if (!wasTilted) {
        // storedArmAngleDeg = desiredArmAngleDeg;
        // storedElevInches = desiredElevInches;
        // }
        // // Force elevator down to a minimum height while tilted
        // desiredElevInches = ArmElevatorConstants.ELEVATOR_MIN_INCHES;
        // } else {
        // // If we were tilted but are level again, restore old setpoints
        // if (wasTilted) {
        // desiredArmAngleDeg = storedArmAngleDeg;
        // desiredElevInches = storedElevInches;
        // }
        // }
        // wasTilted = isTiltedNow;
        // }

        // // Collision protection boundaries
        // double allowedArmMin = ArmElevatorConstants.ARM_MIN_DEG;
        // double allowedArmMax = ArmElevatorConstants.ARM_MAX_DEG;
        // double allowedElevMin = ArmElevatorConstants.ELEVATOR_MIN_INCHES;
        // double allowedElevMax = ArmElevatorConstants.ELEVATOR_MAX_INCHES;

        // // 1) Logic for allowing the arm inside the robot frame
        // boolean wantsInsideRobot = (desiredArmAngleDeg < 0.0);
        // double insideRobotMin = ArmElevatorConstants.ELEV_FUNNEL_SAFE_MIN_INCHES;
        // double insideRobotMax = ArmElevatorConstants.ELEV_FUNNEL_SAFE_MAX_INCHES;

        // if (wantsInsideRobot) {
        // double originalElev = desiredElevInches;
        // double newElev = clamp(originalElev, insideRobotMin, insideRobotMax);
        // // If we had to clamp the elevator, raise the arm's lower limit to 0
        // if (Math.abs(newElev - originalElev) > 0.001) {
        // allowedArmMin = Math.max(allowedArmMin, 0.0);
        // }
        // desiredElevInches = newElev;
        // } else {
        // // If the arm is inside but the elevator tries to move out of funnel range,
        // // clamp
        // boolean armIsInsideRobot = (currentArmDeg < 0.0);
        // boolean elevatorOutOfInsideRobotRange =
        // (desiredElevInches < insideRobotMin) || (desiredElevInches > insideRobotMax);

        // if (armIsInsideRobot && elevatorOutOfInsideRobotRange) {
        // double originalElev = desiredElevInches;
        // double newElev = clamp(originalElev, insideRobotMin, insideRobotMax);
        // desiredElevInches = newElev;
        // allowedArmMin = Math.max(allowedArmMin, 0.0);
        // }
        // }

        // // 2) Prevent elevator from going down if the arm isn't near stow
        // boolean wantsElevDown =
        // (desiredElevInches <= (ArmElevatorConstants.ELEVATOR_MIN_INCHES + 0.01));
        // boolean armOutsideStow = !isArmInTolerance(ArmElevatorConstants.ARM_STOW_DEG,
        // ArmElevatorConstants.ARM_STOW_TOLERANCE_DEG);

        // if (wantsElevDown && armOutsideStow) {
        // allowedElevMin = Math.max(allowedElevMin, currentElevInch);
        // }

        // // 3) Keep the elevator above a safe threshold if the arm is moving out of stow
        // boolean armWantsOutOfStow =
        // (Math.abs(desiredArmAngleDeg) > ArmElevatorConstants.ARM_STOW_TOLERANCE_DEG);
        // boolean elevatorTooLow =
        // (desiredElevInches < ArmElevatorConstants.ELEVATOR_SAFE_LOWER_THRESHOLD);

        // if (armWantsOutOfStow && elevatorTooLow) {
        // double originalElev = desiredElevInches;
        // double newElev = clamp(originalElev, ArmElevatorConstants.ELEVATOR_SAFE_LOWER_THRESHOLD,
        // allowedElevMax);
        // // If we had to clamp elevator, limit the arm within stow tolerance
        // if (Math.abs(newElev - originalElev) > 0.001) {
        // allowedArmMin = -ArmElevatorConstants.ARM_STOW_TOLERANCE_DEG;
        // allowedArmMax = ArmElevatorConstants.ARM_STOW_TOLERANCE_DEG;
        // }
        // desiredElevInches = newElev;
        // }

        // // Clamp final target positions
        // double finalArmDeg = clamp(desiredArmAngleDeg, allowedArmMin, allowedArmMax);
        // double finalElevInch = clamp(desiredElevInches, allowedElevMin, allowedElevMax);

        // // Calculate elevator control output (PID + feedforward)
        // elevatorController.setGoal(finalElevInch);
        // double elevOutput = elevatorController.calculate(currentElevInch);
        // double elevFeedforward = elevatorFF.calculate(elevatorController.getSetpoint().velocity);
        // double totalElevVolts = elevOutput + elevFeedforward;

        // // Calculate arm control output
        // double armOutput = armPID.calculate(currentArmDeg, finalArmDeg);
        double armOutput = armPID.calculate(currentArmDeg, desiredArmAngleDeg) / 2;

        // // Send voltages to the elevator motors
        // elevatorMotorA.setVoltage(totalElevVolts);
        // elevatorMotorB.setVoltage(totalElevVolts);

        // drivebase.setMaximumAllowableSpeeds(
        // Units.feetToMeters(Constants.MAX_SPEED
        // - (getElevatorHeightInches() * ArmElevatorConstants.ACCEL_LIMIT_SCALE)),
        // drivebase.getMaximumChassisAngularVelocity());
        // // Send raw PID output to the arm motor
        armMotor.set(armOutput);

        // // Intake logic
        // if (manualIntakeActive) {
        // // Only run the intake if arm and elevator are at the funnel position
        // boolean armAtFunnel = isArmInTolerance(ArmElevatorConstants.ARM_FUNNEL_DEG,
        // ArmElevatorConstants.ARM_TOLERANCE_DEG);
        // boolean elevatorAtFunnel =
        // isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES,
        // ArmElevatorConstants.ELEVATOR_TOLERANCE_INCH);

        // if (armAtFunnel && elevatorAtFunnel) {
        // intakeMotor.set(ArmElevatorConstants.INTAKE_SPEED);
        // } else {
        // intakeMotor.set(0.0);
        // }
        // } else if (autoIntakeActive) {
        // // Auto intake runs if we are at funnel position; stops if the intake is stalled
        // boolean armAtFunnel = isArmInTolerance(ArmElevatorConstants.ARM_FUNNEL_DEG,
        // ArmElevatorConstants.ARM_TOLERANCE_DEG);
        // boolean elevatorAtFunnel =
        // isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES,
        // ArmElevatorConstants.ELEVATOR_TOLERANCE_INCH);

        // if (armAtFunnel && elevatorAtFunnel) {
        // intakeMotor.set(ArmElevatorConstants.INTAKE_SPEED);

        // // Check for a stall condition, then shut off intake
        // if (Math.abs(getIntakeRPM()) < ArmElevatorConstants.INTAKE_STOPPED_RPM) {
        // stopIntake();
        // }
        // } else {
        // intakeMotor.set(0.0);
        // }
        // } else {
        // intakeMotor.set(0.0);
        // }
    }

    // Ensures a value stays within [min, max]
    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    // Checks if the arm is close enough to a target angle within some tolerance
    private boolean isArmInTolerance(double targetDeg, double toleranceDeg) {
        return Math.abs(getArmAngleDegrees() - targetDeg) <= toleranceDeg;
    }

    // Checks if the elevator is close enough to a target height within some
    // tolerance
    private boolean isElevatorInTolerance(double targetIn, double toleranceIn) {
        return Math.abs(getElevatorHeightInches() - targetIn) <= toleranceIn;
    }
}
