package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

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
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;


public class ArmElevatorEndEffectorSubsystem extends SubsystemBase {

    // The elevator is driven by two TalonFX motors,
    // using a profiled PID controller and a feedforward for control.
    private final TalonFX elevatorMotorA;
    private final TalonFX elevatorMotorB;
    private final ProfiledPIDController elevatorController;
    private ElevatorFeedforward elevatorFF;

    // The arm is powered by a single TalonFX motor,
    // regulated by a standard PID controller.
    private final TalonFX armMotor;
    private final PIDController armPID;

    // The intake/funnel system is driven by a SparkMax (brushless),
    // with a relative encoder plus an absolute encoder on the arm mechanism.
    private final SparkMax intakeMotor;
    private final RelativeEncoder intakeEncoder;
    private final SparkAbsoluteEncoder armAbsEnc;

    // We store the current targets for the arm angle and elevator position.
    private double desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
    private double desiredElevInches = ArmElevatorConstants.ELEVATOR_STOW_INCHES;

    // Reference to our swerve drive system (used for pitch/pose data as needed).
    private final SwerveSubsystem drivebase;

    // Flags for controlling intake states: automatic, manual, outtaking, etc.
    private boolean autoIntakeActive = false;
    private boolean manualIntakeActive = false;
    private boolean manualElevator = false;
    private boolean manualArm = false;
    private boolean outtake = false;

    // We track which preset the mechanism is in (e.g., STOW, FUNNEL, LOADING, or a LEVEL).
    private enum Preset {
        STOW, FUNNEL, LOADING, LEVEL1, LEVEL2, LEVEL3, LEVEL4, LEVEL1SCORE, LEVEL2SCORE, LEVEL3SCORE, LEVEL4SCORE
    }

    // We assume the system starts stowed.
    private Preset currentPreset = Preset.STOW;

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

        // Intake/funnel mechanism
        intakeMotor = new SparkMax(ArmElevatorConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
        armAbsEnc = intakeMotor.getAbsoluteEncoder();

        // Optionally zero motor positions if necessary
        elevatorMotorA.setPosition(0);
        elevatorMotorB.setPosition(0);
        armMotor.setPosition(armAbsEnc.getPosition());
    }

    // --------------------------------------------------------------------------
    // Commands to Move Between Presets (Stow, Funnel, Loading, Levels)
    // --------------------------------------------------------------------------

    /**
     * Moves from the current preset to the funnel preset, sequencing arm vs. elevator to prevent
     * collisions.
     */
    public Command goToFunnelCommand() {
        return Commands.sequence(
                // Decide which part moves first based on the current preset
                Commands.runOnce(() -> {
                    if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                        // Leaving STOW or LEVEL => elevator first
                        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES;
                    } else {
                        // Leaving FUNNEL or LOADING => arm first
                        desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
                    }
                }),
                // Wait until the first part is near its target
                Commands.waitUntil(() -> {
                    if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                        return isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES,
                                2.0);
                    } else {
                        return isArmInTolerance(ArmElevatorConstants.ARM_STOW_DEG, 2.0);
                    }
                }),
                // Then move the second piece and enable auto intake
                Commands.runOnce(() -> {
                    if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                        desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
                    } else {
                        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES;
                    }
                    autoIntakeActive = true;
                }), Commands.runOnce(() -> currentPreset = Preset.FUNNEL));
    }

    /**
     * Moves from the current preset to the loading preset. Similar to funnel logic, but goes to a
     * different target.
     */
    public Command goToLoadingCommand() {
        return Commands.sequence(Commands.runOnce(() -> {
            // If we are in FUNNEL, adjust the arm angle first
            if (currentPreset == Preset.FUNNEL) {
                desiredArmAngleDeg = ArmElevatorConstants.ARM_FUNNEL_DEG;
            }
        }), Commands.waitSeconds(0.5), Commands.runOnce(() -> {
            // If we’re in STOW or a level preset, move the elevator first
            if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES;
            } else {
                // Otherwise, move the arm first
                desiredArmAngleDeg = ArmElevatorConstants.ARM_LOADING_DEG;
            }
        }), Commands.waitUntil(() -> {
            if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                return isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES,
                        2.0);
            } else {
                return isArmInTolerance(ArmElevatorConstants.ARM_LOADING_DEG, 2.0);
            }
        }), Commands.runOnce(() -> {
            // Second movement (arm or elevator)
            if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                desiredArmAngleDeg = ArmElevatorConstants.ARM_LOADING_DEG;
            } else {
                desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES;
            }
            autoIntakeActive = true;
        }), Commands.runOnce(() -> currentPreset = Preset.LOADING));
    }

    /**
     * Command that brings the system back to STOW. If we’re leaving FUNNEL or LOADING, the arm
     * moves first; otherwise the elevator goes first.
     */
    public Command goToStowCommand() {
        return Commands.sequence(Commands.runOnce(() -> {
            // if (currentPreset == Preset.FUNNEL || currentPreset == Preset.LOADING) {
            desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
            // } else {
            // desiredElevInches = ArmElevatorConstants.ELEVATOR_STOW_INCHES;
            // }
        }), Commands.waitUntil(() -> {
            // if (currentPreset == Preset.FUNNEL || currentPreset == Preset.LOADING) {
            return isArmInTolerance(ArmElevatorConstants.ARM_STOW_DEG, 2.0);
            // } else {
            // return isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_STOW_INCHES, 2.0);
            // }
        }), Commands.runOnce(() -> {
            // Second stage (arm or elevator)
            // if (currentPreset == Preset.FUNNEL || currentPreset == Preset.LOADING) {
            desiredElevInches = ArmElevatorConstants.ELEVATOR_STOW_INCHES;
            // } else {
            // desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
            // }
            autoIntakeActive = false;
        }), Commands.runOnce(() -> currentPreset = Preset.STOW));
    }

    /**
     * Generic method to move to a level N preset from the current preset. If we are currently in
     * FUNNEL, we do a special funnel->level logic first; otherwise, we go directly.
     */
    public Command goToLevelCommand(int level) {

        if (currentPreset == Preset.FUNNEL) {
            // We do a funnel->loading approach, then normal "loading->level"
            return Commands.sequence(Commands.runOnce(() -> {
                // If we are in FUNNEL, adjust the arm angle first
                desiredArmAngleDeg = ArmElevatorConstants.ARM_FUNNEL_DEG;
            }), Commands.waitSeconds(0.5),
                    // 1) Turn on manual intake
                    Commands.runOnce(() -> {
                        manualIntakeActive = true;
                        outtake = false;
                    }),
                    // 2) Elevator partially up to loading
                    Commands.runOnce(() -> {
                        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES;
                    }), Commands.race(Commands.waitSeconds(0.5)),
                    // Commands.race(Commands.waitSeconds(0.5)),
                    // 3) Wait for an intake stall or 0.5s
                    Commands.race(
                            Commands.waitUntil(
                                    intakeStallDetector(ArmElevatorConstants.INTAKE_STOPPED_RPM)),
                            Commands.waitSeconds(0.5)),
                    // 4) Slow intake
                    Commands.runOnce(() -> slowIntake()),
                    // Move elevator to funnel height
                    Commands.runOnce(() -> {
                        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES;
                    }), Commands.race(Commands.waitSeconds(1)),
                    Commands.race(
                            Commands.waitUntil(() -> isElevatorInTolerance(
                                    ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES, 2.0)),
                            Commands.waitSeconds(1)),

                    // Large commented-out block remains unchanged except comment style
                    // 5) Move arm first
                    // Commands.runOnce(() -> {
                    // desiredArmAngleDeg = ArmElevatorConstants.ARM_LOADING_DEG;
                    // }),
                    // // 6) Wait for arm
                    // Commands.waitUntil(
                    // () -> isArmInTolerance(ArmElevatorConstants.ARM_LOADING_DEG, 2.0)),
                    // // 7) Then elevator to loading again (if needed)
                    // Commands.runOnce(() -> {
                    // desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES;
                    // }),
                    // Commands.waitUntil(() -> isElevatorInTolerance(
                    // ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES, 2.0)),

                    // Finally do the normal loading->level N movement
                    Commands.runOnce(() -> {
                    }), goToLevelFromLoadingCommand(level));
        } else {
            // If not in FUNNEL, just do the normal approach
            currentPreset = getPresetForLevel(level);
            return goToLevelFromLoadingCommand(level);
        }
    }

    /**
     * Internal helper that assumes we’re basically in a loading style and need to move to a
     * specified level. Depending on the current preset, we might move the elevator or the arm
     * first.
     */
    private Command goToLevelFromLoadingCommand(int level) {
        return Commands.sequence(Commands.runOnce(() -> {
            if (currentPreset == Preset.LOADING || currentPreset == Preset.FUNNEL) {
                desiredArmAngleDeg = getArmAngleForLevel(level);
            } else {
                currentPreset = getPresetForLevel(level);
                desiredElevInches = getElevatorInchesForLevel(level);
            }
        }), Commands.race(Commands.waitUntil(() -> {
            if (currentPreset == Preset.LOADING || currentPreset == Preset.FUNNEL) {
                return isArmInTolerance(getArmAngleForLevel(level), 2.0);
            } else {
                return isElevatorInTolerance(getElevatorInchesForLevel(level), 2.0);
            }
        }), Commands.waitSeconds(1)), Commands.runOnce(() -> {
            if (currentPreset == Preset.LOADING || currentPreset == Preset.FUNNEL) {
                desiredElevInches = getElevatorInchesForLevel(level);
            } else {
                desiredArmAngleDeg = getArmAngleForLevel(level);
            }
            autoIntakeActive = false;
        }), Commands.runOnce(() -> {
            currentPreset = getPresetForLevel(level);
        }));
    }

    // --------------------------------------------------------------------------
    // Score Logic
    // --------------------------------------------------------------------------

    /**
     * Universal method for going to a score position for level N. In principle, it follows the same
     * collision-avoidance logic used in goToLevelCommand(...), but adapted for scoring
     * heights/angles.
     */
    public Command goToLevelScoreCommand(int level) {
        // For now, we skip the funnel->loading logic and go straight:
        return goToScoreFromLoadingCommand(level);
    }

    /**
     * Helper for funnel/loading -> score position transitions. Determines which part to move first,
     * sets the preset, etc.
     */
    private Command goToScoreFromLoadingCommand(int level) {
        // return Commands.sequence(Commands.runOnce(() -> {
        // // If leaving funnel/loading => arm first
        // // Otherwise stow/level => elevator first
        // if (currentPreset == Preset.LOADING || currentPreset == Preset.FUNNEL) {
        // desiredArmAngleDeg = getArmAngleForLevel(level);
        // } else {
        // desiredElevInches = getElevatorInchesForScoreLevel(level);
        // }
        // }),
        // Commands.waitUntil(() -> {
        // if (currentPreset == Preset.LOADING || currentPreset == Preset.FUNNEL) {
        // return isArmInTolerance(getArmAngleForLevel(level), 2.0);
        // } else {
        // return isElevatorInTolerance(getElevatorInchesForScoreLevel(level), 2.0);
        // }
        // }),
        // Commands.runOnce(() -> {
        // if (currentPreset == Preset.LOADING || currentPreset == Preset.FUNNEL) {
        // desiredElevInches = getElevatorInchesForScoreLevel(level);
        // } else {
        // desiredArmAngleDeg = getArmAngleForLevel(level);
        // }
        // // Possibly disable autoIntake or enable outtake
        // currentPreset = getScorePresetForLevel(level);
        // }));

        // desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
        // currentPreset = getScorePresetForLevel(level);
        return Commands.sequence(Commands.runOnce(() -> {
            if (level == 2)
                desiredArmAngleDeg = ArmElevatorConstants.ARM_SCORE_DEG;
            else
                desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
            currentPreset = getScorePresetForLevel(level);
        }), Commands.waitSeconds(1),
                // Commands.runOnce(() -> startManualOuttake()),
                Commands.waitSeconds(1), Commands.runOnce(() -> stopIntake()));
    }

    // --------------------------------------------------------------------------
    // Normal and Score Angles/Positions for Arm and Elevator
    // --------------------------------------------------------------------------

    /** Returns the standard arm angle (in degrees) for a given level. */
    private double getArmAngleForLevel(int level) {
        switch (level) {
            case 1:
                return ArmElevatorConstants.ARM_LEVEL1_DEG;
            case 2:
                return ArmElevatorConstants.ARM_LEVEL2_DEG;
            case 3:
                return ArmElevatorConstants.ARM_LEVEL3_DEG;
            case 4:
                return ArmElevatorConstants.ARM_LEVEL4_DEG;
            default:
                return ArmElevatorConstants.ARM_LEVEL1_DEG;
        }
    }

    /** Returns the standard elevator position (in inches) for a given level. */
    private double getElevatorInchesForLevel(int level) {
        switch (level) {
            case 1:
                return ArmElevatorConstants.ELEVATOR_LEVEL1_INCHES;
            case 2:
                return ArmElevatorConstants.ELEVATOR_LEVEL2_INCHES;
            case 3:
                return ArmElevatorConstants.ELEVATOR_LEVEL3_INCHES;
            case 4:
                return ArmElevatorConstants.ELEVATOR_LEVEL4_INCHES;
            default:
                return ArmElevatorConstants.ELEVATOR_LEVEL1_INCHES;
        }
    }

    /** Returns the elevator position used for score mode at a given level. */
    private double getElevatorInchesForScoreLevel(int level) {
        switch (level) {
            case 1:
                return ArmElevatorConstants.ELEVATOR_LEVEL1_SCORE_INCHES;
            case 2:
                return ArmElevatorConstants.ELEVATOR_LEVEL2_SCORE_INCHES;
            case 3:
                return ArmElevatorConstants.ELEVATOR_LEVEL3_SCORE_INCHES;
            case 4:
                return ArmElevatorConstants.ELEVATOR_LEVEL4_SCORE_INCHES;
            default:
                return ArmElevatorConstants.ELEVATOR_LEVEL1_SCORE_INCHES;
        }
    }

    /**
     * Converts a normal level number (1-4) into the corresponding preset.
     */
    private Preset getPresetForLevel(int level) {
        switch (level) {
            case 1:
                return Preset.LEVEL1;
            case 2:
                return Preset.LEVEL2;
            case 3:
                return Preset.LEVEL3;
            case 4:
                return Preset.LEVEL4;
            default:
                return Preset.LEVEL1;
        }
    }

    /**
     * Converts a score level number (1-4) into the corresponding scoring preset.
     */
    private Preset getScorePresetForLevel(int level) {
        switch (level) {
            case 1:
                return Preset.LEVEL1SCORE;
            case 2:
                return Preset.LEVEL2SCORE;
            case 3:
                return Preset.LEVEL3SCORE;
            case 4:
                return Preset.LEVEL4SCORE;
            default:
                return Preset.LEVEL1SCORE;
        }
    }

    // -----------------------------------------------------------------------
    // Intake Controls (Manual & Auto)
    // -----------------------------------------------------------------------
    public void startManualIntake() {
        manualIntakeActive = true;
        autoIntakeActive = false;
        outtake = false;
    }

    public void startManualOuttake() {
        manualIntakeActive = true;
        autoIntakeActive = false;
        outtake = true;
    }

    public void slowIntake() {
        manualIntakeActive = false;
        autoIntakeActive = false;
        intakeMotor.set(-0.1);
        outtake = false;
    }

    public void stopIntake() {
        manualIntakeActive = false;
        autoIntakeActive = false;
        intakeMotor.set(0.0);
        outtake = false;
    }

    // ------------------------------------------------------------------------
    // Reef Score Command
    // Slows for last 0.1m and checks elevator up/down states.
    // ------------------------------------------------------------------------
    // public Command createReefScoreCommand(boolean leftBranch, int level) {
    // var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
    // List<Integer> allianceTags =
    // (alliance == DriverStation.Alliance.Red) ? ReefConstants.REEF_RED_IDS
    // : ReefConstants.REEF_BLUE_IDS;

    // int visibleTag = findVisibleReefTag(allianceTags);
    // if (visibleTag < 0) {
    // return Commands
    // .print("[ReefScore] No recognized reef tag is visible for our alliance!");
    // }

    // double yOffset = leftBranch ? -ReefConstants.BRANCH_OFFSET_METERS
    // : ReefConstants.BRANCH_OFFSET_METERS;

    // Transform2d approachOffset =
    // new Transform2d(new Translation2d(-ReefConstants.APPROACH_X_OFFSET_METERS, yOffset),
    // new Rotation2d());
    // Pose2d reefContactPose = Vision.getAprilTagPose(visibleTag, approachOffset);

    // Command elevatorNormalCmd;
    // Command elevatorScoreCmd;
    // switch (level) {
    // case 4:
    // elevatorNormalCmd = Commands.runOnce(this::goToLevel4Position, this);
    // elevatorScoreCmd = Commands.runOnce(this::goToLevel4ScorePosition, this);
    // break;
    // case 3:
    // elevatorNormalCmd = Commands.runOnce(this::goToLevel3Position, this);
    // elevatorScoreCmd = Commands.runOnce(this::goToLevel3ScorePosition, this);
    // break;
    // case 2:
    // elevatorNormalCmd = Commands.runOnce(this::goToLevel2Position, this);
    // elevatorScoreCmd = Commands.runOnce(this::goToLevel2ScorePosition, this);
    // break;
    // default: // covers level 1
    // elevatorNormalCmd = Commands.runOnce(this::goToLevel1Position, this);
    // elevatorScoreCmd = Commands.runOnce(this::goToLevel1ScorePosition, this);
    // break;
    // }

    // Command waitForNormalPos = Commands.waitUntil(() -> isElevatorInTolerance(desiredElevInches,
    // ArmElevatorConstants.ELEVATOR_TOLERANCE_INCH));
    // Command waitForScorePos = Commands.waitUntil(() -> isElevatorInTolerance(desiredElevInches,
    // ArmElevatorConstants.ELEVATOR_TOLERANCE_INCH));

    // Command elevatorStow = Commands.runOnce(this::stowElevator, this);
    // Command waitForStowPos = Commands
    // .waitUntil(() -> isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_STOW_INCHES,
    // ArmElevatorConstants.ELEVATOR_TOLERANCE_INCH));

    // Command driveUp = drivebase.twoStepApproach(reefContactPose, 0.1);

    // Pose2d retreatPose = reefContactPose.transformBy(new Transform2d(
    // new Translation2d(-ReefConstants.RETREAT_DISTANCE_METERS, 0.0), new Rotation2d()));
    // Command backAway = drivebase.driveToPose(retreatPose);

    // return Commands.sequence(elevatorNormalCmd, waitForNormalPos, driveUp, elevatorScoreCmd,
    // waitForScorePos, elevatorStow, waitForStowPos, backAway);
    // }

    // private int findVisibleReefTag(List<Integer> validTagIds) {
    // for (Vision.Cameras cam : Vision.Cameras.values()) {
    // var bestResultOpt = cam.getBestResult();
    // if (bestResultOpt.isEmpty()) {
    // continue;
    // }
    // var bestTarget = bestResultOpt.get().getBestTarget();
    // if (bestTarget != null) {
    // int fid = bestTarget.getFiducialId();
    // if (validTagIds.contains(fid)) {
    // return fid;
    // }
    // }
    // }
    // return -1;
    // }

    // -------------------------------
    // Manual Elevator Control
    // -------------------------------
    public void setManualElevatorSpeed(double leftTrigger, double rightTrigger) {
        manualElevator = true;
        double speed = rightTrigger - leftTrigger;
        // Clamp to [-1, 1]
        speed = Math.max(-1.0, Math.min(1.0, speed));
        double volts = 6.0 * speed;
        // Safety check to avoid overextending
        if (getElevatorHeightInches() > 75) {
            elevatorMotorA.setVoltage(0);
            elevatorMotorB.setVoltage(0);
        } else {
            elevatorMotorA.setVoltage(-volts);
            elevatorMotorB.setVoltage(volts);
        }
    }

    public void stopManualElevator() {
        // Latch the current elevator position as our new target
        setManualElevatorSpeed(0.0, 0.0);
        desiredElevInches = getElevatorHeightInches();
        manualElevator = false;
    }

    public void setManualArm(double speed) {
        manualArm = true;
        speed = Math.max(-1.0, Math.min(1.0, speed));
        double volts = 0.7 * speed;
        armMotor.set(-volts);
    }

    public void stopManualArm() {
        setManualArm(0);
        desiredArmAngleDeg = getArmAngleDegrees();
        manualArm = false;
    }

    // -------------------------------
    // Sensor Readouts
    // -------------------------------
    public double getArmAngleDegrees() {
        double sensorDeg = armAbsEnc.getPosition();
        // Apply any offset needed for the absolute encoder
        return (sensorDeg * ArmElevatorConstants.ARM_ABS_ENC_RATIO) - 64.4;
    }

    public double getElevatorHeightInches() {
        double elevTicks = -elevatorMotorB.getPosition().getValueAsDouble();
        return elevTicks / ArmElevatorConstants.ELEV_TICKS_PER_INCH;
    }

    private double getIntakeRPM() {
        return -intakeEncoder.getVelocity();
    }

    // -------------------------------
    // Main Periodic Update
    // -------------------------------
    @Override
    public void periodic() {
        // 1) Pull any updated values from SmartDashboard
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

        // Update the feedforward object
        elevatorFF =
                new ElevatorFeedforward(ArmElevatorConstants.ELEV_kS, ArmElevatorConstants.ELEV_kG,
                        ArmElevatorConstants.ELEV_kV, ArmElevatorConstants.ELEV_kA);

        // Extra constants
        ArmElevatorConstants.ARM_ABS_ENC_RATIO = SmartDashboard.getNumber("Arm Abs Encoder Ratio",
                ArmElevatorConstants.ARM_ABS_ENC_RATIO);
        ArmElevatorConstants.ELEV_TICKS_PER_INCH = SmartDashboard.getNumber("Elev Ticks per Inch",
                ArmElevatorConstants.ELEV_TICKS_PER_INCH);

        // 2) Apply the new constants to the PID controllers
        elevatorController.setP(ArmElevatorConstants.ELEVATOR_kP);
        elevatorController.setI(ArmElevatorConstants.ELEVATOR_kI);
        elevatorController.setD(ArmElevatorConstants.ELEVATOR_kD);
        elevatorController.setConstraints(new TrapezoidProfile.Constraints(
                ArmElevatorConstants.ELEVATOR_MAX_VEL, ArmElevatorConstants.ELEVATOR_MAX_ACC));

        armPID.setP(ArmElevatorConstants.ARM_kP);
        armPID.setI(ArmElevatorConstants.ARM_kI);
        armPID.setD(ArmElevatorConstants.ARM_kD);

        // 3) Send current state data to the dashboard
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

        SmartDashboard.putNumber("Arm Angle (Deg)", getArmAngleDegrees());
        SmartDashboard.putNumber("Elevator Height (In)", getElevatorHeightInches());
        SmartDashboard.putNumber("Arm Desired Position", desiredArmAngleDeg);
        SmartDashboard.putNumber("Elevator Desired Position", desiredElevInches);

        SmartDashboard.putNumber("Intake RPM", getIntakeRPM());

        // 4) Calculate outputs for arm and elevator
        double currentArmDeg = getArmAngleDegrees();
        double currentElevInch = getElevatorHeightInches();

        // Elevator feedforward + PID
        elevatorController.setGoal(Units.inchesToMeters(desiredElevInches));
        double elevOutput = elevatorController.calculate(Units.inchesToMeters(currentElevInch));
        double elevFFVal = elevatorFF.calculate(elevatorController.getSetpoint().velocity);
        double totalElevVolts = Math.max(-4.0, Math.min(4.0, elevOutput + elevFFVal));

        SmartDashboard.putNumber("Elevator Ouput", elevOutput);
        SmartDashboard.putNumber("Elevator FF", elevFFVal);

        // Arm PID
        double armOutput = armPID.calculate(currentArmDeg, desiredArmAngleDeg) / 4;
        double clampedArmVolts = Math.max(-4.0, Math.min(4.0, armOutput));

        // If we're not in manual elevator mode, use the calculated voltage
        if (!manualElevator) {
            elevatorMotorA.setVoltage(totalElevVolts);
            elevatorMotorB.setVoltage(-totalElevVolts);
        }

        // The arm motor is always under PID unless manually overridden
        if (!manualArm) {
            armMotor.set(clampedArmVolts);
        }

        // Intake logic
        if (manualIntakeActive) {
            intakeMotor.set(outtake ? ArmElevatorConstants.INTAKE_SPEED // Outtaking
                    : -ArmElevatorConstants.INTAKE_SPEED); // Intaking
        }
    }

    // -------------------------------
    // Utility Helpers
    // -------------------------------
    private boolean isArmInTolerance(double targetDeg, double toleranceDeg) {
        return Math.abs(getArmAngleDegrees() - targetDeg) <= toleranceDeg;
    }

    private boolean isElevatorInTolerance(double targetIn, double toleranceIn) {
        return Math.abs(getElevatorHeightInches() - targetIn) <= toleranceIn;
    }

    /** Checks if we're in a LEVEL or LEVEL-SCORE preset. */
    private boolean isLevelOrScorePreset(Preset p) {
        return (p == Preset.LEVEL1 || p == Preset.LEVEL2 || p == Preset.LEVEL3 || p == Preset.LEVEL4
                || p == Preset.LEVEL1SCORE || p == Preset.LEVEL2SCORE || p == Preset.LEVEL3SCORE
                || p == Preset.LEVEL4SCORE);
    }

    /**
     * Stall detector that returns true when the intake RPM remains below a threshold for longer
     * than a brief period.
     */
    private BooleanSupplier intakeStallDetector(double stallRpmThreshold) {
        return () -> (Math.abs(getIntakeRPM()) < Math.abs(stallRpmThreshold));
    }
}
