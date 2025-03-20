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
    private final SparkAbsoluteEncoder armAbsEnc;

    // Target positions (arm angle and elevator height)
    private double desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
    private double desiredElevInches = ArmElevatorConstants.ELEVATOR_STOW_INCHES;

    // Reference to the swerve drive, mainly for pitch measurement
    private final SwerveSubsystem drivebase;

    // Intake control modes
    private boolean autoIntakeActive = false;
    private boolean manualIntakeActive = false;
    private boolean manualElevator = false;
    private boolean outtake = false;

    // Keep track of the current “preset” we are in
    private enum Preset {
        STOW, FUNNEL, LOADING, LEVEL1, LEVEL2, LEVEL3, LEVEL4, LEVEL1SCORE, LEVEL2SCORE, LEVEL3SCORE, LEVEL4SCORE
    }

    private Preset currentPreset = Preset.STOW; // start assumed stowed

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

        // Zero the motor positions (if needed)
        elevatorMotorA.setPosition(0);
        elevatorMotorB.setPosition(0);
        armMotor.setPosition(armAbsEnc.getPosition());
    }

    // --------------------------------------------------------------------------
    // Automatic Movement Commands Based on Current/Target Preset
    // --------------------------------------------------------------------------

    /**
     * Moves from the currentPreset to the “Funnel” preset, properly sequencing arm vs elevator,
     * etc.
     */
    public Command goToFunnelCommand() {
        return Commands.sequence(
                // Decide who moves first based on which preset we’re leaving
                Commands.runOnce(() -> {
                    if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                        // If leaving STOW or Level/Score: elevator first
                        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES;
                    } else {
                        // If leaving FUNNEL or LOADING: arm first
                        desiredArmAngleDeg = ArmElevatorConstants.ARM_FUNNEL_DEG;
                    }
                }),
                // Wait until that first movement is within 2 inches or 2 degrees
                Commands.waitUntil(() -> {
                    if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                        return isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES,
                                2.0);
                    } else {
                        return isArmInTolerance(ArmElevatorConstants.ARM_FUNNEL_DEG, 2.0);
                    }
                }),
                // Now move the second piece
                Commands.runOnce(() -> {
                    if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                        desiredArmAngleDeg = ArmElevatorConstants.ARM_FUNNEL_DEG;
                    } else {
                        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES;
                    }
                    autoIntakeActive = true; // funnel => auto intake
                }),
                // Optionally wait again
                Commands.runOnce(() -> currentPreset = Preset.FUNNEL));
    }

    /** Similar command to go to the “Loading” preset. */
    public Command goToLoadingCommand() {
        return Commands.sequence(Commands.runOnce(() -> {
            if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                // elevator first
                desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES;
            } else {
                // arm first
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
            if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                desiredArmAngleDeg = ArmElevatorConstants.ARM_LOADING_DEG;
            } else {
                desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES;
            }
            autoIntakeActive = true;
        }), Commands.runOnce(() -> currentPreset = Preset.LOADING));
    }

    /**
     * Example: going to STOW from whatever we have. If leaving funnel/loading => arm first, else
     * elevator first.
     */
    public Command goToStowCommand() {
        return Commands.sequence(Commands.runOnce(() -> {
            if (currentPreset == Preset.FUNNEL || currentPreset == Preset.LOADING) {
                desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
            } else {
                desiredElevInches = ArmElevatorConstants.ELEVATOR_STOW_INCHES;
            }
        }), Commands.waitUntil(() -> {
            if (currentPreset == Preset.FUNNEL || currentPreset == Preset.LOADING) {
                return isArmInTolerance(ArmElevatorConstants.ARM_STOW_DEG, 2.0);
            } else {
                return isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_STOW_INCHES, 2.0);
            }
        }), Commands.runOnce(() -> {
            if (currentPreset == Preset.FUNNEL || currentPreset == Preset.LOADING) {
                desiredElevInches = ArmElevatorConstants.ELEVATOR_STOW_INCHES;
            } else {
                desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
            }
            autoIntakeActive = false;
        }), Commands.runOnce(() -> currentPreset = Preset.STOW));
    }

    /**
     * Universal method to go to "level N" from the current preset. If you're in FUNNEL, do the
     * special funnel->level logic first, then transitions to normal
     * "goToLevelFromLoadingCommand(level)".
     */
    public Command goToLevelCommand(int level) {
        if (currentPreset == Preset.FUNNEL) {
            return Commands.sequence(
                    // 1) Start intaking
                    Commands.runOnce(() -> {
                        manualIntakeActive = true;
                        outtake = false;
                    }),
                    // 2) Move elevator to loading
                    Commands.runOnce(() -> {
                        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES;
                    }),
                    // 3) Wait for stall or 3 seconds
                    Commands.race(
                            Commands.waitUntil(
                                    intakeStallDetector(ArmElevatorConstants.INTAKE_STOPPED_RPM)),
                            Commands.waitSeconds(3.0)),
                    // 4) Slow intake
                    Commands.runOnce(() -> slowIntake()),
                    // 5) Move arm first
                    Commands.runOnce(() -> {
                        desiredArmAngleDeg = ArmElevatorConstants.ARM_LOADING_DEG;
                    }),
                    // 6) Wait for arm
                    Commands.waitUntil(
                            () -> isArmInTolerance(ArmElevatorConstants.ARM_LOADING_DEG, 2.0)),
                    // 7) Then elevator to loading again (if needed)
                    Commands.runOnce(() -> {
                        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES;
                    }),
                    Commands.waitUntil(() -> isElevatorInTolerance(
                            ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES, 2.0)),
                    // Now normal “loading -> level N” move
                    goToLevelFromLoadingCommand(level));
        } else {
            return goToLevelFromLoadingCommand(level);
        }
    }

    /**
     * Helper command: assume we are basically in “loading” style and want to go to "level N".
     * Elevator-first or arm-first depends on the currentPreset.
     */
    private Command goToLevelFromLoadingCommand(int level) {
        return Commands.sequence(Commands.runOnce(() -> {
            if (currentPreset == Preset.LOADING || currentPreset == Preset.FUNNEL) {
                desiredArmAngleDeg = getArmAngleForLevel(level);
            } else {
                desiredElevInches = getElevatorInchesForLevel(level);
            }
        }), Commands.waitUntil(() -> {
            if (currentPreset == Preset.LOADING || currentPreset == Preset.FUNNEL) {
                return isArmInTolerance(getArmAngleForLevel(level), 2.0);
            } else {
                return isElevatorInTolerance(getElevatorInchesForLevel(level), 2.0);
            }
        }), Commands.runOnce(() -> {
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
    // NEW: Score Logic
    // --------------------------------------------------------------------------

    /**
     * Universal method to go to "score position" for level N. Follows the same collision-avoidance
     * logic as goToLevelCommand(...).
     */
    public Command goToLevelScoreCommand(int level) {
        // If the currentPreset is FUNNEL, handle funnel->loading first
        if (currentPreset == Preset.FUNNEL) {
            return Commands.sequence(
                    // 1) Start intaking
                    Commands.runOnce(() -> {
                        manualIntakeActive = true;
                        outtake = false;
                    }),
                    // 2) Elevator to loading
                    Commands.runOnce(() -> {
                        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES;
                    }),
                    // 3) Wait for stall or 3s
                    Commands.race(
                            Commands.waitUntil(
                                    intakeStallDetector(ArmElevatorConstants.INTAKE_STOPPED_RPM)),
                            Commands.waitSeconds(3.0)),
                    // 4) Slow intake
                    Commands.runOnce(() -> slowIntake()),
                    // 5) Arm first
                    Commands.runOnce(() -> {
                        desiredArmAngleDeg = ArmElevatorConstants.ARM_LOADING_DEG;
                    }),
                    Commands.waitUntil(
                            () -> isArmInTolerance(ArmElevatorConstants.ARM_LOADING_DEG, 2.0)),
                    // 6) Elevator to loading again if needed
                    Commands.runOnce(() -> {
                        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES;
                    }),
                    Commands.waitUntil(() -> isElevatorInTolerance(
                            ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES, 2.0)),
                    // Now do normal "loading -> score"
                    goToScoreFromLoadingCommand(level));
        } else {
            return goToScoreFromLoadingCommand(level);
        }
    }

    /**
     * Helper command: transitions from loading/funnel to the "score" position for level N with
     * collision avoidance (elevator-first or arm-first).
     */
    private Command goToScoreFromLoadingCommand(int level) {
        return Commands.sequence(Commands.runOnce(() -> {
            // If leaving funnel/loading => arm first
            // Otherwise stow/level => elevator first
            if (currentPreset == Preset.LOADING || currentPreset == Preset.FUNNEL) {
                desiredArmAngleDeg = getArmAngleForLevel(level);
            } else {
                desiredElevInches = getElevatorInchesForScoreLevel(level);
            }
        }), Commands.waitUntil(() -> {
            if (currentPreset == Preset.LOADING || currentPreset == Preset.FUNNEL) {
                return isArmInTolerance(getArmAngleForLevel(level), 2.0);
            } else {
                return isElevatorInTolerance(getElevatorInchesForScoreLevel(level), 2.0);
            }
        }), Commands.runOnce(() -> {
            if (currentPreset == Preset.LOADING || currentPreset == Preset.FUNNEL) {
                desiredElevInches = getElevatorInchesForScoreLevel(level);
            } else {
                desiredArmAngleDeg = getArmAngleForLevel(level);
            }
            // If you want to set autoIntakeActive = false or do outtake, you can
        }), Commands.runOnce(() -> {
            // Mark ourselves as in the appropriate "score" preset
            currentPreset = getScorePresetForLevel(level);
        }));
    }

    // --------------------------------------------------------------------------
    // Arm & Elevator: Normal vs Score Positions
    // --------------------------------------------------------------------------

    /** Normal "level" angles. */
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
     * Map normal levels 1-4 to the normal presets.
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
     * Map "score" levels 1-4 to the appropriate "LEVELnSCORE" presets.
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
    // Intake (Manual & Auto)
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

    // -------------------------------
    // Manual elevator
    // -------------------------------

    // ------------------------------------------------------------------------
    // Reef Score Command (new code)
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

    public void setManualElevatorSpeed(double leftTrigger, double rightTrigger) {
        manualElevator = true;
        double speed = rightTrigger - leftTrigger;
        // Clamp speed to [-1,1]
        speed = Math.max(-1.0, Math.min(1.0, speed));
        double volts = 6.0 * speed;
        if (getElevatorHeightInches() > 75) {
            elevatorMotorA.setVoltage(0);
            elevatorMotorB.setVoltage(0);
        } else {
            elevatorMotorA.setVoltage(-volts);
            elevatorMotorB.setVoltage(volts);
        }
    }

    public void stopManualElevator() {
        setManualElevatorSpeed(0.0, 0.0);
        desiredElevInches = getElevatorHeightInches();
        manualElevator = false;
    }

    public void setManualArm(double speed) {
        speed = Math.max(-1.0, Math.min(1.0, speed));
        double volts = 4.0 * speed;
        armMotor.set(volts);
    }

    // -------------------------------
    // Sensor Readouts
    // -------------------------------
    public double getArmAngleDegrees() {
        double sensorDeg = armAbsEnc.getPosition();
        // Adjust by any offset needed for your absolute encoder
        return (sensorDeg * ArmElevatorConstants.ARM_ABS_ENC_RATIO) - 80.7;
    }

    public double getElevatorHeightInches() {
        double elevTicks = -elevatorMotorB.getPosition().getValueAsDouble();
        return elevTicks / ArmElevatorConstants.ELEV_TICKS_PER_INCH;
    }

    private double getIntakeRPM() {
        return intakeEncoder.getVelocity();
    }

    // -------------------------------
    // Main Periodic Update
    // -------------------------------
    @Override
    public void periodic() {
        // 1) Read updated values from SmartDashboard (PID, feedforward, etc.)
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

        // feedforward
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

        // 2) Apply updated constants
        elevatorController.setP(ArmElevatorConstants.ELEVATOR_kP);
        elevatorController.setI(ArmElevatorConstants.ELEVATOR_kI);
        elevatorController.setD(ArmElevatorConstants.ELEVATOR_kD);
        elevatorController.setConstraints(new TrapezoidProfile.Constraints(
                ArmElevatorConstants.ELEVATOR_MAX_VEL, ArmElevatorConstants.ELEVATOR_MAX_ACC));

        armPID.setP(ArmElevatorConstants.ARM_kP);
        armPID.setI(ArmElevatorConstants.ARM_kI);
        armPID.setD(ArmElevatorConstants.ARM_kD);


        // 3) Display sensor + target info
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
        SmartDashboard.putNumber("Arm Angle (Deg)", getArmAngleDegrees());
        SmartDashboard.putNumber("Elevator Height (In)", getElevatorHeightInches());
        SmartDashboard.putNumber("Arm Desired Position", desiredArmAngleDeg);
        SmartDashboard.putNumber("Elevator Desired Position", desiredElevInches);

        // 4) Run your PID loops and set motor outputs
        double currentArmDeg = getArmAngleDegrees();
        double currentElevInch = getElevatorHeightInches();

        // Elevator feedforward & PID
        elevatorController.setGoal(Units.inchesToMeters(desiredElevInches));
        double elevOutput = elevatorController.calculate(Units.inchesToMeters(currentElevInch));
        double elevFF = elevatorFF.calculate(elevatorController.getSetpoint().velocity);
        double totalElevVolts = Math.max(-4.0, Math.min(4.0, elevOutput + elevFF));

        SmartDashboard.putNumber("Elevator Ouput", elevOutput);
        SmartDashboard.putNumber("Elevator FF", elevFF);

        // Arm PID
        double armOutput = armPID.calculate(currentArmDeg, desiredArmAngleDeg) / 4;
        double clampedArmVolts = Math.max(-4.0, Math.min(4.0, armOutput));

        // Apply elevator power if not in manual override
        if (!manualElevator) {
            elevatorMotorA.setVoltage(totalElevVolts);
            elevatorMotorB.setVoltage(-totalElevVolts);
        }

        // Arm always uses the PID voltage (unless you override with setManualArm)
        armMotor.set(clampedArmVolts);

        // Intake motor logic
        if (manualIntakeActive) {
            intakeMotor.set(outtake ? ArmElevatorConstants.INTAKE_SPEED // positive => out
                    : -ArmElevatorConstants.INTAKE_SPEED // negative => in
            );
        }
    }

    // -------------------------------
    // Helper Functions
    // -------------------------------
    private boolean isArmInTolerance(double targetDeg, double toleranceDeg) {
        return Math.abs(getArmAngleDegrees() - targetDeg) <= toleranceDeg;
    }

    private boolean isElevatorInTolerance(double targetIn, double toleranceIn) {
        return Math.abs(getElevatorHeightInches() - targetIn) <= toleranceIn;
    }

    /** True if the preset is one of the LEVEL or LEVELxSCORE states. */
    private boolean isLevelOrScorePreset(Preset p) {
        return (p == Preset.LEVEL1 || p == Preset.LEVEL2 || p == Preset.LEVEL3 || p == Preset.LEVEL4
                || p == Preset.LEVEL1SCORE || p == Preset.LEVEL2SCORE || p == Preset.LEVEL3SCORE
                || p == Preset.LEVEL4SCORE);
    }

    /**
     * Example “stall” detector: returns a BooleanSupplier that’s true if intake RPM has fallen
     * below some threshold for more than e.g. 0.25s (You can implement a short filter if needed.)
     */
    private BooleanSupplier intakeStallDetector(double stallRpmThreshold) {
        return () -> (Math.abs(getIntakeRPM()) < Math.abs(stallRpmThreshold));
    }
}
