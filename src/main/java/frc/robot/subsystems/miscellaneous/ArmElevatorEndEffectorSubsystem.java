package frc.robot.subsystems.miscellaneous;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.Constants;
import frc.robot.Constants.ArmElevatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.Optional;
import org.photonvision.targeting.PhotonPipelineResult;


public class ArmElevatorEndEffectorSubsystem extends SubsystemBase {

    // Elevator
    private final TalonFX elevatorMotorA;
    private final TalonFX elevatorMotorB;
    private final ProfiledPIDController elevatorController;
    private ElevatorFeedforward elevatorFF;
    // Arm
    private final TalonFX armMotor;
    private final PIDController armPID;
    private final SparkMax armEncoderSetup;
    private final SparkAbsoluteEncoder armEncoder;
    // End Effector
    private final TalonFX intakeMotor;

    // Periodic Tracker
    private double desiredArmAngleDeg;
    private double desiredElevInches;
    private Preset currentPreset;

    // Referencing Our Swerve Drive Train
    private final SwerveSubsystem drivebase;
    // State Control : Automatic & Manual
    private boolean autoIntakeActive = false;
    private boolean manualIntakeActive = false;
    private boolean manualElevator = false;
    private boolean manualArm = false;
    private boolean outtake = false;
    private boolean dealgaeState = false;

    private enum Preset {
        STOW, FUNNEL, LOADING, LEVEL1, LEVEL2, LEVEL3, LEVEL4, LEVEL1SCORE, LEVEL2SCORE, LEVEL3SCORE, LEVEL4SCORE, LEVEL2DEALGAE, LEVEL3DEALGAE
    }

    public ArmElevatorEndEffectorSubsystem(SwerveSubsystem driveBase) {

        // Periodic Tracking Initialization
        desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG; // Initial Goal Position
        desiredElevInches = ArmElevatorConstants.ELEVATOR_STOW_INCHES; // Initial Goal Position
        currentPreset = Preset.STOW; // Initial Preset State
        this.drivebase = driveBase;

        // Elevator Components
        elevatorMotorA = new TalonFX(ArmElevatorConstants.ELEVATOR_MOTOR_A_ID);
        elevatorMotorB = new TalonFX(ArmElevatorConstants.ELEVATOR_MOTOR_B_ID);
        elevatorController = new ProfiledPIDController(ArmElevatorConstants.ELEVATOR_kP,
                ArmElevatorConstants.ELEVATOR_kI, ArmElevatorConstants.ELEVATOR_kD,
                new TrapezoidProfile.Constraints(ArmElevatorConstants.ELEVATOR_MAX_VEL,
                        ArmElevatorConstants.ELEVATOR_MAX_ACC));
        elevatorController.setGoal(ArmElevatorConstants.ELEVATOR_STOW_INCHES); // Initial Goal Position

        elevatorFF = new ElevatorFeedforward(ArmElevatorConstants.ELEV_kS, ArmElevatorConstants.ELEV_kG,
                ArmElevatorConstants.ELEV_kV, ArmElevatorConstants.ELEV_kA);

        // Arm Components
        armMotor = new TalonFX(ArmElevatorConstants.ARM_MOTOR_ID);
        armPID = new PIDController(ArmElevatorConstants.ARM_kP, ArmElevatorConstants.ARM_kI,
                ArmElevatorConstants.ARM_kD);
        armEncoderSetup = new SparkMax(Constants.ArmElevatorConstants.ARM_ENCODER_ID, MotorType.kBrushless);
        armEncoder = armEncoderSetup.getAbsoluteEncoder();

        // EndEffector mechanism
        intakeMotor = new TalonFX(Constants.ArmElevatorConstants.INTAKE_MOTOR_ID);

        // Reset Motor Positions
        elevatorMotorA.setPosition(0);
        elevatorMotorB.setPosition(0);
        armMotor.setPosition(0);
    }

    // Sensor Readouts
    public double getArmAngleDegrees() {
        return armEncoder.getPosition() - ArmElevatorConstants.ARM_ABS_ENC_OFFSET;
    }

    public double getElevatorHeightInches() {
        double elevTicks = -elevatorMotorB.getPosition().getValueAsDouble();
        return elevTicks / ArmElevatorConstants.ELEV_TICKS_PER_INCH;
    }

    private double getIntakeRPM() {
        return -intakeMotor.getVelocity().getValueAsDouble();
    }

    // Utility Helpers
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
     * Stall detector that returns true when the intake RPM remains below a
     * threshold for longer
     * than a brief period.
     */
    private BooleanSupplier intakeStallDetector(double stallRpmThreshold) {
        return () -> (Math.abs(getIntakeRPM()) < Math.abs(stallRpmThreshold));
    }

    // Manual And Automatic End Effector Controls
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

    public Command CoralIntakeCommand() {
        return Commands.sequence(
                Commands.run(() -> {
                    startManualIntake();
                }));
    }

    // Preset Management
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
            case -2:
                return Preset.LEVEL2DEALGAE;
            case -3 : 
                return Preset.LEVEL3DEALGAE;
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

    // Elevator & Arm Management
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
            case -2:
                return ArmElevatorConstants.ARM_DEALGAELEVEL2_DEG;
            case -3:
                return ArmElevatorConstants.ARM_DEALGAELEVEL3_DEG;
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
            case -2:
                return ArmElevatorConstants.ELEVATOR_DEALGAELEVEL2_INCHES;
            case -3:
                return ArmElevatorConstants.ELEVATOR_DEALGAELEVEL3_INCHES;
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

    // --------------------------------------------------------------------------
    // Commands to move between Presets : Stow, Funnel, Loading, and Levels 1-4
    // --------------------------------------------------------------------------

    // Command -> Move from (STOW OR LEVEL) to FUNNEL
    public Command goToFunnelCommand() {
        return Commands.sequence(
                // DECIDE WHICH PART MOVES FIRST BASED ON THE CURRENT PRESET
                Commands.runOnce(() -> {
                    if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                        // IF LEAVING THE STOW OR A LEVEL, ELEVATOR FIRST
                        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES;
                    } else {
                        // IF LEAVING FUNNEL OR LOADING, ARM FIRST
                        desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
                    }
                }),
                // WAIT UNTIL THE ELEVATOR OR ARM IS IN TOLERANCE
                Commands.waitUntil(() -> {
                    if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                        return isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES,
                                2.0);
                    } else {
                        return isArmInTolerance(ArmElevatorConstants.ARM_STOW_DEG, 2.0);
                    }
                }),
                // THEN, THEN MOVE THE SECOND PART (ELEVATOR OR FUNNEL) AND ENABLE AUTOINTAKE
                Commands.runOnce(() -> {
                    if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                        desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
                    } else {
                        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES;
                    }
                    // AUTOINTAKE SETS UP FOR CORAL INTAKE POSITION
                    autoIntakeActive = true;
                }),
                // SET CURRENT PRESET to Preset.FUNNEL
                Commands.runOnce(() -> currentPreset = Preset.FUNNEL));
    }

    // Command -> Move from (STOW OR LEVEL) to LOADING
    public Command goToLoadingCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    // IF WE ARE IN FUNNEL, ADJUST ARM ANGLE FIRST
                    if (currentPreset == Preset.FUNNEL) {
                        desiredArmAngleDeg = ArmElevatorConstants.ARM_FUNNEL_DEG;
                    }
                }),
                Commands.waitSeconds(0.5),
                // THEN MOVE TO FUNNEL POSITION (Accounting for Elevator Backlash)
                Commands.runOnce(() -> {
                    // IF WE ARE IN A STOW OR LEVEL PRESET, MOVE THE ELEVATOR FIRST
                    if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES;
                    } else {
                        // OTHERWISE, MOVE THE ARM FIRST
                        desiredArmAngleDeg = ArmElevatorConstants.ARM_LOADING_DEG;
                    }
                }),
                // WAIT UNTIL THE ELEVATOR OR ARM IS IN TOLERANCE
                Commands.waitUntil(() -> {
                    if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                        return isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES,
                                2.0);
                    } else {
                        return isArmInTolerance(ArmElevatorConstants.ARM_LOADING_DEG, 2.0);
                    }
                }),
                // THEN, THEN MOVE THE SECOND PART (ELEVATOR OR FUNNEL) AND ENABLE AUTOINTAKE
                Commands.runOnce(() -> {
                    if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                        desiredArmAngleDeg = ArmElevatorConstants.ARM_LOADING_DEG;
                    } else {
                        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES;
                    }
                    // AUTOINTAKE SETS UP FOR CORAL INTAKE POSITION
                    autoIntakeActive = true;
                }),
                // SET CURRENT PRESET to Preset.LOADING
                Commands.runOnce(() -> currentPreset = Preset.LOADING));
    }

    // Command -> Move from (STOW OR LEVEL) to (LOADING or FUNNEL)
    private Command goToLevelFromLoadingCommand(int level) {
        return Commands.sequence(
                // DECIDE WHICH PART MOVES FIRST BASED ON THE CURRENT PRESET
                Commands.runOnce(() -> {
                    // IF WE ARE IN FUNNEL, ADJUST ARM ANGLE FIRST
                    if (currentPreset == Preset.LOADING || currentPreset == Preset.FUNNEL) {
                        desiredArmAngleDeg = getArmAngleForLevel(level);
                    } else {
                        // IF WE ARE IN STOW OR A LEVEL, MOVE THE ELEVATOR FIRST
                        currentPreset = getPresetForLevel(level);
                        desiredElevInches = getElevatorInchesForLevel(level);
                    }
                }),
                // WAIT UNTIL THE ELEVATOR OR ARM IS IN TOLERANCE
                Commands.race(Commands.waitUntil(() -> {
                    if (currentPreset == Preset.LOADING || currentPreset == Preset.FUNNEL) {
                        return isArmInTolerance(getArmAngleForLevel(level), 2.0);
                    } else {
                        return isElevatorInTolerance(getElevatorInchesForLevel(level), 2.0);
                    }
                }),
                        // WAIT FOR A SHORT TIME TO AVOID COLLISION
                        Commands.waitSeconds(1)),
                // THEN, MOVE THE SECOND PART (ELEVATOR OR FUNNEL) AND DISABLE AUTOINTAKE
                Commands.runOnce(() -> {
                    if (currentPreset == Preset.LOADING || currentPreset == Preset.FUNNEL) {
                        desiredElevInches = getElevatorInchesForLevel(level);
                    } else {
                        desiredArmAngleDeg = getArmAngleForLevel(level);
                    }
                    // DISABLE AUTOINTAKE
                    autoIntakeActive = false;
                }),
                // SET CURRENT PRESET TO THE DESIRED LEVEL
                Commands.runOnce(() -> {
                    currentPreset = getPresetForLevel(level);
                }));
    }

    // Command -> Move from (STOW to LOADING to LEVEL) OR (LOADING TO LEVEL)
    public Command goToLevelCommand(int level) {
        if (currentPreset == Preset.FUNNEL && (level != -2 || level !=-3) ) {
            // IF WE ARE IN FUNNEL...
            // ADJUST ARM ANGLE TO FUNNEL ANGLE
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        desiredArmAngleDeg = ArmElevatorConstants.ARM_FUNNEL_DEG;
                    }),
                    // WAIT UNTIL THE ARM IS IN TOLERANCE
                    Commands.waitUntil(() -> isArmInTolerance(ArmElevatorConstants.ARM_FUNNEL_DEG, 2.0)),
                    Commands.runOnce(() -> {
                        manualIntakeActive = true;
                        outtake = false;
                    }),
                    // MOVE ELEVATOR TO FUNNEL LOADING POSITION
                    Commands.runOnce(() -> {
                        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES;
                    }),
                    Commands.waitUntil(
                            () -> isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES, 2)),
                    Commands.runOnce(() -> {
                        desiredArmAngleDeg = ArmElevatorConstants.ARM_FUNNEL_DEG;
                    }),
                    // WAIT UNTIL THE ARM IS IN TOLERANCE
                    Commands.waitUntil(() -> isArmInTolerance(ArmElevatorConstants.ARM_FUNNEL_DEG, 5)),
                    // WAIT UNTIL THE INTAKE STALLS
                    // Commands.waitUntil(()->
                    // intakeStallDetector(ArmElevatorConstants.INTAKE_STOPPED_RPM).getAsBoolean()),
                    Commands.waitSeconds(0.5),
                    // STOP INTAKE
                    Commands.runOnce(() -> {
                        manualIntakeActive = false;
                        outtake = false;
                        intakeMotor.set(0);
                    }),
                    // MOVE ELEVATOR TO FUNNEL POSITION
                    Commands.runOnce(() -> {
                        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES;
                    }),
                    // WAIT UNTIL THE ELEVATOR IS IN TOLERANCE
                    Commands.waitUntil(() -> {
                        return isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES, 2);
                    }),
                    Commands.waitSeconds(0.5),
                    goToLevelFromLoadingCommand(level));
        } else if (currentPreset == Preset.FUNNEL && (level == -2 || level ==-3) ){
            return goToLevelFromLoadingCommand(level);
        } 
        else if (level == 2) {
            return Commands.sequence(
                goToLevelFromLoadingCommand(3),
                goToLevelFromLoadingCommand(2)    
            );
        } else if (level == -2){
            return Commands.sequence(
                goToLevelFromLoadingCommand(3),
                goToLevelFromLoadingCommand(-2)    
            );
        } else {
            // IF WE ARE NOT IN FUNNEL...
            currentPreset = getPresetForLevel(level);
            return goToLevelFromLoadingCommand(level);
        }
    }

    // Command -> Move FROM LEVEL to STOW
    public Command goToStowCommand() {

        /*
         * Default Logic:
         * - Moves the arm to the stow angle first.
         * - Waits until the arm is within tolerance.
         * - Then moves the elevator to the stow height and disables auto intake.
         */
        Command DefaultStowPos = Commands.sequence(
                Commands.runOnce(() -> {
                    desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
                }),
                Commands.race(
                        Commands.waitSeconds(1),
                        Commands.waitUntil(() -> {
                            return isArmInTolerance(ArmElevatorConstants.ARM_STOW_DEG, 2.0);
                        })),
                Commands.runOnce(() -> {
                    desiredElevInches = ArmElevatorConstants.ELEVATOR_STOW_INCHES;
                    autoIntakeActive = false;
                }),
                Commands.runOnce(() -> currentPreset = Preset.STOW));

        // Decision Matrix
        if (currentPreset == Preset.LEVEL2 ||
                currentPreset == Preset.LEVEL2SCORE || currentPreset == Preset.LEVEL2DEALGAE || currentPreset == Preset.LEVEL2DEALGAE) {
            return Commands.sequence(
                    // SET DESIRED ELEVATOR POSITION TO FUNNEL HEIGHT
                    Commands.runOnce(() -> {
                        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES;
                    }),
                    // WAIT UNTIL THE ELEVATOR IS IN TOLERANCE
                    Commands.waitUntil(() -> isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES, 2.0)),
                    // SET DESIRED ARM ANGLE TO STOW POSITION
                    Commands.runOnce(() -> {
                        desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
                    }),
                    // WAIT UNTIL THE ARM IS IN TOLERANCE
                    Commands.waitUntil(() -> isArmInTolerance(ArmElevatorConstants.ARM_STOW_DEG, 10)),
                    // RETURN TO DEFAULT STOW POSITION
                    DefaultStowPos);
        } else if (currentPreset == Preset.LOADING) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
                    }),
                    goToFunnelCommand(),
                    DefaultStowPos);
        }
        return DefaultStowPos;
    }

    // --------------------------------------------------------------------------
    // Vision
    // --------------------------------------------------------------------------
    // Find's Visible Reef's April Tag.
    // Returns Tag Id as an int if found, otherwise returns -1.
    private int findVisibleReefTag(List<Integer> visibleTags) {
        for (Vision.Cameras cam : Vision.Cameras.values()) {
            var bestResult = cam.getBestResult();
            if (bestResult.isEmpty()) {
                continue;
            }
            var bestTarget = bestResult.get().getBestTarget();
            if (bestTarget != null) {
                int fid = bestTarget.getFiducialId();
                if (visibleTags.contains(fid)) {
                    return fid; // Return the first visible tag ID found
                }
            }
        }
        return -1;
    }

    // Determines pose via tag id.
    // Performs a 2dTransform to move to the desired pose
    // Returns a command to execute the movement.
    public Command createReefScoreCommand(boolean leftBranch) {

        // Stores Alliance Color
        var driverAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
        List<Integer> TagColors;
        double yOffSet;

        // Determine the tag colors based on the alliance
        if (driverAlliance == DriverStation.Alliance.Red) {
            TagColors = Constants.ReefConstants.REEF_RED_IDS;
        } else {
            TagColors = Constants.ReefConstants.REEF_BLUE_IDS;
        }
        // Find the visible reef tag ID
        // If no tag is found, return a command that prints an error message
        int targetId = findVisibleReefTag(TagColors);
        if (targetId == -1) {
            Command errorCommand = Commands.runOnce(() -> {
                System.out.println("No Visible Reef Tag");
                }
            );
            return errorCommand;
        }
        // If left branch, sets the y offset to a positive value
        // If right branch, sets the y offset to a negative value
        // This is used to offset the robot's position when approaching the reef
        if (leftBranch) {
            yOffSet = Constants.ReefConstants.BRANCH_OFFSET_METERS;

        } else {
            yOffSet = -Constants.ReefConstants.BRANCH_OFFSET_METERS;
        }

        // Create a Transform2d to offset the robot's position based on the branch and
        // level
        // APPROACH_X_OFFSET_METERS is a constant that defines the x offset for the
        // approach
        // The yOffSet is used to adjust the robot's position based on the branch
        Transform2d offsetTransform2d = new Transform2d(
                new Translation2d(Constants.ReefConstants.APPROACH_X_OFFSET_METERS, yOffSet),
                new Rotation2d());

        // Get the pose of the April Tag using the Vision subsystem
        // The pose is adjusted by the offsetTransform2d to account for the approach
        // offset
        Pose2d reefContactPose = Vision.getAprilTagPose(targetId, offsetTransform2d);

        // Return a command to drive the robot to the reef contact pose
        // The drivebase.twoStepApproach method is used to approach the reef contact
        // pose
        // The second parameter is the tolerance for the approach during which it will
        // drive slower
        Command reefDriveCommand = drivebase.twoStepApproach(reefContactPose, 0.1);

        return reefDriveCommand;
    }

    // --------------------------------------------------------------------------
    // Commands To Score Levels 1-4
    // --------------------------------------------------------------------------

    // Command -> Move from LOADING to LEVEL-SCORE
    // This command is used to transition from the loading position to a specific
    // level score position.
    public Command goToLevelScoreCommand(int level) {
        return goToScoreFromLoadingCommand(level);
    }

    // Command -> Move from LOADING to LEVEL-SCORE
    // This command is used to transition from the loading position to a specific
    // level score position.
    private Command goToScoreFromLoadingCommand(int level) {
        if (level == 2 || currentPreset == Preset.LEVEL2 || level == -2 || currentPreset == Preset.LEVEL2DEALGAE || level ==-3 || currentPreset == Preset.LEVEL3DEALGAE) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        startManualOuttake();
                    }),
                    Commands.waitSeconds(0.5),
                    Commands.runOnce(() -> {
                        stopIntake();
                    }),
                    Commands.runOnce(() -> {
                        currentPreset = getScorePresetForLevel(level);
                    }));
        } else {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        desiredArmAngleDeg = getArmAngleForLevel(level)-Constants.ArmElevatorConstants.ARM_SCORE_DEG_OFFSET;
                        currentPreset = getScorePresetForLevel(level);
                    }));
        }
    }

    // --------------------------------------------------------------------------
    // Manual Elevator Controls
    // --------------------------------------------------------------------------
    // SETS MANUAL ELEVATOR SPEED BASED ON LEFT TRIGGER INPUT & DEAD BAND
    public void setManualElevatorSpeed(double leftTrigger, double deadBand) {
        manualElevator = true;
        double speed = 0 - leftTrigger;
        speed = Math.max(-1.0, Math.min(1, speed));
        double volts = 6.0 * speed * 0.;
        if (getElevatorHeightInches() > 75) {
            elevatorMotorA.setVoltage(0);
            elevatorMotorB.setVoltage(0);
        } else if (getElevatorHeightInches() > 5) {
            elevatorMotorA.setVoltage(-volts);
            elevatorMotorB.setVoltage(volts);
        } else {
            elevatorMotorA.setVoltage(-volts * deadBand);
            elevatorMotorB.setVoltage(-volts * deadBand);
        }
    }

    // STOPS MANUAL ELEVATOR CONTROL AND LATCHES CURRENT POSITION AS DESIRED
    // POSITION
    public void stopManualElevator() {
        // Latch the current elevator position as our new target
        setManualElevatorSpeed(0.0, 0.0);
        desiredElevInches = getElevatorHeightInches();
        manualElevator = false;
    }

    // SETS MANUAL ARM SPEED BASED ON LEFT TRIGGER INPUT & DEAD BAND
    public void setManualArm(double speed) {
        manualArm = true;
        speed = Math.max(-1.0, Math.min(1.0, speed));
        double volts = 0.7 * speed;
        armMotor.set(-volts);
    }

    // STOPS MANUAL ARM CONTROL AND LATCHES CURRENT POSITION AS DESIRED POSITION
    public void stopManualArm() {
        setManualArm(0);
        desiredArmAngleDeg = getArmAngleDegrees();
        manualArm = false;
    }

    // --------------------------------------------------------------------------
    // Command Sequences For Autonomous
    // --------------------------------------------------------------------------
        public Command AutoScoreSetupSequence(int level){
            Command CoralPickUP = Commands.sequence(
                Commands.runOnce(() -> {
                    desiredArmAngleDeg = ArmElevatorConstants.ARM_FUNNEL_DEG;
                }),
                // WAIT UNTIL THE ARM IS IN TOLERANCE
                Commands.waitUntil(() -> isArmInTolerance(ArmElevatorConstants.ARM_FUNNEL_DEG, 2.0)),
                Commands.runOnce(() -> {
                    manualIntakeActive = true;
                    outtake = false;
                }),
                // MOVE ELEVATOR TO FUNNEL LOADING POSITION
                Commands.runOnce(() -> {
                    desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES;
                }),
                Commands.waitUntil(
                        () -> isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES, 2)),
                Commands.runOnce(() -> {
                    desiredArmAngleDeg = ArmElevatorConstants.ARM_FUNNEL_DEG;
                }),
                // WAIT UNTIL THE ARM IS IN TOLERANCE
                Commands.waitUntil(() -> isArmInTolerance(ArmElevatorConstants.ARM_FUNNEL_DEG, 5)),
                // WAIT UNTIL THE INTAKE STALLS
                // Commands.waitUntil(()->
                // intakeStallDetector(ArmElevatorConstants.INTAKE_STOPPED_RPM).getAsBoolean()),
                Commands.waitSeconds(0.5),
                // STOP INTAKE
                Commands.runOnce(() -> {
                    manualIntakeActive = false;
                    outtake = false;
                    intakeMotor.set(0);
                }),
                // MOVE ELEVATOR TO FUNNEL POSITION
                Commands.runOnce(() -> {
                    desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES;
                }),
                // WAIT UNTIL THE ELEVATOR IS IN TOLERANCE
                Commands.waitUntil(() -> {
                    return isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES, 2);
                }),
                Commands.waitSeconds(0.5),
                goToLevelFromLoadingCommand(level));

            return Commands.sequence(
                goToFunnelCommand(),
                CoralPickUP,
                Commands.waitSeconds(0.75)
            );
        }

        public Command AutoScoreSequence(int level){
            Command EndEffectorScore = Commands.sequence(
                Commands.runOnce(()->{
                    startManualOuttake();
                }),
                Commands.waitSeconds(1),
                Commands.runOnce(()->{
                    stopIntake();
                })
            );

            return Commands.sequence(
                goToLevelScoreCommand(level),
                EndEffectorScore,
                goToStowCommand()
            );
        }


    @Override
    public void periodic(){
        //UPDATED VALUES FROM SMARTDASHBOARD
            //ELEVATOR PIDF
                ArmElevatorConstants.ELEVATOR_kP =
                    SmartDashboard.getNumber("Elevator kP", ArmElevatorConstants.ELEVATOR_kP);
                ArmElevatorConstants.ELEVATOR_kI =
                    SmartDashboard.getNumber("Elevator kI", ArmElevatorConstants.ELEVATOR_kI);
                ArmElevatorConstants.ELEVATOR_kD =
                    SmartDashboard.getNumber("Elevator kD", ArmElevatorConstants.ELEVATOR_kD);

                ArmElevatorConstants.ELEVATOR_MAX_VEL = 
                    SmartDashboard.getNumber("Elevator MaxVelocity", ArmElevatorConstants.ELEVATOR_MAX_VEL);
                ArmElevatorConstants.ELEVATOR_MAX_ACC = 
                    SmartDashboard.getNumber("Elevator MaxAccel",ArmElevatorConstants.ELEVATOR_MAX_ACC);

                //APPLY NEW CONSTANTS TO ELEVATOR PIDF
                    elevatorController.setP(ArmElevatorConstants.ELEVATOR_kP);
                    elevatorController.setI(ArmElevatorConstants.ELEVATOR_kI);
                    elevatorController.setD(ArmElevatorConstants.ELEVATOR_kD);
                    elevatorController.setConstraints(new TrapezoidProfile.Constraints(ArmElevatorConstants.ELEVATOR_MAX_VEL, ArmElevatorConstants.ELEVATOR_MAX_ACC));
            //ARM PID
                ArmElevatorConstants.ARM_kP =
                    SmartDashboard.getNumber("Arm kP", ArmElevatorConstants.ARM_kP);
                ArmElevatorConstants.ARM_kI =
                    SmartDashboard.getNumber("Arm kI", ArmElevatorConstants.ARM_kI);
                ArmElevatorConstants.ARM_kD =
                    SmartDashboard.getNumber("Arm kD", ArmElevatorConstants.ARM_kD);
                    
                //APPLY NEW CONSTANTS TO ARM PID
                    armPID.setP(ArmElevatorConstants.ARM_kP);
                    armPID.setI(ArmElevatorConstants.ARM_kI);
                    armPID.setD(ArmElevatorConstants.ARM_kD);

            //ELEVATOR FEEDFORWARD
                ArmElevatorConstants.ELEV_kS =
                    SmartDashboard.getNumber("Elev kS", ArmElevatorConstants.ELEV_kS);
                ArmElevatorConstants.ELEV_kG =
                    SmartDashboard.getNumber("Elev kG", ArmElevatorConstants.ELEV_kG);
                ArmElevatorConstants.ELEV_kV =
                    SmartDashboard.getNumber("Elev kV", ArmElevatorConstants.ELEV_kV);
                ArmElevatorConstants.ELEV_kA =
                    SmartDashboard.getNumber("Elev kA", ArmElevatorConstants.ELEV_kA);
        
                elevatorFF =
                new ElevatorFeedforward(ArmElevatorConstants.ELEV_kS, ArmElevatorConstants.ELEV_kG, ArmElevatorConstants.ELEV_kV, ArmElevatorConstants.ELEV_kA);

            // EXTRA CONSTANTS
                ArmElevatorConstants.ARM_ABS_ENC_RATIO = 
                    SmartDashboard.getNumber("Arm Abs Encoder Ratio", ArmElevatorConstants.ARM_ABS_ENC_RATIO);
                ArmElevatorConstants.ELEV_TICKS_PER_INCH = 
                    SmartDashboard.getNumber("Elev Ticks per Inch", ArmElevatorConstants.ELEV_TICKS_PER_INCH);

        // CURRENT VALUES ON SMARTDASHBOARD
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

            SmartDashboard.putString("Elevator Preset", currentPreset.toString());
        
        //ARM & ELEVATOR LOGIC (PERIODIC UPDATE)

            //CALCULATE ARM OUTPUT & ELEVATOR OUTPUT
                double currentArmDeg = getArmAngleDegrees();
                double currentElevInch = getElevatorHeightInches();
            
            //ELEVATOR FEEDFORWARD + PID MANAGEMENT
            elevatorController.setGoal(Units.inchesToMeters(desiredElevInches)); //SETS DESIRED GOAL STATE
                double elevOutput = elevatorController.calculate(Units.inchesToMeters(currentElevInch)); //CALCULATES VOLTAGE OUTPUT VIA PID
                double elevFFVal = elevatorFF.calculate(elevatorController.getSetpoint().velocity); //CALCULATES VOLTAGE OUTPUT VIA FEEDFORWARD
                double totalElevVolts = Math.max(-4.0, Math.min(4.0, elevOutput + elevFFVal)); // COMBINES OUTPUTS AND LIMITS TO -4.0 TO 4.0 VOLTS

                //TRACKER
                SmartDashboard.putNumber("Elevator Ouput", elevOutput);
                SmartDashboard.putNumber("Elevator FF", elevFFVal);
            //ARM PID MANAGEMENT
                // desiredArmAngleDeg = (desiredArmAngleDeg > 254) ? desiredArmAngleDeg : 200;
                double armOutput = armPID.calculate(currentArmDeg, desiredArmAngleDeg) / 4;
                double clampedArmVolts = Math.max(-4.0, Math.min(4.0, armOutput));

            //DECISION MATRIX

                // IF WE ARE IN NOT MANUAL ELEVATOR MODE...
                    if (!manualElevator) {
                        elevatorMotorA.setVoltage(totalElevVolts);
                        elevatorMotorB.setVoltage(-totalElevVolts);
                    }
                //IF WE ARE NOT IN MANUAL ARM MODE...
                    if (!manualArm) {
                        armMotor.set(clampedArmVolts);
                    }
                //INTAKE LOGIC FOR RESPECTIVE STATES : Manual & Auto
                    if (manualIntakeActive) {
                        intakeMotor.set(outtake ? ArmElevatorConstants.INTAKE_SPEED // Outtaking
                                : -ArmElevatorConstants.INTAKE_SPEED); // Intaking
                    }
    }
}
