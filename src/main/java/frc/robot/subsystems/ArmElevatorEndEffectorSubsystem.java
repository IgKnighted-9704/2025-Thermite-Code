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
    private boolean manualElevator = false;
    private boolean outtake = false;

    // Track whether we need to outtake at a scoring position
    private boolean scoringActive = false;
    private int scoringLevel = 0;

    // Track if we were at funnel
    private boolean wasAtFunnel = false;

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

        elevatorMotorA.setPosition(0);
        elevatorMotorB.setPosition(0);
        armMotor.setPosition(armAbsEnc.getPosition());
    }

    // ----------------------------------
    // Arm Presets
    // ----------------------------------
    public void funnelArmPosition() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_FUNNEL_DEG;
    }

    public void loadingArmPosition() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_LOADING_DEG;
    }

    public void stowArmPosition() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
    }

    public void goToLevel1ArmPosition() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL1_DEG;
    }

    public void goToLevel2ArmPosition() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL2_DEG;
    }

    public void goToLevel3ArmPosition() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL3_DEG;
    }

    public void goToLevel4ArmPosition() {
        desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL4_DEG;
    }

    // ----------------------------------
    // Elevator Presets
    // ----------------------------------
    public void funnelElevatorPosition() {
        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES;
        autoIntakeActive = true;
    }

    public void loadingElevatorPosition() {
        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES;
        autoIntakeActive = true;
    }

    public void stowElevatorPosition() {
        desiredElevInches = ArmElevatorConstants.ELEVATOR_STOW_INCHES;
        autoIntakeActive = false;
    }

    public void goToLevel1ElevatorPosition() {
        desiredElevInches = ArmElevatorConstants.ELEVATOR_LEVEL1_INCHES;
        autoIntakeActive = false;
    }

    public void goToLevel2ElevatorPosition() {
        desiredElevInches = ArmElevatorConstants.ELEVATOR_LEVEL2_INCHES;
        autoIntakeActive = false;
    }

    public void goToLevel3ElevatorPosition() {
        desiredElevInches = ArmElevatorConstants.ELEVATOR_LEVEL3_INCHES;
        autoIntakeActive = false;
    }

    public void goToLevel4ElevatorPosition() {
        desiredElevInches = ArmElevatorConstants.ELEVATOR_LEVEL4_INCHES;
        autoIntakeActive = false;
    }


    // Score positions (new)
    public void goToLevel1ScorePosition() {
        // desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL1_DEG;
        // desiredElevInches = ArmElevatorConstants.ELEVATOR_LEVEL1_SCORE_INCHES; //
        // removed
        autoIntakeActive = false;
        scoringActive = true;
        scoringLevel = 1;
    }

    public void goToLevel2ScorePosition() {
        // desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL2_DEG;
        desiredElevInches = ArmElevatorConstants.ELEVATOR_LEVEL2_SCORE_INCHES;
        autoIntakeActive = false;
        scoringActive = true;
        scoringLevel = 2;
    }

    public void goToLevel3ScorePosition() {
        // desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL3_DEG;
        desiredElevInches = ArmElevatorConstants.ELEVATOR_LEVEL3_SCORE_INCHES;
        autoIntakeActive = false;
        scoringActive = true;
        scoringLevel = 3;
    }

    public void goToLevel4ScorePosition() {
        // desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL4_DEG;
        desiredElevInches = ArmElevatorConstants.ELEVATOR_LEVEL4_SCORE_INCHES;
        autoIntakeActive = false;
        scoringActive = true;
        scoringLevel = 4;
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
    // Sensor Readouts
    // -------------------------------
    public double getArmAngleDegrees() {
        double sensorDeg = armAbsEnc.getPosition();
        return (sensorDeg * Constants.ArmElevatorConstants.ARM_ABS_ENC_RATIO) - 80.7;
    }

    public double getElevatorHeightInches() {
        double elevTicks = -elevatorMotorB.getPosition().getValueAsDouble();
        return elevTicks / ArmElevatorConstants.ELEV_TICKS_PER_INCH;
    }

    private double getIntakeRPM() {
        return intakeEncoder.getVelocity();
    }

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

        SmartDashboard.putNumber("Arm Angle (Deg)", getArmAngleDegrees());
        SmartDashboard.putNumber("Elevator Height (In)", getElevatorHeightInches());
        SmartDashboard.putNumber("Arm Desired Position", desiredArmAngleDeg);
        SmartDashboard.putNumber("Elevator Desired Position", desiredElevInches);

        double currentArmDeg = getArmAngleDegrees();
        double currentElevInch = getElevatorHeightInches();


        // // Calculate elevator control output (PID + feedforward)
        elevatorController.setGoal(Units.inchesToMeters(desiredElevInches));
        double elevOutput = elevatorController.calculate(Units.inchesToMeters(currentElevInch));
        double elevFeedforward = elevatorFF.calculate(elevatorController.getSetpoint().velocity);
        double totalElevVolts = Math.max(-4.0, Math.min(4.0, elevOutput + elevFeedforward));

        SmartDashboard.putNumber("Elevator Ouput", elevOutput);
        SmartDashboard.putNumber("Elevator FF", elevFeedforward);

        // // Calculate arm control output
        // double armOutput = armPID.calculate(currentArmDeg, finalArmDeg);
        // Clamp speed to [-1,1]
        double armOutput = armPID.calculate(currentArmDeg, desiredArmAngleDeg) / 4;

        // // Send voltages to the elevator motors
        if (!manualElevator) {
            elevatorMotorA.setVoltage(totalElevVolts);
            elevatorMotorB.setVoltage(-totalElevVolts);
        }

        armMotor.set(Math.max(-4.0, Math.min(4.0, armOutput)));

        if (manualIntakeActive) {
            intakeMotor.set(outtake ? ArmElevatorConstants.INTAKE_SPEED
                    : -ArmElevatorConstants.INTAKE_SPEED);
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
