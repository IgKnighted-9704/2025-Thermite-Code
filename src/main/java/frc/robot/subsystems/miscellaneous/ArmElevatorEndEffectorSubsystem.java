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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.Constants;
import frc.robot.Constants.ArmElevatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ArmElevatorEndEffectorSubsystem extends SubsystemBase {
    
    //Elevator 
        private final TalonFX elevatorMotorA;
        private final TalonFX elevatorMotorB;
        private final ProfiledPIDController elevatorController;
        private ElevatorFeedforward elevatorFF;
     // Arm 
        private final TalonFX armMotor;
        private final PIDController armPID;
    // End Effector
        private final SparkMax intakeMotor;
        private final RelativeEncoder intakeEncoder;
        private final SparkAbsoluteEncoder armAbsEnc;
    
    // Periodic Tracker
        private double desiredArmAngleDeg;
        private double desiredElevInches;
        private Preset currentPreset;

    //Referencing Our Swerve Drive Train 
        private final SwerveSubsystem drivebase;

    // State Control : Automatic & Manual
    private boolean autoIntakeActive = false;
    private boolean manualIntakeActive = false;
    private boolean manualElevator = false;
    private boolean manualArm = false;
    private boolean outtake = false;


    private enum Preset{
        STOW, FUNNEL, LOADING, LEVEL1, LEVEL2, LEVEL3, LEVEL4, LEVEL1SCORE, LEVEL2SCORE,LEVEL3SCORE,LEVEL4SCORE
    }

    public ArmElevatorEndEffectorSubsystem(SwerveSubsystem driveBase) {

    //Periodic Tracking Initialization
        desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG; //Initial Goal Position
        desiredElevInches = ArmElevatorConstants.ELEVATOR_STOW_INCHES; //Initial Goal Position
        currentPreset = Preset.STOW; //Initial Preset State
        this.drivebase = driveBase;

    // Elevator Components
        elevatorMotorA = new TalonFX(ArmElevatorConstants.ELEVATOR_MOTOR_A_ID);
        elevatorMotorB = new TalonFX(ArmElevatorConstants.ELEVATOR_MOTOR_B_ID);
        elevatorController = new ProfiledPIDController(ArmElevatorConstants.ELEVATOR_kP,
                ArmElevatorConstants.ELEVATOR_kI, ArmElevatorConstants.ELEVATOR_kD,
                new TrapezoidProfile.Constraints(ArmElevatorConstants.ELEVATOR_MAX_VEL,
                        ArmElevatorConstants.ELEVATOR_MAX_ACC));
        elevatorController.setGoal(ArmElevatorConstants.ELEVATOR_STOW_INCHES);  //Initial Goal Position
        
        elevatorFF = new ElevatorFeedforward(ArmElevatorConstants.ELEV_kS, ArmElevatorConstants.ELEV_kG,
                ArmElevatorConstants.ELEV_kV, ArmElevatorConstants.ELEV_kA);

    // Arm Components
        armMotor = new TalonFX(ArmElevatorConstants.ARM_MOTOR_ID);
        armPID = new PIDController(ArmElevatorConstants.ARM_kP, ArmElevatorConstants.ARM_kI, ArmElevatorConstants.ARM_kD);
    
    // EndEffector mechanism
        intakeMotor = new SparkMax(ArmElevatorConstants.ARM_MOTOR_ID, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
        armAbsEnc = intakeMotor.getAbsoluteEncoder();
    
    // Reset Motor Positions
        elevatorMotorA.setPosition(0);
        elevatorMotorB.setPosition(0);
        armMotor.setPosition(armAbsEnc.getPosition());
    }

    //Sensor Readouts
        public double getArmAngleDegrees() {
            double sensorDeg = armAbsEnc.getPosition();
            // Apply any offset needed for the absolute encoder
            return (sensorDeg * ArmElevatorConstants.ARM_ABS_ENC_RATIO) - ArmElevatorConstants.ARM_ABS_ENC_OFFSET;
        }

        public double getElevatorHeightInches() {
            double elevTicks = -elevatorMotorB.getPosition().getValueAsDouble();
            return elevTicks / ArmElevatorConstants.ELEV_TICKS_PER_INCH;
        }

        private double getIntakeRPM() {
            return -intakeEncoder.getVelocity();
        }

    //Utility Helpers
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
    

    //Manual And Automatic End Effector Controls
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

        public Command CoralIntakeCommand(){
            return Commands.sequence(
                    Commands.run(()->{
                        startManualIntake();
                    }), 
                    Commands.waitSeconds(0.5),
                    Commands.run(()->{
                        stopIntake();
                    })
                );
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
    
    //Elevator & Arm Management
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

    // --------------------------------------------------------------------------
    // Commands to move between Presets : Stow, Funnel, Loading, and Levels 1-4
    // --------------------------------------------------------------------------

        // Command -> Move from (STOW OR LEVEL) to FUNNEL
        public Command goToFunnelCommand() {
            return Commands.sequence(
                    goToStowCommand(),
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
                    //WAIT UNTIL THE ELEVATOR OR ARM IS IN TOLERANCE
                    Commands.waitUntil(() -> {
                        if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                            return isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES,
                                    2.0);
                        } else {
                            return isArmInTolerance(ArmElevatorConstants.ARM_STOW_DEG, 2.0);
                        }
                    }),
                    //THEN, THEN MOVE THE SECOND PART (ELEVATOR OR FUNNEL) AND ENABLE AUTOINTAKE
                    Commands.runOnce(() -> {
                        if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                            desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
                        } else {
                            desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES;
                        }
                        autoIntakeActive = true;
                    }), Commands.runOnce(() -> currentPreset = Preset.FUNNEL));
        }

        //Command -> Move from (STOW OR LEVEL) to LOADING
        public Command goToLoadingCommand() {
            return Commands.sequence(Commands.runOnce(() -> {
                // IF WE ARE IN FUNNEL, ADJUST ARM ANGLE FIRST
                if (currentPreset == Preset.FUNNEL) {
                    desiredArmAngleDeg = ArmElevatorConstants.ARM_FUNNEL_DEG;
                }
            }), 
            Commands.waitSeconds(0.5), Commands.runOnce(() -> {
                // IF WE ARE IN A STOW OR LEVEL PRESET, MOVE THE ELEVATOR FIRST
                if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                    desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES;
                } else {
                //OTHERWISE, MOVE THE ARM FIRST
                    desiredArmAngleDeg = ArmElevatorConstants.ARM_LOADING_DEG;
                }
            }), 
            //WAIT UNTIL THE ELEVATOR OR ARM IS IN TOLERANCE
            Commands.waitUntil(() -> {
                if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                    return isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES,
                            2.0);
                } else {
                    return isArmInTolerance(ArmElevatorConstants.ARM_LOADING_DEG, 2.0);
                }
            }), 
            //THEN, THEN MOVE THE SECOND PART (ELEVATOR OR FUNNEL) AND ENABLE AUTOINTAKE
            Commands.runOnce(() -> {
                if (currentPreset == Preset.STOW || isLevelOrScorePreset(currentPreset)) {
                    desiredArmAngleDeg = ArmElevatorConstants.ARM_LOADING_DEG;
                } else {
                    desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES;
                }
                autoIntakeActive = true;
            }), 
            Commands.runOnce(() -> currentPreset = Preset.LOADING));
        }    

        // Command -> Move from (STOW OR LEVEL) to (LOADING or FUNNEL)
        private Command goToLevelFromLoadingCommand(int level){
            return Commands.sequence(
                Commands.runOnce(()-> {
                    //IF WE ARE IN THE LOADING OR FUNNEL...
                    if(currentPreset == Preset.LOADING || currentPreset == Preset.FUNNEL){
                        //Set the desired angle position for the level
                        desiredArmAngleDeg = getArmAngleForLevel(level);
                    //IF WE ARE NOT IN THE LOADING OR FUNNEL...
                    } else {
                        currentPreset = getPresetForLevel(level);
                        //Set the desired elevator position for the level
                        desiredElevInches = getElevatorInchesForLevel(level);
                    }
                }),
                // Wait until the arm or elevator is in tolerance for the desired level
                Commands.race(
                    Commands.waitUntil( () -> {
                        if (currentPreset == Preset.LOADING || currentPreset == Preset.FUNNEL) {
                            return isArmInTolerance(getArmAngleForLevel(level), 2.0);
                        } else {
                            return isElevatorInTolerance(getElevatorInchesForLevel(level), 2.0);
                        }      
                    }),
                    Commands.waitSeconds(1),
                    Commands.runOnce(() -> {
                        currentPreset = getPresetForLevel(level);
                    })
                ), 
                // Set the desired arm angle or elevator position based on the current preset
                Commands.runOnce(() ->{
                    //IF WE ARE IN THE LOADING OR FUNNEL...
                    if (currentPreset == Preset.LOADING || currentPreset == Preset.FUNNEL) {
                        //Set the desired elevator position for the level
                        desiredElevInches = getElevatorInchesForLevel(level);
                    //IF WE ARE NOT IN THE LOADING OR FUNNEL...
                    } else {
                        //Set the desired arm angle for the level
                        desiredArmAngleDeg = getArmAngleForLevel(level);
                    }
                    autoIntakeActive = false;
                }),
                Commands.runOnce(() -> {
                    currentPreset = getPresetForLevel(level);
                })
            );
        } 

        // Command -> Move from (STOW to LOADING to LEVEL) OR (LOADING TO LEVEL)
            public Command goToLevelCommand(int level){
            /*
             *The goToLevelFromLoading command first moves to Level 3 to avoid Collision 2 when raising the arm to Level 2. 
             *By going to Level 3, the arm passes the collision point safely, then transitions to the stow position and finally to Level 2, effectively avoiding the collision.
            */
                if(level == 2){
                    // IF LEVEL2 IS DESIRED STATE...
                    currentPreset = Preset.LEVEL2;
                    return Commands.sequence( 
                        //Go to level 3
                        goToLevelFromLoadingCommand(3),
                        //Wait for arm to be in tolerance
                        Commands.waitUntil(() -> {
                            return isArmInTolerance(ArmElevatorConstants.ARM_LEVEL2_DEG, 2.0);
                        }),
                        //Go to level 2
                        goToLevelFromLoadingCommand(2)
                    );
                }

                if(currentPreset == Preset.FUNNEL){
                //IF WE ARE IN FUNNEL...
                    //Adjust Arm Angle
                    return Commands.sequence(Commands.runOnce(() ->{
                        desiredArmAngleDeg = ArmElevatorConstants.ARM_FUNNEL_DEG;
                    }), 
                    // Wait for Arm to be in Tolerance
                    Commands.race(                          //TODO : Create Utility
                        Commands.waitSeconds(0.5),
                        Commands.waitUntil(() -> {
                            return isArmInTolerance(ArmElevatorConstants.ARM_FUNNEL_DEG, 2.0);
                        })),
                    Commands.runOnce(() -> {
                        manualIntakeActive = true;
                        outtake = false;
                    }),
                    // Move Elevator to Loading Height
                    Commands.runOnce(() -> {
                        desiredElevInches = ArmElevatorConstants.ELEVATOR_FUNNEL_LOADING_INCHES;
                    }),
                    Commands.race(
                        Commands.waitUntil(
                            intakeStallDetector(ArmElevatorConstants.INTAKE_STOPPED_RPM)),
                        Commands.waitSeconds(0.5)
                    ),
                    Commands.runOnce(() -> slowIntake()),
                    // Wait for Elevator to be in Tolerance
                    Commands.race(
                        Commands.waitSeconds(0.5),
                        Commands.waitUntil(() -> {
                            return isElevatorInTolerance(ArmElevatorConstants.ELEVATOR_FUNNEL_INCHES, 2.0);
                        })),
                    // Move Arm to Loading Angle
                    Commands.runOnce(() -> {
                        desiredArmAngleDeg = ArmElevatorConstants.ARM_LEVEL1_DEG; // Default to Level 1
                    }),
                     goToLevelFromLoadingCommand(level)
                    );   
                } else {
                //IF WE ARE NOT IN FUNNEL...
                    currentPreset = getPresetForLevel(level);
                    return goToLevelFromLoadingCommand(level);
                }
            }

        // Command -> Move FROM LEVEL to STOW
            public Command goToStowCommand(){

                /*
                *  Default Logic:
                *   - Moves the arm to the stow angle first.
                *   - Waits until the arm is within tolerance.
                *   - Then moves the elevator to the stow height and disables auto intake.
                */
                Command DefaultStowPos =  Commands.sequence(
                    Commands.runOnce(() -> {
                    desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
                }), 
                    Commands.waitUntil(() -> {
                    return isArmInTolerance(ArmElevatorConstants.ARM_STOW_DEG, 2.0);
                }), 
                    Commands.runOnce(() -> {
                    desiredElevInches = ArmElevatorConstants.ELEVATOR_STOW_INCHES;
                    autoIntakeActive = false;
                }), 
                    Commands.runOnce(() -> currentPreset = Preset.STOW));
                                        
                //Decision Matrix
                    if(currentPreset == Preset.LEVEL2 || 
                    currentPreset == Preset.LEVEL2SCORE || 
                    currentPreset == Preset.LEVEL4 || 
                    currentPreset == Preset.LEVEL4SCORE){
                        return Commands.sequence(
                            goToLevelCommand(3),
                            DefaultStowPos
                        );
                    }
                return DefaultStowPos;
            }

    // --------------------------------------------------------------------------
    // Vision
    // --------------------------------------------------------------------------
            //Find's Visible Reef's April Tag.
            //Returns Tag Id as an int if found, otherwise returns -1.
                private int findVisibleReefTag(List <Integer> visibleTags) {
                    for(Vision.Cameras cam : Vision.Cameras.values()) {
                        var bestResult = cam.getBestResult();
                        if(bestResult.isEmpty()){
                            continue;
                        }
                        var bestTarget = bestResult.get().getBestTarget();
                        if(bestTarget != null){
                            int fid = bestTarget.getFiducialId();
                             if(visibleTags.contains(fid)){
                                return fid; // Return the first visible tag ID found
                             }
                        }
                    }
                    return -1;
                }
            //Determines pose via tag id.
            //Performs a 2dTransform to move to the desired pose
            //Returns a command to execute the movement.
                public Command createReefScoreCommand(boolean leftBranch){
                    
                    //Stores Alliance Color
                    var driverAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
                    List <Integer> TagColors;
                    double yOffSet;

                    // Determine the tag colors based on the alliance
                        if(driverAlliance == DriverStation.Alliance.Red)
                        {
                            TagColors = Constants.ReefConstants.REEF_RED_IDS;
                        } else{
                            TagColors = Constants.ReefConstants.REEF_BLUE_IDS;
                        }
                    // Find the visible reef tag ID
                        // If no tag is found, return a command that prints an error message
                    int targetId = findVisibleReefTag(TagColors);
                        if(targetId == -1){
                            Command errorCommand = Commands.runOnce(() -> {
                                System.out.println("No visible reef tag found.");
                            });
                            return errorCommand;
                        }
                    //If left branch, sets the y offset to a positive value
                    //If right branch, sets the y offset to a negative value
                    //This is used to offset the robot's position when approaching the reef
                    if(leftBranch){
                        yOffSet = Constants.ReefConstants.BRANCH_OFFSET_METERS;

                    } else {
                        yOffSet = -Constants.ReefConstants.BRANCH_OFFSET_METERS;
                    }
                    
                    // Create a Transform2d to offset the robot's position based on the branch and level
                    // APPROACH_X_OFFSET_METERS is a constant that defines the x offset for the approach
                    // The yOffSet is used to adjust the robot's position based on the branch
                    Transform2d offsetTransform2d = new Transform2d(
                        new Translation2d(Constants.ReefConstants.APPROACH_X_OFFSET_METERS, yOffSet), 
                        new Rotation2d());
                    
                    // Get the pose of the April Tag using the Vision subsystem
                    // The pose is adjusted by the offsetTransform2d to account for the approach offset
                    Pose2d reefContactPose = Vision.getAprilTagPose(targetId, offsetTransform2d);
                    
                    //Return a command to drive the robot to the reef contact pose
                    // The drivebase.twoStepApproach method is used to approach the reef contact pose
                    // The second parameter is the tolerance for the approach during which it will drive slower
                    Command reefDriveCommand = drivebase.twoStepApproach(reefContactPose, 0.1);

                    return reefDriveCommand;
                }

    // --------------------------------------------------------------------------
    // Commands To Score Levels 1-4
    // --------------------------------------------------------------------------
    
        // Command -> Move from LOADING to LEVEL-SCORE
        // This command is used to transition from the loading position to a specific level score position.
            public Command goToLevelScoreCommand(int level){
                return goToScoreFromLoadingCommand(level);
            }
        //  Command -> Move from LOADING to LEVEL-SCORE
        //  This command is used to transition from the loading position to a specific level score position.
            private Command goToScoreFromLoadingCommand(int level){
            
            if(level == 2 || currentPreset == Preset.LEVEL2){
                startManualIntake();
            }
                return Commands.sequence(
                    Commands.runOnce(() -> {
                        if(level != 2 || currentPreset != Preset.LEVEL2 || currentPreset != Preset.LEVEL2SCORE){
                            desiredArmAngleDeg = ArmElevatorConstants.ARM_STOW_DEG;
                            currentPreset = getScorePresetForLevel(level);
                        }
                    }),
                    Commands.waitSeconds(1),
                    Commands.runOnce(() -> stopIntake())
                );
            }
    
    // --------------------------------------------------------------------------
    // Manual Elevator Controls
    // --------------------------------------------------------------------------
        // SETS MANUAL ELEVATOR SPEED BASED ON RIGHT TRIGGER INPUT & DEAD BAND
         public void setManualElevatorSpeed(double rightTrigger, double deadBand){
            manualElevator = true;
            double speed = rightTrigger * deadBand;
            speed = Math.max(-1.0, Math.min(1, speed));
            double volts = 6.0*speed;
            if(getElevatorHeightInches() > 75){
                elevatorMotorA.setVoltage(volts);
                elevatorMotorB.setVoltage(volts);
            } else {
                elevatorMotorA.setVoltage(-volts);
                elevatorMotorB.setVoltage(volts);
            }
         }
         
         //STOPS MANUAL ELEVATOR CONTROL AND LATCHES CURRENT POSITION AS DESIRED POSITION
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

            SmartDashboard.putNumber("Intake RPM", getIntakeRPM());
        
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
