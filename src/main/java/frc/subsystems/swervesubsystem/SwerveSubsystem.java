package frc.subsystems.swervesubsystem;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.DriveConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class SwerveSubsystem extends SubsystemBase {
	private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;
    private Pigeon2 gyroscope;

    public final ShuffleboardTab SwerveSubsystemTracker;
        private GenericEntry RobotVelocity;
        private GenericEntry RobotHeading;
        private GenericEntry ModuleDriveVelocity;
        private GenericEntry ModuleAngle;
        
        private GenericEntry PoseEstimatorX;
        private GenericEntry PoseEstimatorY;
        private GenericEntry PoseEstimatorRotation;

        private GenericEntry PathPlannerTranslationkP;
        private GenericEntry PathPlannerTranslationkI;
        private GenericEntry PathPlannerTranslationkD;

        private GenericEntry PathPlannerRotationkP;
        private GenericEntry PathPlannerRotationkI;
        private GenericEntry PathPlannerRotationKd;

        private GenericEntry DrivekP;
        private GenericEntry DrivekI;
        private GenericEntry DrivekD;

        private GenericEntry AnglekP;
        private GenericEntry AnglekI;
        private GenericEntry AnglekD;

        private GenericEntry DriveFFkS;
        private GenericEntry DriveFFkV;
        private GenericEntry DriveFFkA;


    private final SwerveDrivePoseEstimator poseEstimator;

    public SwerveSubsystem() {

        //Module Initialization
        frontLeftModule = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            0.0
        );

        frontRightModule = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            0.0
        );

        backLeftModule = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            0.0
        );

        backRightModule = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            0.0
        );

        //Gyroscope Initializaiton
        gyroscope = new Pigeon2(Constants.SwerveConstants.DriveConstants.kGyroPort);

        //Zero Gyroscope at Startup
        new Thread(()->{
            try {
                Thread.sleep(1000);
                zeroGyroscope();
            } catch (Exception e) {
            }
        }).start();

        // Initial Module Positions
            SwerveModulePosition [] initialModulePositions = {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            };
        
        //Pose Estimator Setup
        poseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.DriveConstants.kDriveKinematics, getRotation2d(), initialModulePositions , new Pose2d(0, 0, new Rotation2d(0)));

        //Shuffleboard Initialization
        SwerveSubsystemTracker = Shuffleboard.getTab("Swerve Subsystem");
            //Robot Information
                RobotVelocity = SwerveSubsystemTracker.add("Robot Velocity", Math.hypot(getRobotVelocity().vxMetersPerSecond, getRobotVelocity().vyMetersPerSecond)).getEntry();
                RobotHeading = SwerveSubsystemTracker.add("Robot Heading", getHeading()).getEntry();
                ModuleDriveVelocity = SwerveSubsystemTracker.add("Module Drive Velocity", frontRightModule.getDriveVelocity()).getEntry();
                ModuleAngle = SwerveSubsystemTracker.add("Module Angle", frontRightModule.getAngularPosition()).getEntry();
            //Pose Estimator
                PoseEstimatorX = SwerveSubsystemTracker.add("Pose Estimator X", getPose().getX()).getEntry();
                PoseEstimatorY = SwerveSubsystemTracker.add("Pose Estimator Y", getPose().getY()).getEntry();
                PoseEstimatorRotation = SwerveSubsystemTracker.add("Pose Estimator Z", getPose().getRotation().getDegrees()).getEntry();
            //Pathplanner
                //Translation
                PathPlannerTranslationkP = SwerveSubsystemTracker
                    .add("Path Planner kP", Constants.SwerveConstants.DriveConstants.kPTranslation)
                    .getEntry();
                PathPlannerTranslationkI = SwerveSubsystemTracker
                    .add("Path Planner kI", Constants.SwerveConstants.DriveConstants.kITranslation)
                    .getEntry();
                PathPlannerTranslationkD = SwerveSubsystemTracker
                    .add("Path Planner kD", Constants.SwerveConstants.DriveConstants.kDTranslation)
                    .getEntry();
                //Rotation
                PathPlannerRotationkP = SwerveSubsystemTracker
                    .add("Rotation kP", Constants.SwerveConstants.DriveConstants.kPRotation)
                    .getEntry();
                PathPlannerRotationkI = SwerveSubsystemTracker
                    .add("Rotation kI", Constants.SwerveConstants.DriveConstants.kIRotation)
                    .getEntry();
                PathPlannerRotationKd = SwerveSubsystemTracker
                    .add("Rotation kD", Constants.SwerveConstants.DriveConstants.kDRotation)
                    .getEntry();
            //Raw Swerve
                // Drive PID
                DrivekP = SwerveSubsystemTracker
                    .add("Drive kP", Constants.SwerveConstants.ModuleConstants.kPDriving)
                    .getEntry();
                DrivekI = SwerveSubsystemTracker
                    .add("Drive kI", Constants.SwerveConstants.ModuleConstants.kIDriving)
                    .getEntry();
                DrivekD = SwerveSubsystemTracker
                    .add("Drive kD", Constants.SwerveConstants.ModuleConstants.kDDriving)
                    .getEntry();

                // Angular PID (Turning Motor)
                AnglekP = SwerveSubsystemTracker
                    .add("Turn kP", Constants.SwerveConstants.ModuleConstants.kPTurning)
                    .getEntry();
                AnglekI = SwerveSubsystemTracker
                    .add("Turn kI", Constants.SwerveConstants.ModuleConstants.kITurning)
                    .getEntry();
                AnglekD = SwerveSubsystemTracker
                    .add("Turn kD", Constants.SwerveConstants.ModuleConstants.kDTurning)
                    .getEntry();

                // Drive Feedforward
                DriveFFkS = SwerveSubsystemTracker
                    .add("Drive kS", Constants.SwerveConstants.ModuleConstants.kSDriving)
                    .getEntry();
                DriveFFkV = SwerveSubsystemTracker
                    .add("Drive kV", Constants.SwerveConstants.ModuleConstants.kVDriving)
                    .getEntry();
                DriveFFkA = SwerveSubsystemTracker
                    .add("Drive kA", Constants.SwerveConstants.ModuleConstants.kADriving)
                    .getEntry();

        setupPathPlanner();
    }

    //Utility Methods
        public static ChassisSpeeds toChassisSpeeds(double vx, double vy, double omega, boolean fieldRelative, SwerveSubsystem swerveSubsystem){
            return fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega , swerveSubsystem.getRotation2d()) : 
                                new ChassisSpeeds(vx, vy, omega);
        }

    //Swerve Data Acess
        //Gyroscope Heading
        public double getHeading(){
                return Math.IEEEremainder(gyroscope.getYaw().getValueAsDouble(), 360);
        }
        //Gyroscope Heading - Rotation2D
        public Rotation2d getRotation2d(){
            return Rotation2d.fromDegrees(getHeading());
        }
        //PoseEstimator - Robot Pose
        public Pose2d getPose(){
            return poseEstimator.getEstimatedPosition();
        }
        //Robot Velocity
        public ChassisSpeeds getRobotVelocity(){
            return Constants.SwerveConstants.DriveConstants.kDriveKinematics.toChassisSpeeds(
                frontLeftModule.getState(),
                frontRightModule.getState(),
                backLeftModule.getState(),
                backRightModule.getState()
            );
        } 
        //Module Positions
        public SwerveModulePosition [] getModulePositions(){
            return new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            };
        }
        //Module States
        public SwerveModuleState [] getModuleStates(){
            return new SwerveModuleState[] {
                frontLeftModule.getState(),
                frontRightModule.getState(),
                backLeftModule.getState(),
                backRightModule.getState()
            };
        }
    //Swerve Methods
        //Set Robot Speed
        public void setModuleStates(SwerveModuleState[] desiredStates){
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.ModuleConstants.kPhysicalMaxSpeedMetersPerSecond);
            frontLeftModule.setDesiredState(desiredStates[0]);
            frontRightModule.setDesiredState(desiredStates[1]);
            backLeftModule.setDesiredState(desiredStates[2]);
            backRightModule.setDesiredState(desiredStates[3]);
        }
        //Stop Robot
        public void stopModules(){
            frontLeftModule.stop();
            frontRightModule.stop();
            backLeftModule.stop();
            backRightModule.stop();
        }
        //Zero Gyro
        public void zeroGyroscope(){
            gyroscope.reset();
        }

        //Pathplanner
            //Setup Pathplanner
            public void setupPathPlanner(){
                RobotConfig config;
                try {
                    config = RobotConfig.fromGUISettings();
                    AutoBuilder.configure(
                        () -> this.getPose(), 
                        pose -> this.poseEstimator.resetPosition(
                            getRotation2d(), 
                            new SwerveModulePosition[] {
                                frontLeftModule.getPosition(),
                                frontRightModule.getPosition(),
                                backLeftModule.getPosition(),
                                backRightModule.getPosition()
                            },
                            pose) , 
                            () -> this.getRobotVelocity(), 
                            (speeds, feedforwards) -> this.setModuleStates(
                                SwerveConstants.DriveConstants.kDriveKinematics.toSwerveModuleStates(
                                    toChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false, this)
                                )
                            ), 
                            new PPHolonomicDriveController( 
                                    new PIDConstants(
                                        SwerveConstants.DriveConstants.kPTranslation, 
                                        SwerveConstants.DriveConstants.kITranslation, 
                                        SwerveConstants.DriveConstants.kDTranslation
                                    ),
                                    new PIDConstants(
                                        SwerveConstants.DriveConstants.kPRotation, 
                                        SwerveConstants.DriveConstants.kIRotation, 
                                        SwerveConstants.DriveConstants.kDRotation
                                    ) 
                                    
                            ),
                            config, 
                            () -> {
                            var alliance = DriverStation.getAlliance();
                            if (alliance.isPresent()) {
                                return alliance.get() == DriverStation.Alliance.Red;
                            }
                            return false;
                            },
                            this
                    );
                } catch (IOException | ParseException e) {
                    e.printStackTrace();
                }

            }

    @Override
    public void periodic() {
        //Update Pose Estimator
        poseEstimator.update(getRotation2d(), 
               this.getModulePositions()
        );
        //Shuffleboard
            //Live Data
                RobotVelocity.setDouble(Math.hypot(getRobotVelocity().vxMetersPerSecond, getRobotVelocity().vyMetersPerSecond));
                RobotHeading.setDouble(getHeading());
                ModuleDriveVelocity.setDouble(frontRightModule.getDriveVelocity());
                ModuleAngle.setDouble(frontRightModule.getAngularPosition());
            //Pose Estimator
                PoseEstimatorX.setDouble(getPose().getX());
                PoseEstimatorY.setDouble(getPose().getY());
                PoseEstimatorRotation.setDouble(getPose().getRotation().getDegrees());
            //Path Planner
                //Translation
                    //GET NEW PID
                    Constants.SwerveConstants.DriveConstants.kPTranslation = PathPlannerTranslationkP.getDouble(Constants.SwerveConstants.DriveConstants.kPTranslation);
                    Constants.SwerveConstants.DriveConstants.kITranslation = PathPlannerTranslationkI.getDouble(Constants.SwerveConstants.DriveConstants.kITranslation);
                    Constants.SwerveConstants.DriveConstants.kDTranslation = PathPlannerTranslationkD.getDouble(Constants.SwerveConstants.DriveConstants.kDTranslation);
                    //SET NEW PID
                    PathPlannerTranslationkP.setDouble(Constants.SwerveConstants.DriveConstants.kPTranslation);
                    PathPlannerTranslationkI.setDouble(Constants.SwerveConstants.DriveConstants.kITranslation);
                    PathPlannerTranslationkD.setDouble(Constants.SwerveConstants.DriveConstants.kDTranslation);
                //Rotation
                    //GET NEW PID
                    Constants.SwerveConstants.DriveConstants.kPRotation = PathPlannerRotationkP.getDouble(Constants.SwerveConstants.DriveConstants.kPRotation);
                    Constants.SwerveConstants.DriveConstants.kIRotation = PathPlannerRotationkI.getDouble(Constants.SwerveConstants.DriveConstants.kIRotation);
                    Constants.SwerveConstants.DriveConstants.kDRotation = PathPlannerRotationKd.getDouble(Constants.SwerveConstants.DriveConstants.kDRotation);
                    //SET NEW PID
                    PathPlannerRotationkP.setDouble(Constants.SwerveConstants.DriveConstants.kPRotation);
                    PathPlannerRotationkI.setDouble(Constants.SwerveConstants.DriveConstants.kIRotation);
                    PathPlannerRotationKd.setDouble(Constants.SwerveConstants.DriveConstants.kDRotation);
            //Raw Drive
                //Drive PID
                    //GET NEW PID
                    Constants.SwerveConstants.ModuleConstants.kPDriving = DrivekP.getDouble(Constants.SwerveConstants.ModuleConstants.kPDriving);
                    Constants.SwerveConstants.ModuleConstants.kIDriving = DrivekI.getDouble(Constants.SwerveConstants.ModuleConstants.kIDriving);
                    Constants.SwerveConstants.ModuleConstants.kDDriving = DrivekD.getDouble(Constants.SwerveConstants.ModuleConstants.kDDriving);
                    //SET NEW PID
                    DrivekP.setDouble(Constants.SwerveConstants.ModuleConstants.kPDriving);
                    DrivekI.setDouble(Constants.SwerveConstants.ModuleConstants.kIDriving);
                    DrivekD.setDouble(Constants.SwerveConstants.ModuleConstants.kDDriving);
                // Angle PID
                    // GET NEW PID
                    Constants.SwerveConstants.ModuleConstants.kPTurning = AnglekP.getDouble(Constants.SwerveConstants.ModuleConstants.kPTurning);
                    Constants.SwerveConstants.ModuleConstants.kITurning = AnglekI.getDouble(Constants.SwerveConstants.ModuleConstants.kITurning);
                    Constants.SwerveConstants.ModuleConstants.kDTurning = AnglekD.getDouble(Constants.SwerveConstants.ModuleConstants.kDTurning);
                    // SET NEW PID
                    AnglekP.setDouble(Constants.SwerveConstants.ModuleConstants.kPTurning);
                    AnglekI.setDouble(Constants.SwerveConstants.ModuleConstants.kITurning);
                    AnglekD.setDouble(Constants.SwerveConstants.ModuleConstants.kDTurning);

                // Drive Feedforward
                    // GET NEW Feedforward
                    Constants.SwerveConstants.ModuleConstants.kSDriving = DriveFFkS.getDouble(Constants.SwerveConstants.ModuleConstants.kSDriving);
                    Constants.SwerveConstants.ModuleConstants.kVDriving = DriveFFkV.getDouble(Constants.SwerveConstants.ModuleConstants.kVDriving);
                    Constants.SwerveConstants.ModuleConstants.kADriving = DriveFFkA.getDouble(Constants.SwerveConstants.ModuleConstants.kADriving);
                    // SET NEW Feedforward
                    DriveFFkS.setDouble(Constants.SwerveConstants.ModuleConstants.kSDriving);
                    DriveFFkV.setDouble(Constants.SwerveConstants.ModuleConstants.kVDriving);
                    DriveFFkA.setDouble(Constants.SwerveConstants.ModuleConstants.kADriving);
    }

}