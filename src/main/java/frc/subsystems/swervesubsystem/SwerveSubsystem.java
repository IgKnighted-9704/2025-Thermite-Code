package frc.subsystems.swervesubsystem;

import java.io.IOException;
import java.util.function.BooleanSupplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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


    private final SwerveDrivePoseEstimator poseEstimator;

    public SwerveSubsystem() {

        //Module Initialization
        frontLeftModule = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed
        );

        frontRightModule = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed
        );

        backLeftModule = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed
        );

        backRightModule = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed
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
        poseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.DriveConstants.kDriveKinematics, getRotation2d(), initialModulePositions , new Pose2d(0, 0, new Rotation2d(0)));

        setupPathPlanner();
    }

    public void zeroGyroscope(){
        gyroscope.reset();
    }

    public double getHeading(){
            return Math.IEEEremainder(gyroscope.getYaw().getValueAsDouble(), 360);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getRobotVelocity(){
        return Constants.SwerveConstants.DriveConstants.kDriveKinematics.toChassisSpeeds(
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState()
        );
    } 

    public void stopModules(){
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading()); 
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.ModuleConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        backLeftModule.setDesiredState(desiredStates[2]);
        backRightModule.setDesiredState(desiredStates[3]);

    }

    public static ChassisSpeeds toChassisSpeeds(double vx, double vy, double omega, boolean fieldRelative, SwerveSubsystem swerveSubsystem){
        return fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega , swerveSubsystem.getRotation2d()) : 
                               new ChassisSpeeds(vx, vy, omega);
    }

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

}