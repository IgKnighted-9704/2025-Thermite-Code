package frc.subsystems.swervesubsystem;
import frc.subsystems.miscellaneous.*;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants.DriveConstants;
import com.pathplanner.lib.auto.AutoBuilder;

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
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
        );

        frontRightModule = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
        );

        backLeftModule = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
        );

        backRightModule = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed
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
                new SwerveModulePosition(frontLeftModule.getDrivePosition(), frontLeftModule.getState().angle),
                new SwerveModulePosition(frontRightModule.getDrivePosition(), frontRightModule.getState().angle),
                new SwerveModulePosition(backLeftModule.getDrivePosition(), backLeftModule.getState().angle),
                new SwerveModulePosition(backRightModule.getDrivePosition(), backRightModule.getState().angle)
            };
        poseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.DriveConstants.kDriveKinematics, getRotation2d(), initialModulePositions , new Pose2d(0, 0, new Rotation2d(0)));
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

    public static ChassisSpeeds getChassisSpeeds(double vx, double vy, double omega){
        return new ChassisSpeeds(vx, vy, omega);
    }

    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

}