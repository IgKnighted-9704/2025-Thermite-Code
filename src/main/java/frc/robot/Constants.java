package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    
    //Swervedrive Constants
        public static final class SwerveConstants{
            public static final class ModuleConstants{
                // Gear Ratios & Physical Constants
                public static final double kWheelDiameterMeters = Units.inchesToMeters(0.0); 
                public static final double kPhysicalMaxSpeedMetersPerSecond = 0.0;
                public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 0.0; 
                public static final double kDriveMotorGearRatio = 4.71; 
                public static final double kAngleMotorGearRatio = 46.42;  
                public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 0.0; 
                public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 0.0; 
                
                //Conversion Factors
                    public static final double kDriveEncoderRot2Meters = (Math.PI * kWheelDiameterMeters)/kDriveMotorGearRatio; 
                    public static final double kDriveEncoderRot2MetersPerSec = kDriveEncoderRot2Meters/60; 
                    public static final double kTurningEncoderRot2Rad = (Math.PI * kWheelDiameterMeters)/kAngleMotorGearRatio; 
                    public static final double kTurningEncoderRot2RadPerSec = kTurningEncoderRot2Rad/60;
                //PID Values-Angle Motor
                    public static final double kPTurning = 0.0; 
                    public static final double kITurning = 0.0; 
                    public static final double kDTurning = 0.0; 
                //PID Values-Drive Motor
                    public static final double kPDriving = 0.0; 
                    public static final double kIDriving = 0.0; 
                    public static final double kDDriving = 0.0;
                //Feedforward Values-Drive Motor
                    public static final double kSDriving = 0.0; 
                    public static final double kVDriving = 0.0; 
                    public static final double kADriving = 0.0;
            }
             public static final class DriveConstants{
                    // Front Left Module
                    public static final int kFrontLeftDriveMotorPort = 2;
                    public static final int kFrontLeftTurningMotorPort = 3;
                    public static final boolean kFrontLeftDriveEncoderReversed = false;
                    public static final boolean kFrontLeftTurningEncoderReversed = true;
                    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.0;
                    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;

                    // Front Right Module
                    public static final int kFrontRightDriveMotorPort = 4;
                    public static final int kFrontRightTurningMotorPort = 5;
                    public static final boolean kFrontRightDriveEncoderReversed = false;
                    public static final boolean kFrontRightTurningEncoderReversed = true;
                    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.0;
                    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;

                    // Back Left Module
                    public static final int kBackLeftDriveMotorPort = 6;
                    public static final int kBackLeftTurningMotorPort = 7;
                    public static final boolean kBackLeftDriveEncoderReversed = false;
                    public static final boolean kBackLeftTurningEncoderReversed = true;
                    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.0;
                    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;

                    // Back Right Module
                    public static final int kBackRightDriveMotorPort = 8;
                    public static final int kBackRightTurningMotorPort = 9;
                    public static final boolean kBackRightDriveEncoderReversed = false;
                    public static final boolean kBackRightTurningEncoderReversed = true;
                    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.0;
                    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

                    //Gyroscope
                    public static final int kGyroPort = 0;

                    //Distance Between Wheels Horizontal
                    public static final double kTrackWidth = Units.inchesToMeters(24.849);
                    //Distance Between Wheels Vertical
                    public static final double kWheelBase = Units.inchesToMeters(24.849);

                    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics (
                        new Translation2d(kWheelBase / 2, kTrackWidth / 2), //Front Left
                        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //Front Right
                        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //Back Left
                        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2) //Back Right
                    );
             }
             public static final class JoyStickConstants {
                public static final double kDeadBand = 0.2;
             }
        }

}
