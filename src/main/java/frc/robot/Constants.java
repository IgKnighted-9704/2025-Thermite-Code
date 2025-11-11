package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    // Swervedrive Constants
    public static final class SwerveConstants {
        public static final class ModuleConstants {
            // Gear Ratios & Physical Constants
            public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
            public static final double kPhysicalMaxSpeedMetersPerSecond = 3;
            public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = Math.PI;
            public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 10;
            public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = Math.PI;
            public static final double kDriveMotorGearRatio = 4.71;
            public static final double kAngleMotorGearRatio = 46.42;

            // Conversion Factors
            public static final double kDriveEncoderRot2Meters = (Math.PI * kWheelDiameterMeters)
                    / kDriveMotorGearRatio;
            public static final double kDriveEncoderRot2MetersPerSec = kDriveEncoderRot2Meters / 60;
            public static final double kTurningEncoderRot2Rad = (Math.PI * kWheelDiameterMeters) / kAngleMotorGearRatio;
            public static final double kTurningEncoderRot2RadPerSec = kTurningEncoderRot2Rad / 60;
            // PID Values-Angle Motor
            public static double kPTurning = 0.0;
            public static double kITurning = 0.0;
            public static double kDTurning = 0.0;
            // PID Values-Drive Motor
            public static double kPDriving = 0.2;
            public static double kIDriving = 0.0;
            public static double kDDriving = 0.0;
            // Feedforward Values-Drive Motor
            public static double kSDriving = 0.0;
            public static double kVDriving = 0.0;
            public static double kADriving = 0.0;
        }

        public static final class DriveConstants {
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

            // Gyroscope
            public static final int kGyroPort = 0;

            // Distance Between Wheels Horizontal
            public static final double kTrackWidth = Units.inchesToMeters(24.849);
            // Distance Between Wheels Vertical
            public static final double kWheelBase = Units.inchesToMeters(24.849);

            public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                    new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front Left
                    new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front Right
                    new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Back Left
                    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2) // Back Right
            );

            // Auto Constants
                // Translation PID Values
                public static double kPTranslation = 0.0;
                public static double kITranslation = 0.0;
                public static double kDTranslation = 0.0;
                // Rotation PID Values
                public static double kPRotation = 0.0;
                public static double kIRotation = 0.0;
                public static double kDRotation = 0.0;
        }

        public static final class JoyStickConstants {
            public static final double kDeadBand = 0.2;
        }
    }

    // ArmElevator Constants
    public static final class ArmElevatorConstants {
        public static final class StateConstants {
            // Ratios
            public static final double kElevatorMotorRot2Inches = 0.0;
            // End Effector
            public static final int kEndEffectorMotorID = 10;
            // Elevator
            // MOTOR IDs
            public static final int kElevatorMotorAID = 11;
            public static final int kElevatorMotorBID = 12;
            // PID Values
            public static double kPElevator = 0.0;
            public static double kIElevator = 0.0;
            public static  double kDElevator = 0.0;
            public static double kProfileConstantsMaxSpeed = 0.0;
            public static double kProfileConstantsMaxAccel = 0.0;
            // Feedforward Values
            public static double kSElevator = 0.0;
            public static double kVElevator = 0.0;
            public static double kAElevator = 0.0;
            // Arm
            // MOTOR IDs
            public static final int kArmMotorID = 13;
            public static final int kArmAbsoluteEncoderID = 17;
            // Offset
            public static final double kArmAbsoluteEncoderOffset = 0.0;
            // PID Values
            public static double kPArm = 0.0;
            public static double kIArm = 0.0;
            public static double kDArm = 0.0;
            // Feedforward Values
            public static double kSArm = 0.0;
            public static double kVArm = 0.0;
            public static double kAArm = 0.0;
            // Manual Speed Multipliers
            public static final double kManualElevatorSpeedMultiplier = 0.75;
            public static final double kManualArmSpeedMultiplier = 0.75;
            // Sensors
            public static final int kCoralFunnelSensorPort = 1;
            public static final int kCoralInEndEffectorSensorPort = 2;
        }

        public static final class LevelConstants {
            // Max and Min Heights/Angles
            public static final double maxElevatorHeightInches = 0.0;
            public static final double minElevatorHeightInches = 0.0;
            public static final double maxArmAngleDegrees = 0.0;
            public static final double minArmAngleDegrees = 0.0;
            // Setpoints For Levels
            public static final double kStowElevatorSetpoint = 0.0;
            public static final double kStowArmSetpoint = 0.0;

            public static final double kLoadingElevatorSetpoint = 0.0;
            public static final double kLoadingArmSetpoint = 0.0;

            public static final double kL1FunnelSetpoint = 0.0;
            public static final double kFunnelArmSetpoint = 0.0;

            public static final double kL2ElevatorSetpoint = 0.0;
            public static final double kL2ArmSetpoint = 0.0;

            public static final double kL2ScoreElevatorSetpoint = 0.0;
            public static final double kL2ScoreArmSetpoint = 0.0;

            public static final double kL3ElevatorSetpoint = 0.0;
            public static final double kL3ArmSetpoint = 0.0;

            public static final double kL3ScoreElevatorSetpoint = 0.0;
            public static final double kL3ScoreArmSetpoint = 0.0;

            public static final double kL4ElevatorSetpoint = 0.0;
            public static final double kL4ArmSetpoint = 0.0;

            public static final double kL4ScoreElevatorSetpoint = 0.0;
            public static final double kL4ScoreArmSetpoint = 0.0;

            public static final double kDropCoralArmSetpoint = 0.0;

            //STALL RPM
            public static final double kEndEffectorStallRPM = 0.0;
        }

    }

}
