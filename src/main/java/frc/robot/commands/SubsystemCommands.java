package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants.ModuleConstants;
import frc.subsystems.swervesubsystem.SwerveSubsystem;

public class SubsystemCommands {

    private SlewRateLimiter xLimiter, yLimiter;
    
    //Swerve Subsystem
    public Command DriveToDistance(double xMeters, double yMeters, double velocity, SwerveSubsystem swerveSubsystem){

            double distance = Math.hypot(xMeters, yMeters);
            double angle = Math.atan2(yMeters, xMeters);

            double time = Math.abs(distance/velocity);
                double xSpeed = xLimiter.calculate(velocity * Math.cos(angle) * ModuleConstants.kPhysicalMaxSpeedMetersPerSecond);
                double ySpeed = yLimiter.calculate(velocity * Math.sin(angle) * ModuleConstants.kPhysicalMaxSpeedMetersPerSecond);
            
            ChassisSpeeds setPointSpeeds = SwerveSubsystem.toChassisSpeeds(xSpeed, ySpeed, 0.0, false, swerveSubsystem);
            SwerveModuleState[] setPointStates = Constants.SwerveConstants.DriveConstants.kDriveKinematics.toSwerveModuleStates(setPointSpeeds);
            
            return Commands.sequence(
                Commands.runOnce(() -> swerveSubsystem.setModuleStates(setPointStates)),
                Commands.waitSeconds(time),
                Commands.runOnce(() -> swerveSubsystem.stopModules())
            );
    }

}
