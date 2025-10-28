package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.swervesubsystem.SwerveSubsystem;
import java.util.function.Supplier;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.JoyStickConstants;
import frc.robot.Constants.SwerveConstants.ModuleConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveJoystickCommand extends Command{

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpeedSupplier, ySpeedSupplier, rotSupplier;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

    public SwerveJoystickCommand(SwerveSubsystem swerveSubsystem,
                                Supplier<Double> xSpeedSupplier,
                                Supplier<Double> ySpeedSupplier,
                                Supplier<Double> rotSupplier,
                                Supplier<Boolean> fieldOrientedFunction){
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.rotSupplier = rotSupplier;
        this.fieldOrientedFunction = fieldOrientedFunction;

        this.xLimiter = new SlewRateLimiter(ModuleConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(ModuleConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.rotLimiter = new SlewRateLimiter(ModuleConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);


        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
                //Get Real Time Joystick Inputs
        double xSpeed = xSpeedSupplier.get();
        double ySpeed = ySpeedSupplier.get();
        double rotSpeed = rotSupplier.get();

        //Apply Deadband
        xSpeed = Math.abs(xSpeed) > JoyStickConstants.kDeadBand ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > JoyStickConstants.kDeadBand ? ySpeed : 0.0;
        rotSpeed = Math.abs(rotSpeed) > JoyStickConstants.kDeadBand ? rotSpeed : 0.0;

        //Use Slew Rate Limiters To Make Drive Smoother
        xSpeed = xLimiter.calculate(xSpeed * ModuleConstants.kPhysicalMaxSpeedMetersPerSecond);
        ySpeed = yLimiter.calculate(ySpeed * ModuleConstants.kPhysicalMaxSpeedMetersPerSecond);
        rotSpeed = rotLimiter.calculate(rotSpeed * ModuleConstants.kPhysicalMaxSpeedMetersPerSecond);

        ChassisSpeeds chassispeeds;
        // Construct Desired Chassis Speeds
        if(fieldOrientedFunction.get()){
            //Relative To Field
            chassispeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                rotSpeed,
                swerveSubsystem.getRotation2d()
            );
        } else {
            //Relative To Robot
            chassispeeds = new ChassisSpeeds(
                xSpeed,
                ySpeed,
                rotSpeed
            );
        }

        //Convert Chassis Speeds To Individual Module States
        SwerveModuleState[] moduleStates = SwerveConstants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassispeeds);

        //Set Module States
        swerveSubsystem.setModuleStates(moduleStates);
    }
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
