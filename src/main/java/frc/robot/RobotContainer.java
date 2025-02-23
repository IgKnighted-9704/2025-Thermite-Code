// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

import java.io.File;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.miscellaneous.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final    CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/maxSwerve"));
  private final ArmSubsystem armBase = new ArmSubsystem();
  private final ClimbSubsystem climbBase = new ClimbSubsystem();
  private final ElevatorSubsystem elevatorbase = new ElevatorSubsystem();
  private final MainSubsystem mainBase = new MainSubsystem();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX(    ) * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
      //subsystems

        //drivetrain
        driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro))); //zero the gyro

        //arm subsystem
          driverXbox.povUp().onTrue(new RunCommand(() -> armBase.raiseArm2pos(Constants.ArmPositions.coralOuttakePos))); //change arm position to coral scoring position
          driverXbox.povDown().onTrue(new RunCommand(() -> armBase.raiseArm2pos(Constants.ArmPositions.reefIntakePos))); // change arm position to reef station intake position

        //climb subsystem
          driverXbox.rightTrigger().whileTrue(new RunCommand(() -> climbBase.changeClimbAngle(0.5))).onFalse(new InstantCommand(() -> climbBase.changeClimbAngle(0))); //forward climb
          driverXbox.leftTrigger().whileTrue(new RunCommand(() -> climbBase.changeClimbAngle(-0.5))).onFalse(new InstantCommand(() -> climbBase.changeClimbAngle(0))); // reverse climb
          driverXbox.povRight().onTrue(new InstantCommand(() -> climbBase.raiseClimbAngle(Constants.ClimbPositions.climbEstimatePos))); //preset climb angle/position

        //elevator subsystem
          driverXbox.x().onTrue(new InstantCommand(() -> elevatorbase.raiseElevatorPos(Constants.ElevatorPositions.level2))); //elevator height to level 2
          driverXbox.y().onTrue(new InstantCommand(() -> elevatorbase.raiseElevatorPos(Constants.ElevatorPositions.level3))); //elevator height to level 3
          driverXbox.b().onTrue(new InstantCommand(() -> elevatorbase.raiseElevatorPos(Constants.ElevatorPositions.level4))); //elevator height to level 4

        //main subsystem
          driverXbox.rightBumper().whileTrue(new RunCommand(() -> mainBase.endEffector(0.5))).onFalse(new InstantCommand(() -> mainBase.endEffector(0))); //intake for end effector
          driverXbox.leftBumper().whileTrue(new RunCommand(() -> mainBase.endEffector(-0.5))).onFalse(new InstantCommand(() -> mainBase.endEffector(0))); //outtake for end effector
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous    
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
