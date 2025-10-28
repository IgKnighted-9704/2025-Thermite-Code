// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.SwerveJoystickCommand;
import frc.subsystems.swervesubsystem.SwerveSubsystem;

public class RobotContainer {

    //Subsystem Initialization
      SwerveSubsystem drivebase = new SwerveSubsystem();

    //Joystick Initial
      CommandPS4Controller main = new CommandPS4Controller(0);
        SwerveJoystickCommand driveCommand = new SwerveJoystickCommand(
          drivebase,
          () -> -main.getLeftY(),
          () -> -main.getLeftX(),
          () -> -main.getRightX(),
          () -> true
        );

  public RobotContainer() {

    drivebase.setDefaultCommand(driveCommand);
    configureBindings();
  }

  private void configureBindings() {

    //Reset Gyro
    main.touchpad().onTrue(Commands.runOnce(() -> drivebase.zeroGyroscope()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
