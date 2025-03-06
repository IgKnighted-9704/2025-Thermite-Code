package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ArmElevatorEndEffectorSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {

  // Using a PS4 controller for driver input instead of the old Xbox controller.
  private final CommandPS4Controller driverPS4 = new CommandPS4Controller(0);

  // private final CommandXboxController auxXbox = new CommandXboxController(1);

  // Subsystems
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
  private final AlgaeIntakeSubsystem algaeIntake = new AlgaeIntakeSubsystem();
  private final ArmElevatorEndEffectorSubsystem armElevator = new ArmElevatorEndEffectorSubsystem(drivebase);
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  // Set up example input streams for controlling the swerve drive.
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverPS4.getLeftY() * -1,
      () -> driverPS4.getLeftX() * -1)
      .withControllerRotationAxis(driverPS4::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(driverPS4::getRightX, driverPS4::getRightY)
      .headingWhile(true);

  public RobotContainer() {
    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  private void configureBindings() {
    // Set the default command for the drivebase: use direct angle control in
    // simulation,
    // and angular velocity control in real-world operation.
    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveDirectAngle));
    } else {
      drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));
    }

    // Press L2 to pivot the algae intake to the proper position and start intake.
    // Releasing L2 stops both the pivot and intake.
    driverPS4.L2()
        .whileTrue(
            Commands.run(
                () -> {
                  algaeIntake.setPivotToIntake();
                  algaeIntake.intakeForward();
                },
                algaeIntake))
        .onFalse(
            Commands.runOnce(
                () -> {
                  algaeIntake.stopPivot();
                  algaeIntake.intakeStop();
                },
                algaeIntake));

    // Hold L1 to run the algae intake in reverse (outtaking).
    // Releasing L1 stops the reverse action.
    driverPS4.L1()
        .whileTrue(
            Commands.run(
                () -> algaeIntake.intakeReverse(),
                algaeIntake))
        .onFalse(
            Commands.runOnce(
                algaeIntake::intakeStop,
                algaeIntake));

    // Press R1 to move the arm elevator to the funnel position.
    driverPS4.R1()
        .onTrue(
            Commands.runOnce(
                armElevator::funnelPosition,
                armElevator));

    // Hold R2 to activate manual intake; releasing R2 stops the intake.
    driverPS4.R2()
        .whileTrue(
            Commands.run(
                armElevator::startManualIntake,
                armElevator))
        .onFalse(
            Commands.runOnce(
                armElevator::stopIntake,
                armElevator));

    // Press D-Pad Up to move the elevator to level 4.
    driverPS4.povUp()
        .onTrue(
            Commands.runOnce(
                armElevator::goToLevel4Position,
                armElevator));

    // Press D-Pad Right to move the elevator to level 3.
    driverPS4.povRight()
        .onTrue(
            Commands.runOnce(
                armElevator::goToLevel3Position,
                armElevator));

    // Press D-Pad Down to move the elevator to level 2.
    driverPS4.povDown()
        .onTrue(
            Commands.runOnce(
                armElevator::goToLevel2Position,
                armElevator));

    // Press D-Pad Left to move the elevator to level 1.
    driverPS4.povLeft()
        .onTrue(
            Commands.runOnce(
                armElevator::goToLevel1Position,
                armElevator));

    // Press the X button to reset the gyro.
    driverPS4.cross()
        .onTrue(
            Commands.runOnce(
                drivebase::zeroGyro,
                drivebase));

    // Hold Square to engage the climb mechanism at a fixed speed; releasing stops
    // the climb motor.
    driverPS4.square()
        .whileTrue(
            Commands.run(
                climbSubsystem::climb,
                climbSubsystem))
        .onFalse(
            Commands.runOnce(
                climbSubsystem::stopClimbMotor,
                climbSubsystem));

    // Press Circle to stow the elevator.
    driverPS4.circle()
        .onTrue(
            Commands.runOnce(
                armElevator::stowElevator,
                armElevator));

    // Press Triangle to set the climb pivot to the correct angle.
    driverPS4.triangle()
        .onTrue(
            Commands.runOnce(
                climbSubsystem::setPivotToClimbAngle,
                climbSubsystem));
  }

  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public SwerveSubsystem getDrivebase() {
    return drivebase;
  }
}
