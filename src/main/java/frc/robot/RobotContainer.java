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
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller; // <-- PS4
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ArmElevatorEndEffectorSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {

  // Replace the old driverXbox with a PS4 controller
  private final CommandPS4Controller driverPS4 = new CommandPS4Controller(0);

  // final CommandXboxController auxXbox = new CommandXboxController(1);

  // Subsystems
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
  private final AlgaeIntakeSubsystem algaeIntake = new AlgaeIntakeSubsystem();
  private final ArmElevatorEndEffectorSubsystem armElevator = new ArmElevatorEndEffectorSubsystem(drivebase);
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  // Example SwerveInputStreams for the drivebase
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
    // Example default command for drivebase
    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveDirectAngle));
    } else {
      drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));
    }

    // L2: Move algae pivot to intake angle and start intake while held.
    // When released, stop pivot and intake.
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

    // L1: Outtake algae intake while held, stop when released
    driverPS4.L1()
        .whileTrue(
            Commands.run(
                () -> algaeIntake.intakeReverse(),
                algaeIntake))
        .onFalse(
            Commands.runOnce(
                algaeIntake::intakeStop,
                algaeIntake));

    // R1: Funnel position (once on press)
    driverPS4.R1()
        .onTrue(
            Commands.runOnce(
                armElevator::funnelPosition,
                armElevator));

    // R2: Start manual intake while held, stop on release
    driverPS4.R2()
        .whileTrue(
            Commands.run(
                armElevator::startManualIntake,
                armElevator))
        .onFalse(
            Commands.runOnce(
                armElevator::stopIntake,
                armElevator));

    // D-Pad Up: Elevator to level 4
    driverPS4.povUp()
        .onTrue(
            Commands.runOnce(
                armElevator::goToLevel4Position,
                armElevator));

    // D-Pad Right: Elevator to level 3
    driverPS4.povRight()
        .onTrue(
            Commands.runOnce(
                armElevator::goToLevel3Position,
                armElevator));

    // D-Pad Down: Elevator to level 2
    driverPS4.povDown()
        .onTrue(
            Commands.runOnce(
                armElevator::goToLevel2Position,
                armElevator));

    // D-Pad Left: Elevator to level 1
    driverPS4.povLeft()
        .onTrue(
            Commands.runOnce(
                armElevator::goToLevel1Position,
                armElevator));

    // Cross (X button): reset gyro
    driverPS4.cross()
        .onTrue(
            Commands.runOnce(
                drivebase::zeroGyro,
                drivebase));

    // Square: set climb to fixed speed while held, stop on release
    driverPS4.square()
        .whileTrue(
            Commands.run(
                climbSubsystem::climb,
                climbSubsystem))
        .onFalse(
            Commands.runOnce(
                climbSubsystem::stopClimbMotor,
                climbSubsystem));

    // Circle: stow elevator
    driverPS4.circle()
        .onTrue(
            Commands.runOnce(
                armElevator::stowElevator,
                armElevator));

    // Triangle: set climb angle
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
