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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ArmElevatorEndEffectorSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {

        private final CommandPS4Controller driverPS4 = new CommandPS4Controller(0);
        private final CommandXboxController auxXbox = new CommandXboxController(1);

        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
        private final AlgaeIntakeSubsystem algaeIntake = new AlgaeIntakeSubsystem();
        private final ArmElevatorEndEffectorSubsystem armElevator =
                        new ArmElevatorEndEffectorSubsystem(drivebase);
        private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

        // Example input streams for controlling the swerve
        SwerveInputStream driveAngularVelocity = SwerveInputStream
                        .of(drivebase.getSwerveDrive(), () -> driverPS4.getLeftY() * -1,
                                        () -> driverPS4.getLeftX() * -1)
                        .withControllerRotationAxis(() -> driverPS4.getRightX() * -1)
                        .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(driverPS4::getRightX, driverPS4::getRightY)
                        .headingWhile(true);

        // Track which level was selected
        private int selectedLevel = 1;

        public RobotContainer() {
                configureBindings();
                DriverStation.silenceJoystickConnectionWarning(true);
                NamedCommands.registerCommand("test", Commands.print("I EXIST"));
        }

        private void configureBindings() {
                // L2 => pivot intake forward
                driverPS4.L2().whileTrue(Commands.run(() -> {
                        algaeIntake.setPivotToIntake();
                        algaeIntake.intakeForward();
                }, algaeIntake)).onFalse(Commands.runOnce(algaeIntake::intakeStop, algaeIntake));

                // R2 => pivot intake reverse
                driverPS4.R2().whileTrue(Commands.run(() -> {
                        algaeIntake.intakeReverse();
                }, algaeIntake)).onFalse(Commands.runOnce(() -> {
                        algaeIntake.stopPivot();
                        algaeIntake.intakeStop();
                }, algaeIntake));
                // -------------------------------------------
                // Keep existing triggers for intake as-is
                // -------------------------------------------
                auxXbox.rightTrigger()
                                .whileTrue(Commands.run(() -> armElevator.startManualIntake(),
                                                armElevator))
                                .onFalse(Commands.runOnce(armElevator::slowIntake, armElevator));

                auxXbox.leftTrigger()
                                .whileTrue(Commands.run(() -> armElevator.startManualOuttake(),
                                                armElevator))
                                .onFalse(Commands.runOnce(armElevator::stopIntake, armElevator));

                // -------------------------------------------
                // Y => go to level 4
                // -------------------------------------------
                auxXbox.y().onTrue(Commands.runOnce(() -> {
                        selectedLevel = 4;
                        armElevator.goToLevelCommand(4).schedule();
                }, armElevator));

                // -------------------------------------------
                // A => go to level 3
                // -------------------------------------------
                auxXbox.a().onTrue(Commands.runOnce(() -> {
                        selectedLevel = 3;
                        armElevator.goToLevelCommand(3).schedule();
                }, armElevator));

                // -------------------------------------------
                // POV Left => level 1
                // -------------------------------------------
                auxXbox.povLeft().onTrue(Commands.runOnce(() -> {
                        selectedLevel = 1;
                        armElevator.goToLevelCommand(1).schedule();
                }, armElevator));

                // -------------------------------------------
                // POV Right => level 2
                // -------------------------------------------
                auxXbox.povRight().onTrue(Commands.runOnce(() -> {
                        selectedLevel = 2;
                        armElevator.goToLevelCommand(2).schedule();
                }, armElevator));

                // -------------------------------------------
                // X => go to funnel
                // -------------------------------------------
                auxXbox.x().onTrue(Commands.runOnce(() -> {
                        armElevator.goToFunnelCommand().schedule();
                }, armElevator));

                // -------------------------------------------
                // B => go to loading
                // -------------------------------------------
                auxXbox.b().onTrue(Commands.runOnce(() -> {
                        armElevator.goToLoadingCommand().schedule();
                }, armElevator));

                // -------------------------------------------
                // POV Down => stow
                // -------------------------------------------
                auxXbox.povDown().onTrue(Commands.runOnce(() -> {
                        armElevator.goToStowCommand().schedule();
                }, armElevator));

                // -------------------------------------------
                // Left bumper => move elevator to score
                // for the selected level
                // -------------------------------------------
                auxXbox.rightBumper().onTrue(Commands.runOnce(() -> {
                        // Just call the universal "goToLevelScoreCommand"
                        // and schedule it with the currently selectedLevel
                        armElevator.goToLevelScoreCommand(selectedLevel).schedule();
                }, armElevator));

                // -------------------------------------------
                // Manual elevator using left Y
                // -------------------------------------------
                new Trigger(() -> Math.abs(auxXbox.getLeftY()) > 0.1).whileTrue(Commands.run(() -> {
                        double raw = -auxXbox.getLeftY();
                        double val = (Math.abs(raw) < 0.05) ? 0.0 : raw;
                        // We pass `val` as "leftTrigger" and 0.0 as "rightTrigger",
                        // so that positive or negative val raises/lowers the elevator.
                        armElevator.setManualElevatorSpeed(val, 0.0);
                }, armElevator)).onFalse(Commands.runOnce(() -> armElevator.stopManualElevator(),
                                armElevator));

                // -------------------------------------------
                // Keep your drive bindings, etc.
                // -------------------------------------------
                if (RobotBase.isSimulation()) {
                        drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveDirectAngle));
                } else {
                        drivebase.setDefaultCommand(
                                        drivebase.driveFieldOriented(driveAngularVelocity));
                }

                // Example: Cross => zero gyro
                driverPS4.povLeft().onTrue(Commands.runOnce(drivebase::zeroGyro, drivebase));
        }

        public Command getAutonomousCommand() {
                // return drivebase.getAutonomousCommand("Leave Auto");
                Command autonomousCommand = new SequentialCommandGroup(new InstantCommand(() -> {
                        drivebase.driveCommand(() -> -0.5, () -> 0, () -> 0).schedule();
                }), new WaitCommand(2), // Wait for 2 seconds
                                new InstantCommand(() -> {
                                        drivebase.driveCommand(() -> 0, () -> 0, () -> 0)
                                                        .schedule();
                                }));
                return autonomousCommand;
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }

        public SwerveSubsystem getDrivebase() {
                return drivebase;
        }
}
