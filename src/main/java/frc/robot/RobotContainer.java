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

/**
 * RobotContainer sets up the operator interface, button bindings, and default commands. It also
 * provides the selected autonomous command for the main robot.
 */
public class RobotContainer {

        // Primary driver on a PS4 controller and auxiliary operator on an Xbox controller.
        private final CommandPS4Controller driverPS4 = new CommandPS4Controller(0);
        private final CommandXboxController auxXbox = new CommandXboxController(1);

        // Core subsystems: swerve drive, intake, arm/elevator, and climb (commented out).
        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
        private final AlgaeIntakeSubsystem algaeIntake = new AlgaeIntakeSubsystem();
        private final ArmElevatorEndEffectorSubsystem armElevator =
                        new ArmElevatorEndEffectorSubsystem(drivebase);
        // private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

        // Swerve input streams for controlling the drivebase.
        // The first one uses angular velocity from a rotation axis, alliance-relative.
        SwerveInputStream driveAngularVelocity = SwerveInputStream
                        .of(drivebase.getSwerveDrive(), () -> -driverPS4.getLeftY(),
                                        () -> -driverPS4.getLeftX())
                        .withControllerRotationAxis(() -> -driverPS4.getRightX())
                        .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        // This second stream uses direct angle control for heading, also alliance-relative.
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(driverPS4::getRightX, driverPS4::getRightY)
                        .headingWhile(true);

        // Track which level (1-4) is selected for elevator/arm presets.
        private int selectedLevel = 1;

        public RobotContainer() {
                configureBindings();
                // Hide joystick connection warnings (optional).
                DriverStation.silenceJoystickConnectionWarning(true);

                // Named command registration for path planner usage.
                NamedCommands.registerCommand("test", Commands.print("I EXIST"));
        }

        /**
         * Defines and configures all button bindings and default commands.
         */
        private void configureBindings() {
                // -------------------------------------------
                // Driver PS4 triggers for intake pivot forward/reverse
                // -------------------------------------------
                driverPS4.L2().whileTrue(Commands.run(() -> {
                        algaeIntake.setPivotToIntake();
                        algaeIntake.intakeForward();
                }, algaeIntake)).onFalse(Commands.runOnce(() -> {
                        algaeIntake.disablePID();
                        algaeIntake.intakeSlow();
                }, algaeIntake));

                driverPS4.R2().whileTrue(Commands.run(() -> {
                        algaeIntake.intakeReverse();
                }, algaeIntake)).onFalse(Commands.runOnce(() -> {
                        algaeIntake.stopPivot();
                        algaeIntake.intakeStop();
                }, algaeIntake));

                // -------------------------------------------
                // Aux triggers for manual intake and outtake
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
                // Aux face buttons for selecting presets
                // Y => go to level 4, A => level 3
                // -------------------------------------------
                auxXbox.y().onTrue(Commands.runOnce(() -> {
                        selectedLevel = 4;
                        armElevator.goToLevelCommand(4).schedule();
                }, armElevator));

                auxXbox.a().onTrue(Commands.runOnce(() -> {
                        selectedLevel = 3;
                        armElevator.goToLevelCommand(3).schedule();
                }, armElevator));

                // -------------------------------------------
                // Aux POV controls for levels 1 and 2
                // -------------------------------------------
                auxXbox.povLeft().onTrue(Commands.runOnce(() -> {
                        selectedLevel = 1;
                        armElevator.goToLevelCommand(1).schedule();
                }, armElevator));

                auxXbox.povRight().onTrue(Commands.runOnce(() -> {
                        selectedLevel = 2;
                        armElevator.goToLevelCommand(2).schedule();
                }, armElevator));

                // -------------------------------------------
                // X => funnel position, B => loading position
                // -------------------------------------------
                auxXbox.x().onTrue(Commands.runOnce(() -> {
                        armElevator.goToFunnelCommand().schedule();
                }, armElevator));

                auxXbox.b().onTrue(Commands.runOnce(() -> {
                        armElevator.goToLoadingCommand().schedule();
                }, armElevator));

                // -------------------------------------------
                // POV Down => stow everything
                // -------------------------------------------
                auxXbox.povDown().onTrue(Commands.runOnce(() -> {
                        armElevator.goToStowCommand().schedule();
                }, armElevator));

                auxXbox.leftBumper().onTrue(Commands.runOnce(() -> {
                        armElevator.goToStowCommand().schedule();
                }, armElevator));

                // -------------------------------------------
                // Right Bumper => move elevator to "score" position
                // for the currently selected level
                // -------------------------------------------
                auxXbox.rightBumper().onTrue(Commands.runOnce(() -> {
                        // Universal method that transitions to the appropriate "score" preset
                        armElevator.goToLevelScoreCommand(selectedLevel).schedule();
                }, armElevator));

                // -------------------------------------------
                // Left Y stick => manual elevator control
                // -------------------------------------------
                new Trigger(() -> Math.abs(auxXbox.getLeftY()) > 0.1).whileTrue(Commands.run(() -> {
                        double raw = -auxXbox.getLeftY();
                        double val = (Math.abs(raw) < 0.05) ? 0.0 : raw;
                        // We pass `val` as "leftTrigger" and 0.0 as "rightTrigger"
                        // so the sign of val raises or lowers the elevator.
                        armElevator.setManualElevatorSpeed(val, 0.0);
                }, armElevator)).onFalse(Commands.runOnce(() -> armElevator.stopManualElevator(),
                                armElevator));

                new Trigger(() -> Math.abs(auxXbox.getRightY()) > 0.1)
                                .whileTrue(Commands.run(() -> {
                                        double raw = auxXbox.getRightY();
                                        double val = (Math.abs(raw) < 0.05) ? 0.0 : raw;
                                        armElevator.setManualArm(val);
                                }, armElevator)).onFalse(Commands.runOnce(
                                                () -> armElevator.stopManualArm(), armElevator));

                // -------------------------------------------
                // Default drive command logic (differing for simulation vs. real)
                // -------------------------------------------
                if (RobotBase.isSimulation()) {
                        drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveDirectAngle));
                } else {
                        drivebase.setDefaultCommand(
                                        drivebase.driveFieldOriented(driveAngularVelocity));
                }

                // Map the PS4 D-Pad left to zero the gyro
                driverPS4.povLeft().onTrue(Commands.runOnce(drivebase::zeroGyro, drivebase));

                driverPS4.cross().onTrue(Commands.runOnce(drivebase::lock, drivebase));
        }

        /**
         * Provides the autonomous command to be scheduled in auto mode.
         */
        public Command getAutonomousCommand() {
                // A simple drive forward for 2 seconds, then stop.
                // Command autonomousCommand = new SequentialCommandGroup(new InstantCommand(() -> {
                // drivebase.driveCommand(() -> -0.5, () -> 0, () -> 0).schedule();
                // }), new WaitCommand(2), new InstantCommand(() -> {
                // drivebase.driveCommand(() -> 0, () -> 0, () -> 0).schedule();
                // }));

                Command autonomousCommand = new SequentialCommandGroup(new InstantCommand(() -> {
                        armElevator.goToFunnelCommand().schedule();
                }), new WaitCommand(2), new InstantCommand(() -> {
                        selectedLevel = 4;
                        armElevator.goToLevelCommand(4).schedule();
                }), new WaitCommand(2), new InstantCommand(() -> {
                        drivebase.driveCommand(() -> -0.25, () -> 0, () -> 0).schedule();
                }), new WaitCommand(2.5), new InstantCommand(() -> {
                        drivebase.driveCommand(() -> 0, () -> 0, () -> 0).schedule();
                }), new WaitCommand(3), new InstantCommand(() -> {
                        armElevator.goToLevelScoreCommand(4).schedule();
                //         armElevator.slowIntake();
                // }), new WaitCommand(1), new InstantCommand(() -> {
                //         selectedLevel = 1;
                //         armElevator.goToLevelCommand(1).schedule();
                }), new WaitCommand(2), new InstantCommand(() -> {
                        armElevator.startManualOuttake();
                }), new WaitCommand(1), new InstantCommand(() -> {
                        armElevator.stopIntake();
                        armElevator.goToStowCommand().schedule();
                }));
                return autonomousCommand;
        }

        /** Sets the drive motors to brake or coast mode. */
        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }

        /** Retrieves the swerve subsystem for external references. */
        public SwerveSubsystem getDrivebase() {
                return drivebase;
        }
}
