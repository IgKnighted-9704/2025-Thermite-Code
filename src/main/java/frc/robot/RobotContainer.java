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

        // These track which branch (left or right) is selected, and what level was
        // chosen.
        private boolean leftBranch = true;
        private int selectedLevel = 1;

        public RobotContainer() {
                configureBindings();
                DriverStation.silenceJoystickConnectionWarning(true);
                NamedCommands.registerCommand("test", Commands.print("I EXIST"));
        }

        private void configureBindings() {
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
                // Only move elevator if RB not pressed;
                // if RB is pressed, move arm instead
                // -------------------------------------------
                auxXbox.y().onTrue(Commands.runOnce(() -> {
                        if (!auxXbox.rightBumper().getAsBoolean()) {
                                // Elevator to level 4
                                selectedLevel = 4;
                                armElevator.goToLevel4ElevatorPosition();
                        } else {
                                // Right bumper => move arm
                                armElevator.goToLevel4ArmPosition();
                        }
                }, armElevator));

                auxXbox.a().onTrue(Commands.runOnce(() -> {
                        if (!auxXbox.rightBumper().getAsBoolean()) {
                                // Elevator to level 3
                                selectedLevel = 3;
                                armElevator.goToLevel3ElevatorPosition();
                        } else {
                                // Right bumper => move arm
                                armElevator.goToLevel3ArmPosition();
                        }
                }, armElevator));

                auxXbox.povLeft().onTrue(Commands.runOnce(() -> {
                        if (!auxXbox.rightBumper().getAsBoolean()) {
                                // Elevator to level 1
                                selectedLevel = 1;
                                armElevator.goToLevel1ElevatorPosition();
                        } else {
                                // Right bumper => move arm
                                armElevator.goToLevel1ArmPosition();
                        }
                }, armElevator));

                auxXbox.povRight().onTrue(Commands.runOnce(() -> {
                        if (!auxXbox.rightBumper().getAsBoolean()) {
                                // Elevator to level 2
                                selectedLevel = 2;
                                armElevator.goToLevel2ElevatorPosition();
                        } else {
                                // Right bumper => move arm
                                armElevator.goToLevel2ArmPosition();
                        }
                }, armElevator));

                // X and A both send funnel commands
                auxXbox.x().onTrue(Commands.runOnce(() -> {
                        if (!auxXbox.rightBumper().getAsBoolean()) {
                                armElevator.funnelElevatorPosition();
                        } else {
                                armElevator.funnelArmPosition();
                        }
                }, armElevator));

                auxXbox.b().onTrue(Commands.runOnce(() -> {
                        if (!auxXbox.rightBumper().getAsBoolean()) {
                                armElevator.loadingElevatorPosition();
                        } else {
                                armElevator.loadingArmPosition();
                        }
                }, armElevator));

                // POV Down => stow
                auxXbox.povDown().onTrue(Commands.runOnce(() -> {
                        if (!auxXbox.rightBumper().getAsBoolean()) {
                                armElevator.stowElevatorPosition();
                        } else {
                                armElevator.stowArmPosition();
                        }
                }, armElevator));

                // -------------------------------------------
                // Left bumper => move elevator to score
                // for the selected level
                // -------------------------------------------
                auxXbox.leftBumper().onTrue(Commands.runOnce(() -> {
                        switch (selectedLevel) {
                                case 1:
                                        armElevator.goToLevel1ScorePosition();
                                        break;
                                case 2:
                                        armElevator.goToLevel2ScorePosition();
                                        break;
                                case 3:
                                        armElevator.goToLevel3ScorePosition();
                                        break;
                                case 4:
                                        armElevator.goToLevel4ScorePosition();
                                        break;
                                default:
                                        // If no valid level selected, do nothing or stow, etc.
                                        break;
                        }
                }, armElevator));

                new Trigger(() -> Math.abs(auxXbox.getLeftY()) > 0.1).whileTrue(Commands.run(() -> {
                        double raw = -auxXbox.getLeftY();
                        double val = (Math.abs(raw) < 0.05) ? 0.0 : raw;
                        armElevator.setManualElevatorSpeed(val, 0.0);
                }, armElevator)).onFalse(Commands.runOnce(
                                () -> armElevator.stopManualElevator(), armElevator));

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
                Command autonomousCommand = new SequentialCommandGroup(

                                new InstantCommand(() -> {
                                        drivebase.driveCommand(() -> -0.5, () -> 0, () -> 0)
                                                        .schedule();
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
