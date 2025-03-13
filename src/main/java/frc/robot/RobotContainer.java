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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
                        .withControllerRotationAxis(driverPS4::getRightX)
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
                // Keep drive as is
                if (RobotBase.isSimulation()) {
                        drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveDirectAngle));
                } else {
                        drivebase.setDefaultCommand(
                                        drivebase.driveFieldOriented(driveAngularVelocity));
                }

                // Example: Press Options => run reef score command
                // (leftBranch/selectedLevel set by L1/R1 and face buttons)
                // This line is kept exactly to preserve the original comment; we replace the
                // actual binding.

                driverPS4.options().onTrue(Commands.runOnce(() -> {
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
                                        break;
                        }
                }, armElevator));

                // L1 => hold to select left branch
                driverPS4.L1().onTrue(Commands.runOnce(() -> {
                        armElevator.funnelPosition();
                }, armElevator));

                // R1 => hold to select right branch
                driverPS4.R1().onTrue(Commands.runOnce(() -> {
                        armElevator.loadingPosition();
                }, armElevator));

                // X => choose level 1
                driverPS4.cross().onTrue(Commands.runOnce(() -> {
                        selectedLevel = 1;
                        armElevator.goToLevel1Position();
                }, armElevator));

                // Square => choose level 2
                driverPS4.square().onTrue(Commands.runOnce(() -> {
                        selectedLevel = 2;
                        armElevator.goToLevel2Position();
                }, armElevator));

                // Circle => choose level 3
                driverPS4.circle().onTrue(Commands.runOnce(() -> {
                        selectedLevel = 3;
                        armElevator.goToLevel3Position();
                }, armElevator));

                // Triangle => choose level 4
                driverPS4.triangle().onTrue(Commands.runOnce(() -> {
                        selectedLevel = 4;
                        armElevator.goToLevel4Position();
                }, armElevator));

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

                // driverPS4.L2().or(driverPS4.R2()).whileTrue(Commands.run(() -> {
                // double leftVal = driverPS4.getL2Axis();
                // double rightVal = driverPS4.getR2Axis();
                // armElevator.setManualElevatorSpeed(leftVal, rightVal);
                // }, armElevator)).onFalse(Commands.runOnce(
                // () -> armElevator.setManualElevatorSpeed(0.0, 0.0), armElevator));



                // Share button => stow elevator and arm
                driverPS4.share().onTrue(Commands.runOnce(() -> {
                        armElevator.stowElevator();
                }, armElevator));

                // POV up => manual elevator up at quarter speed
                driverPS4.povUp()
                                .whileTrue(Commands.run(
                                                () -> armElevator.setManualElevatorSpeed(0.0, 0.25),
                                                armElevator))
                                .onFalse(Commands.runOnce(
                                                () -> armElevator.setManualElevatorSpeed(0.0, 0.0),
                                                armElevator));

                // POV down => manual elevator down at quarter speed
                driverPS4.povDown()
                                .whileTrue(Commands.run(
                                                () -> armElevator.setManualElevatorSpeed(0.25, 0.0),
                                                armElevator))
                                .onFalse(Commands.runOnce(
                                                () -> armElevator.setManualElevatorSpeed(0.0, 0.0),
                                                armElevator));

                // Cross => zero gyro
                driverPS4.povLeft().onTrue(Commands.runOnce(drivebase::zeroGyro, drivebase));

                // aux
                auxXbox.start().onTrue(Commands.runOnce(() -> {
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
                                        break;
                        }
                }, armElevator));

                // L1 => hold to select left branch
                auxXbox.leftBumper().onTrue(Commands.runOnce(() -> {
                        armElevator.funnelPosition();
                }, armElevator));

                // R1 => hold to select right branch
                auxXbox.rightBumper().onTrue(Commands.runOnce(() -> {
                        armElevator.loadingPosition();
                }, armElevator));

                // X => choose level 1
                auxXbox.a().onTrue(Commands.runOnce(() -> {
                        selectedLevel = 1;
                        armElevator.goToLevel1Position();
                }, armElevator));

                // Square => choose level 2
                auxXbox.x().onTrue(Commands.runOnce(() -> {
                        selectedLevel = 2;
                        armElevator.goToLevel2Position();
                }, armElevator));

                // Circle => choose level 3
                auxXbox.b().onTrue(Commands.runOnce(() -> {
                        selectedLevel = 3;
                        armElevator.goToLevel3Position();
                }, armElevator));

                // Triangle => choose level 4
                auxXbox.y().onTrue(Commands.runOnce(() -> {
                        selectedLevel = 4;
                        armElevator.goToLevel4Position();
                }, armElevator));

                // L2 => pivot intake forward
                auxXbox.leftTrigger().whileTrue(Commands.run(() -> {
                        algaeIntake.setPivotToIntake();
                        algaeIntake.intakeForward();
                }, algaeIntake)).onFalse(Commands.runOnce(algaeIntake::intakeStop, algaeIntake));

                // R2 => pivot intake reverse
                auxXbox.rightTrigger().whileTrue(Commands.run(() -> {
                        algaeIntake.intakeReverse();
                }, algaeIntake)).onFalse(Commands.runOnce(() -> {
                        algaeIntake.stopPivot();
                        algaeIntake.intakeStop();
                }, algaeIntake));


                // Share button => stow elevator and arm
                auxXbox.back().onTrue(Commands.runOnce(() -> {
                        armElevator.stowElevator();
                }, armElevator));

                // POV up => manual elevator up at quarter speed
                auxXbox.povUp().whileTrue(Commands.run(
                                () -> armElevator.setManualElevatorSpeed(0.0, 0.25), armElevator))
                                .onFalse(Commands.runOnce(
                                                () -> armElevator.setManualElevatorSpeed(0.0, 0.0),
                                                armElevator));

                // POV down => manual elevator down at quarter speed
                auxXbox.povDown()
                                .whileTrue(Commands.run(
                                                () -> armElevator.setManualElevatorSpeed(0.25, 0.0),
                                                armElevator))
                                .onFalse(Commands.runOnce(
                                                () -> armElevator.setManualElevatorSpeed(0.0, 0.0),
                                                armElevator));

                // Cross => zero gyro
                auxXbox.povLeft().onTrue(Commands.runOnce(drivebase::zeroGyro, drivebase));
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
