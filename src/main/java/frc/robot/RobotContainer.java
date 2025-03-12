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
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ArmElevatorEndEffectorSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {

        private final CommandPS4Controller driverPS4 = new CommandPS4Controller(0);

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

        public RobotContainer() {
                configureBindings();
                DriverStation.silenceJoystickConnectionWarning(true);
                NamedCommands.registerCommand("test", Commands.print("I EXIST"));
        }

        private void configureBindings() {
                // Default drive command
                if (RobotBase.isSimulation()) {
                        drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveDirectAngle));
                } else {
                        drivebase.setDefaultCommand(
                                        drivebase.driveFieldOriented(driveAngularVelocity));
                }

                // // L2 => pivot algae intake + run intake
                // driverPS4.L2().whileTrue(Commands.run(() -> {
                // algaeIntake.setPivotToIntake();
                // algaeIntake.intakeForward();
                // }, algaeIntake)).onFalse(Commands.runOnce(() -> {
                // algaeIntake.stopPivot();
                // algaeIntake.intakeStop();
                // }, algaeIntake));

                driverPS4.L2().or(driverPS4.R2())
                                .whileTrue(Commands.run(() -> {
                                        double leftVal = driverPS4.getL2Axis();
                                        double rightVal = driverPS4.getR2Axis();
                                        armElevator.setManualElevatorSpeed(leftVal, rightVal);
                                }, armElevator))
                                .onFalse(Commands.runOnce(
                                                () -> armElevator.setManualElevatorSpeed(0.0, 0.0),
                                                armElevator));

                // Example: Press Options => run reef score command
                driverPS4.options().onTrue(armElevator.createReefScoreCommand(true, 4));

                // // L1 => outtake while held
                // driverPS4.L1().whileTrue(
                //                 Commands.run(() -> algaeIntake.intakeReverse(), algaeIntake))
                //                 .onFalse(Commands.runOnce(algaeIntake::intakeStop, algaeIntake));

                // R1 => funnel position
                driverPS4.R1().onTrue(Commands.runOnce(armElevator::funnelPosition, armElevator));

                // R2 => manual intake
                driverPS4.L1().whileTrue(Commands.run(armElevator::startManualIntake,
                armElevator))
                .onFalse(Commands.runOnce(armElevator::stopIntake, armElevator));

                // Elevator level shortcuts
                driverPS4.povUp().onTrue(
                                Commands.runOnce(armElevator::goToLevel4Position, armElevator));
                driverPS4.povRight().onTrue(
                                Commands.runOnce(armElevator::goToLevel3Position, armElevator));
                driverPS4.povDown().onTrue(
                                Commands.runOnce(armElevator::goToLevel2Position, armElevator));
                driverPS4.povLeft().onTrue(
                                Commands.runOnce(armElevator::goToLevel1Position, armElevator));

                // Cross => zero gyro
                driverPS4.cross().onTrue(Commands.runOnce(drivebase::zeroGyro, drivebase));

                // // Square => climb while held
                // driverPS4.square().whileTrue(Commands.run(climbSubsystem::climb, climbSubsystem))
                //                 .onFalse(Commands.runOnce(climbSubsystem::stopClimbMotor,
                //                                 climbSubsystem));

                // Circle => stow elevator
                driverPS4.circle().onTrue(Commands.runOnce(armElevator::stowElevator, armElevator));

                // // Triangle => pivot climb
                // driverPS4.triangle().onTrue(Commands.runOnce(climbSubsystem::setPivotToClimbAngle,
                //                 climbSubsystem));
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
