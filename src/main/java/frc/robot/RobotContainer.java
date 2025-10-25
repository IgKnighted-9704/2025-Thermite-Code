
package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.miscellaneous.ArmElevatorEndEffectorSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * RobotContainer sets up the operator interface, button bindings, and default
 * commands. It also
 * provides the selected autonomous command for the main robot.
 */
public class RobotContainer {

        // Controller Initialization
        // Primary Driver (Drive Train) -> driverPS4
        private final CommandPS4Controller driverPS4 = new CommandPS4Controller(0);
        // Secondary Driver (Arm/Intake) -> auxPS4
        private final CommandPS4Controller auxPS4 = new CommandPS4Controller(1);

        // Subsystem Initialization
        // Drive Base
        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
        // Arm Elevator End Effector
        private final ArmElevatorEndEffectorSubsystem armElevator = new ArmElevatorEndEffectorSubsystem(drivebase);

        // Default Drive
        // -------------------------------------------
        // Swerve input streams for driving the robot.
        // This first stream uses joystick input for manual control:
        // - Left stick controls translational movement (forward/backward, left/right).
        // - Right stick X-axis controls angular velocity (how fast the robot rotates).
        // - Controls are alliance-relative, meaning they adjust based on which side of
        // the field you're on.
        // -------------------------------------------
        SwerveInputStream driveAngularVelocity = SwerveInputStream
                        .of(drivebase.getSwerveDrive(), () -> -driverPS4.getLeftY(),
                                        () -> -driverPS4.getLeftX())
                        .withControllerRotationAxis(() -> -driverPS4.getRightX())
                        .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8)
                        .allianceRelativeControl(true);
        // Simulation Drive
        // -------------------------------------------
        // This second input stream uses direct angle control for robot heading:
        // - Left stick still controls translational movement.
        // - Right stick (X and Y axes together) sets the direction the robot should
        // face.
        // - The robot automatically rotates to face that direction (field-oriented).
        // - Controls are alliance-relative, so intended heading adjusts based on
        // alliance side.
        // -------------------------------------------
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(driverPS4::getRightX, driverPS4::getRightY)
                        .headingWhile(true);

        // Level Tracker
        // -------------------------------------------
        // Tracks the selected preset level (0–4) for the elevator/arm system:
        // - 0 = Stow (fully retracted/safe position)
        // - 1 = Level 1 scoring position
        // - 2 = Level 2 scoring position
        // - 3 = Level 3 scoring position
        // - 4 = Substation pickup or max height position (if applicable)
        // -------------------------------------------
        private int selectedLevel = 1;

        // Subsystem Disable/Enable
        boolean ENABLE_ARM_ELEVATOR_SUBSYSTEM = true; // Set to false to disable the Arm Elevator subsystem
        boolean ENABLE_DRIVEBASE_SUBSYSTEM = true; // Set to false to disable the Drivebase subsystem
        boolean ENABLE_MANUAL_CONTROL = false; // Set to false to disable manual control of the Arm Elevator subsystem

        // Driver Disable/Enable
        boolean varunMain = false;
        boolean jakeMain = true;

        // Path Chooser
        SendableChooser<Command> AutonChooser;

        public RobotContainer() {

                // Setup Coral
                        NamedCommands.registerCommand("L4 Coral Setup", armElevator.AutoScoreSequence(4, false));
                        NamedCommands.registerCommand("L3 Coral Setup", armElevator.AutoScoreSequence(3, false));
                        NamedCommands.registerCommand("L2 Coral Setup", armElevator.AutoScoreSequence(2, false));
                        NamedCommands.registerCommand("L1 Coral Setup", armElevator.AutoScoreSequence(1, false));
                // Score Coral
                        NamedCommands.registerCommand("L4 Coral Score", armElevator.AutoScoreSequence(4, true));
                        NamedCommands.registerCommand("L3 Coral Score", armElevator.AutoScoreSequence(3, true));
                        NamedCommands.registerCommand("L2 Coral Score", armElevator.AutoScoreSequence(2, true));
                        NamedCommands.registerCommand("L1 Coral Score", armElevator.AutoScoreSequence(1, true));

                configureBindings();

                // Hide joystick connection warnings (optional).
                DriverStation.silenceJoystickConnectionWarning(true);

                // Path Planner Chooser
                AutonChooser = new SendableChooser<>();
                        AutonChooser.addOption("Auton 1 - 12 Coral Auton", drivebase.getAutonomousCommand("ARL Champs Auton #1"));
                        AutonChooser.addOption("Auton 2 - 6 Coral Auton", drivebase.getAutonomousCommand("ARL Champs Auton #2"));
                        AutonChooser.addOption("Auton 3 - 1 Coral Auton", drivebase.getAutonomousCommand("ARL Champs Auton #3"));
                        AutonChooser.addOption("Auton 4 - Clear 3 Algae + 3 Coral Setup", drivebase.getAutonomousCommand("ARL Champs Auton #4") );
                        AutonChooser.addOption("Auton 5 - Leave Path", drivebase.getAutonomousCommand("ARL Champs Auton #5") );
                        AutonChooser.setDefaultOption("Auton 5 - Leave Path", drivebase.getAutonomousCommand("ARL Champs Auton #5") );
        }

        /**
         * Defines and configures all button bindings and default commands.
         */
        private void configureBindings() {

                // End Effector Manual Control (AUX)
                if (ENABLE_ARM_ELEVATOR_SUBSYSTEM) {
                        auxPS4.R2()
                                        .whileTrue(Commands.run(() -> armElevator.startManualIntake(),
                                                        armElevator))
                                        .onFalse(Commands.runOnce(armElevator::slowIntake, armElevator));

                        auxPS4.L2()
                                        .whileTrue(Commands.run(() -> armElevator.startManualOuttake(),
                                                        armElevator))
                                        .onFalse(Commands.runOnce(armElevator::stopIntake, armElevator));
                }

                // Elevator + Arm Control (AUX)
                       /*
                        * -------------------------------------------
                        * Varun - Main Driver
                                * - Triangle: Level 4
                        * - Square: Level 3
                        * - Circle: Level 2
                        * - R2: Funnel
                        * - L1: Stow
                        * - D-Pad Up: Manual Outtake
                        * - D-Pad Down: Manual Intake
                        * - L2: Manual Intake
                        * - D-Pad Right: Manual Arm Out
                        * - D-Pad Left: Manual Arm In
                        * Jake - Main Driver
                        * - Triangle: Level 4
                        * - Square: Level 3
                        * - Circle: Level 2
                        * - Cross: Funnel
                        * - L1: Stow
                        * - R2: Manual Outtake
                        * - L2: Manual Intake
                        * - D-Pad Right: Manual Arm Out
                        * - D-Pad Left: Manual Arm In
                        */
                if (ENABLE_ARM_ELEVATOR_SUBSYSTEM) {
                        // Driver PS4 Controls
                        if (varunMain) {
                                driverPS4.triangle().onTrue(Commands.runOnce(() -> {
                                        selectedLevel = 4;
                                        armElevator.goToLevelCommand(4).schedule();
                                }, armElevator));

                                driverPS4.square().onTrue(Commands.runOnce(() -> {
                                        selectedLevel = 3;
                                        armElevator.goToLevelCommand(3).schedule();
                                }, armElevator));
                                driverPS4.circle().onTrue(Commands.runOnce(() -> {
                                        selectedLevel = 2;
                                        armElevator.goToLevelCommand(2).schedule();
                                }, armElevator));

                                driverPS4.R2().onTrue(Commands.runOnce(() -> {
                                        armElevator.goToFunnelCommand().schedule();
                                }, armElevator));

                                driverPS4.L1().onTrue(Commands.runOnce(() -> {
                                        selectedLevel = 0;
                                        armElevator.goToStowCommand().schedule();
                                }, armElevator));

                                driverPS4.povRight().whileTrue(Commands.runOnce(() -> {
                                        armElevator.setManualArm(0.1);
                                })).onFalse(Commands.runOnce(() -> {
                                        armElevator.stopManualArm();
                                }));

                                driverPS4.povLeft().whileTrue(Commands.runOnce(() -> {
                                        armElevator.setManualArm(-0.1);
                                })).onFalse(Commands.runOnce(() -> {
                                        armElevator.stopManualArm();
                                }));

                                driverPS4.povUp().onTrue(Commands.runOnce(()->{
                                        armElevator.startManualOuttake();
                                })).onFalse(Commands.runOnce(()->{
                                        armElevator.stopIntake();
                                }));
                                
                                driverPS4.povDown().onTrue(Commands.runOnce(()->{
                                        armElevator.startManualOuttake();
                                })).onFalse(Commands.runOnce(()->{
                                        armElevator.stopIntake();
                                }));

                                //Alignment Assistance Buttons
                                        driverPS4.R3().onTrue(Commands.runOnce(()->{
                                                drivebase.drive(new ChassisSpeeds(0, -0.1, 0));
                                        }));
                                        driverPS4.L3().onTrue(Commands.runOnce(()->{
                                                drivebase.drive(new ChassisSpeeds(0, 0.1, 0));
                                        }));

                                // driverPS4.povUp().onTrue(Commands.runOnce(() -> {
                                //         selectedLevel = -3;
                                //         armElevator.goToLevelCommand(-3).schedule();
                                // }, armElevator));

                                // driverPS4.povDown().onTrue(Commands.runOnce(() -> {
                                //         selectedLevel = -2;
                                //         armElevator.goToLevelCommand(-2).schedule();
                                // }, armElevator));
                        } else {
                                driverPS4.triangle().onTrue(Commands.runOnce(() -> {
                                        selectedLevel = 4;
                                        armElevator.goToLevelCommand(4).schedule();
                                }, armElevator));

                                driverPS4.square().onTrue(Commands.runOnce(() -> {
                                        selectedLevel = 3;
                                        armElevator.goToLevelCommand(3).schedule();
                                }, armElevator));
                                driverPS4.circle().onTrue(Commands.runOnce(() -> {
                                        selectedLevel = 2;
                                        armElevator.goToLevelCommand(2).schedule();
                                }, armElevator));

                                driverPS4.cross().onTrue(Commands.runOnce(() -> {
                                        armElevator.goToFunnelCommand().schedule();
                                }, armElevator));

                                driverPS4.L1().onTrue(Commands.runOnce(() -> {
                                        selectedLevel = 0;
                                        armElevator.goToStowCommand().schedule();
                                }, armElevator));

                                driverPS4.R2().whileTrue(Commands.runOnce(() -> {
                                        armElevator.startManualOuttake();
                                })).onFalse(Commands.runOnce(() -> {
                                        armElevator.stopIntake();
                                }));

                                driverPS4.L2().whileTrue(Commands.runOnce(() -> {
                                        armElevator.startManualIntake();
                                })).onFalse(Commands.runOnce(() -> {
                                        armElevator.stopIntake();
                                }));

                                driverPS4.povRight().whileTrue(Commands.runOnce(() -> {
                                        armElevator.setManualArm(0.1);
                                })).onFalse(Commands.runOnce(() -> {
                                        armElevator.stopManualArm();
                                }));

                                driverPS4.povLeft().whileTrue(Commands.runOnce(() -> {
                                        armElevator.setManualArm(-0.1);
                                })).onFalse(Commands.runOnce(() -> {
                                        armElevator.stopManualArm();
                                }));
                                //Alignment Assistance Buttons
                                        driverPS4.R3().onTrue(Commands.runOnce(()->{
                                                drivebase.drive(new ChassisSpeeds(0, -0.1, 0));
                                        }));
                                        driverPS4.L3().onTrue(Commands.runOnce(()->{
                                                drivebase.drive(new ChassisSpeeds(0, 0.1, 0));
                                        }));
                                
                                // driverPS4.povUp().onTrue(Commands.runOnce(() -> {
                                //         selectedLevel = -3;
                                //         armElevator.goToLevelCommand(-3).schedule();
                                // }, armElevator));

                                // driverPS4.povDown().onTrue(Commands.runOnce(() -> {
                                //         selectedLevel = -2;
                                //         armElevator.goToLevelCommand(-2).schedule();
                                // }, armElevator));
                                
                        }

                        // Aux PS4 Controls
                        auxPS4.triangle().onTrue(Commands.runOnce(() -> {
                                selectedLevel = 4;
                                armElevator.goToLevelCommand(4).schedule();
                        }, armElevator));

                        auxPS4.square().onTrue(Commands.runOnce(() -> {
                                selectedLevel = 3;
                                armElevator.goToLevelCommand(3).schedule();
                        }, armElevator));
                        auxPS4.circle().onTrue(Commands.runOnce(() -> {
                                selectedLevel = 2;
                                armElevator.goToLevelCommand(2).schedule();
                        }, armElevator));

                        auxPS4.cross().onTrue(Commands.runOnce(() -> {
                                armElevator.goToFunnelCommand().schedule();
                        }, armElevator));

                        auxPS4.L1().onTrue(Commands.runOnce(() -> {
                                selectedLevel = 0;
                                armElevator.goToStowCommand().schedule();
                        }, armElevator));

                        //Alignment Assistance Buttons
                                auxPS4.R3().onTrue(Commands.runOnce(()->{
                                        drivebase.drive(new ChassisSpeeds(0, -0.1, 0));
                                }));
                                auxPS4.L3().onTrue(Commands.runOnce(()->{
                                        drivebase.drive(new ChassisSpeeds(0, 0.1, 0));
                                }));
                }

                // Score Command (AUX)
                if (ENABLE_ARM_ELEVATOR_SUBSYSTEM) {
                        driverPS4.R1().onTrue(Commands.runOnce(() -> {
                                // Universal method that transitions to the appropriate "score" preset
                                armElevator.goToLevelScoreCommand(selectedLevel, true).schedule();
                        }, armElevator));
                        auxPS4.R1().onTrue(Commands.runOnce(() -> {
                                // Universal method that transitions to the appropriate "score" preset
                                armElevator.goToLevelScoreCommand(selectedLevel, true).schedule();
                        }, armElevator));
                }

                // Manual Elevator Control (AUX)
                // -------------------------------------------
                // Aux PS4 joysticks for manual control of the arm and elevator:
                //
                // - Left Y-axis:
                // → Manually controls elevator movement
                // → Positive/negative stick input adjusts height up/down
                // → Applies a small deadband to prevent drift
                //
                // - Right Y-axis:
                // → Manually controls arm movement
                // → Positive/negative stick input rotates the arm up/down
                // → Also applies a deadband to ignore minor noise
                //
                // Releasing either stick will stop the respective motor.
                // -------------------------------------------
                if (ENABLE_MANUAL_CONTROL) {
                        new Trigger(() -> Math.abs(auxPS4.getLeftY()) > 0.1).whileTrue(Commands.run(() -> {
                                double raw = -auxPS4.getLeftY();
                                double val = (Math.abs(raw) < 0.05) ? 0.0 : raw;
                                armElevator.setManualElevatorSpeed(val, 0.0);
                        }, armElevator)).onFalse(Commands.runOnce(() -> armElevator.stopManualElevator(),
                                        armElevator));
                        new Trigger(() -> Math.abs(auxPS4.getRightY()) > 0.1)
                                        .whileTrue(Commands.run(() -> {
                                                double raw = auxPS4.getRightY();
                                                double val = (Math.abs(raw) < 0.05) ? 0.0 : raw;
                                                armElevator.setManualArm(val);
                                        }, armElevator)).onFalse(Commands.runOnce(
                                                        () -> armElevator.stopManualArm(), armElevator));
                }

                // Swerve Control
                // -------------------------------------------
                // Set the default drive command based on environment:
                // - If running in Simulation, use direct angle control (driveDirectAngle)
                // - If on a physical robot, use angular velocity control (driveAngularVelocity)
                // This ensures more predictable heading control during simulation
                // testing.drivebase
                // -------------------------------------------
                if (ENABLE_DRIVEBASE_SUBSYSTEM) {
                        if (RobotBase.isSimulation()) {
                                drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));
                        } else {
                                drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));
                        }
                }

                // Gyro ReZero
                // -------------------------------------------
                // Map the PS4 Triangle button to zero the gyro:
                // - When pressed, resets the robot's heading to 0°
                // - Useful for re-aligning field-relative driving during a match
                // -------------------------------------------
                if (ENABLE_DRIVEBASE_SUBSYSTEM) {
                        driverPS4.options().onTrue(Commands.runOnce(drivebase::zeroGyro, drivebase));
                        if (varunMain) {
                                driverPS4.share().onTrue(Commands.runOnce(drivebase::zeroGyro, drivebase));
                        }
                }

        }

        /**
         * Provides the autonomous command to be scheduled in auto mode.
         */
        public Command getAutonomousCommand() {
               return AutonChooser.getSelected();
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
