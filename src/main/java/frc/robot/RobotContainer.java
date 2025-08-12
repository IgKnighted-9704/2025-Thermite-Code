
package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.miscellaneous.ArmElevatorEndEffectorSubsystem;
import frc.robot.subsystems.miscellaneous.AlgaeIntakeSubsystem;
import java.io.File;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * RobotContainer sets up the operator interface, button bindings, and default commands. It also
 * provides the selected autonomous command for the main robot.
 */
public class RobotContainer {

        // Controller Initialization
                // Primary Driver (Drive Train) -> driverPS4
                        private final CommandPS4Controller driverPS4 = new CommandPS4Controller(0);
                // Secondary Driver (Arm/Intake) -> auxPS4
                        private final CommandPS4Controller auxPS4 = new CommandPS4Controller(1);

        // Subsystem Initialization
                //Drive Base
                        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                                        new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
                //Algae Intake
                        private final AlgaeIntakeSubsystem algaeIntake = new AlgaeIntakeSubsystem();
                // Arm Elevator End Effector
                        private final ArmElevatorEndEffectorSubsystem armElevator =
                                        new ArmElevatorEndEffectorSubsystem(drivebase);

        // Default Drive
                // -------------------------------------------
                // Swerve input streams for driving the robot.
                // This first stream uses joystick input for manual control:
                // - Left stick controls translational movement (forward/backward, left/right).
                // - Right stick X-axis controls angular velocity (how fast the robot rotates).
                // - Controls are alliance-relative, meaning they adjust based on which side of the field you're on.
                // -------------------------------------------
                        SwerveInputStream driveAngularVelocity = SwerveInputStream
                                        .of(drivebase.getSwerveDrive(), () -> -driverPS4.getLeftY(),
                                                        () -> -driverPS4.getLeftX())
                                        .withControllerRotationAxis(() -> -driverPS4.getRightX())
                                        .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8)
                                        .allianceRelativeControl(true);
        //Simulation Drive
                // -------------------------------------------
                // This second input stream uses direct angle control for robot heading:
                // - Left stick still controls translational movement.
                // - Right stick (X and Y axes together) sets the direction the robot should face.
                // - The robot automatically rotates to face that direction (field-oriented).
                // - Controls are alliance-relative, so intended heading adjusts based on alliance side.
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
        
        //Subsystem Disable/Enable
                boolean ENABLE_ARM_ELEVATOR_SUBSYSTEM = true; // Set to false to disable the Arm Elevator subsystem
                boolean ENABLE_ALGAE_INTAKE_SUBSYSTEM = false; // Set to false to disable the Algae Intake subsystem
                boolean ENABLE_DRIVEBASE_SUBSYSTEM = true; // Set to false to disable the Drivebase subsystem
                boolean ENABLE_MANUAL_CONTROL = true; // Set to false to disable manual control of the Arm Elevator subsystem

        //Autonomous

        //Path Chooser
        SendableChooser<Command> AutonChooser;
                //AUTONOMOUS COMMANDS
                        //PATH - 1
                                private Command L4AutonomousCommand = new SequentialCommandGroup(
                                        new InstantCommand(() -> {
                                                armElevator.goToFunnelCommand().schedule();
                                }),                                                             // goes to funnel position
                                        new WaitCommand(2),                             // waits 2 seconds
                                        new InstantCommand(() -> {
                                        selectedLevel = 4;
                                        armElevator.goToLevelCommand(4).schedule();
                                }),                                                             // goes to level 4
                                        new WaitCommand(2),                             // waits 2 seconds
                                        new InstantCommand(() -> {
                                        drivebase.driveCommand(() -> 0.25/Constants.DrivebaseConstants.VELOCITY_DRIVE_RATIO, () -> 0, () -> 0).schedule(); // drives with a velocity of 0.5 m/s 
                                }),             
                                        new WaitCommand(5),                                     // waits 1 second
                                        new InstantCommand(() -> {
                                        drivebase.driveCommand(() -> 0, () -> 0, () -> 0).schedule();   // stops the drivebase
                                }),     
                                        new WaitCommand(1.5),                                   // wait 1.5 seconds
                                        new InstantCommand(() -> {
                                        armElevator.goToLevelScoreCommand(4).schedule();          // score L4
                                }),     
                                        new WaitCommand(1.5),                                   // waits 1.5 seconds
                                        new InstantCommand(() -> {
                                        armElevator.goToLevelCommand(4).schedule();
                                }),                                                                     // goes back to level 4
                                        new WaitCommand(1), 
                                        new InstantCommand(() -> {
                                        armElevator.startManualOuttake();                               // starts manual outtake
                                }),
                                        new WaitCommand(2),                                     // waits 2 seconds
                                        new InstantCommand(() -> {                                      
                                        armElevator.stopIntake();                                       // stops the intake
                                }),     new InstantCommand(() -> {
                                        armElevator.goToStowCommand().schedule();                       // goes to stow position
                                }));;
                //SCORE COMMANDS
                        //SCORE L4
                                private Command scoreLevel4Command = new InstantCommand(() -> {
                                        selectedLevel = 4;
                                        armElevator.goToLevelCommand(4).schedule();
                                }, armElevator);
                        //SCORE L3
                                private Command scoreLevel3Command = new InstantCommand(() -> {
                                        selectedLevel = 3;
                                        armElevator.goToLevelCommand(3).schedule();
                                }, armElevator);
                        //SCORE L2
                                private Command scoreLevel2Command = new InstantCommand(() -> {
                                        selectedLevel = 2;
                                        armElevator.goToLevelCommand(2).schedule();
                                }, armElevator);
                        //GO TO FUNNEL
                                private Command goToFunnelCommand = new InstantCommand(() -> {
                                        armElevator.goToFunnelCommand().schedule();
                                }, armElevator);
                        //GO TO STOW
                                private Command goToStowCommand = new InstantCommand(() -> {
                                        armElevator.goToStowCommand().schedule();
                                }, armElevator);

        public RobotContainer() {
                // Named command registration for path planner usage.
                        NamedCommands.registerCommand("goToLevel4Score", scoreLevel4Command);
                        NamedCommands.registerCommand("goToLevel3Score", scoreLevel3Command);
                        NamedCommands.registerCommand("goToLevel2Score", scoreLevel2Command);
                        NamedCommands.registerCommand("goToFunnel", goToFunnelCommand);
                        NamedCommands.registerCommand("goToStow", goToStowCommand);

                configureBindings();

                // Hide joystick connection warnings (optional).
                DriverStation.silenceJoystickConnectionWarning(true);

                //Path Planner Chooser
                AutonChooser = new SendableChooser<>();
                        AutonChooser.addOption("Straight Path", drivebase.getTestDriveStraight(10,0.5));
                        AutonChooser.addOption("Straight Auton-Pathplanner", drivebase.getAutonomousCommand("Straight Auton"));
                        AutonChooser.addOption("Rotate Auton-Pathplanner", drivebase.getAutonomousCommand("Rotate Auton"));
                        AutonChooser.addOption("Two Step Approach", drivebase.getTestTwoStepApproach(2, 0, 0, 0));
                        AutonChooser.addOption("Coral L4 Score", L4AutonomousCommand);
                SmartDashboard.putData("Drive Auton", AutonChooser);
        }

        /**
         * Defines and configures all button bindings and default commands.
         */
        private void configureBindings() {

                //Algae Intake (MAIN)
                        // -------------------------------------------
                        // Maps PS4 controller triggers to control the Algae Intake pivot and rollers:
                        // - Holding L2 pivots the intake down and runs the intake forward
                        //   (on release: disables PID and slows the intake)
                        // - Holding R2 runs the intake in reverse
                        //   (on release: stops both pivot and intake)
                        // -------------------------------------------
                                if(ENABLE_ALGAE_INTAKE_SUBSYSTEM){
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
                                }

                // End Effector Manual Control (AUX)
                        // -------------------------------------------
                        // Maps Aux PS4 controller triggers to control manual intake and outtake:
                        // - Holding R2 activates the intake (pulls game piece in)
                        //     → On release, slows the intake for smoother handoff or hold
                        // - Holding L2 activates the outtake (pushes game piece out)
                        //     → On release, stops the intake completely
                        // -------------------------------------------
                                if(ENABLE_ARM_ELEVATOR_SUBSYSTEM){
                                        auxPS4.R2()
                                        .whileTrue(Commands.run(() -> armElevator.startManualIntake(),
                                                        armElevator))
                                        .onFalse(Commands.runOnce(armElevator::slowIntake, armElevator));

                                        auxPS4.L2()
                                        .whileTrue(Commands.run(() -> armElevator.startManualOuttake(),
                                                        armElevator))
                                        .onFalse(Commands.runOnce(armElevator::stopIntake, armElevator));
                                }

                                
                //Elevator + Arm Control (AUX)
                        // -------------------------------------------
                        // Aux PS4 controller button mappings for arm/elevator presets and special positions:
                        // 
                        // • Face Buttons:
                        //   - Triangle        => Go to Level 4 (highest scoring position)
                        //   - Circle          => Go to Level 3
                        //   - Cross           => Go to Level 2
                        //   - Square          => Loading station position (for game piece pickup)
                        //
                        // • D-Pad (POV):
                        //   - Up              => Funnel position (e.g., for scoring/drop-in zone)
                        //   - Down            => Stow everything (safe driving configuration)
                                if(ENABLE_ARM_ELEVATOR_SUBSYSTEM){
                                        auxPS4.triangle().onTrue(Commands.runOnce(() -> {
                                                selectedLevel = 4;
                                                armElevator.goToLevelCommand(4).schedule();
                                        }, armElevator));
        
                                        auxPS4.circle().onTrue(Commands.runOnce(() -> {
                                                selectedLevel = 3;
                                                armElevator.goToLevelCommand(3).schedule();
                                        }, armElevator));
                                        auxPS4.cross().onTrue(Commands.runOnce(() -> {
                                                selectedLevel = 2;
                                                armElevator.goToLevelCommand(2).schedule();
                                        }, armElevator));
        
                                        auxPS4.povUp().onTrue(Commands.runOnce(() -> {
                                                armElevator.goToFunnelCommand().schedule();
                                        }, armElevator));
        
                                        auxPS4.square().onTrue(Commands.runOnce(() -> {
                                                armElevator.goToLoadingCommand().schedule();
                                        }, armElevator));
        
                                        auxPS4.povDown().onTrue(Commands.runOnce(() -> {
                                                selectedLevel = 0;
                                                armElevator.goToStowCommand().schedule();
                                        }, armElevator));
                                }

                //CORALIN INTAKE
                                if(ENABLE_ARM_ELEVATOR_SUBSYSTEM){
                                        auxPS4.L2().whileTrue(armElevator.CoralIntakeCommand());
                                }

                //Vision Based Drive
                        //
                                if(ENABLE_DRIVEBASE_SUBSYSTEM){
                                        driverPS4.povLeft().onTrue(Commands.run(()->{
                                                armElevator.createReefScoreCommand(true).schedule();
                                        }));
                                        driverPS4.povRight().onTrue(Commands.run(()->{
                                                armElevator.createReefScoreCommand(false).schedule();
                                        }));
                                }
                
                //Score Command (AUX)
                        // -------------------------------------------
                        // Maps the Right Bumper to move the elevator to the "score" position
                        // for the currently selected level (e.g., Level 1–4).
                        // Triggers the preset scoring height based on operator selection.
                        // -------------------------------------------
                                if(ENABLE_ARM_ELEVATOR_SUBSYSTEM){
                                        auxPS4.R1().onTrue(Commands.runOnce(() -> {
                                                // Universal method that transitions to the appropriate "score" preset
                                                armElevator.goToLevelScoreCommand(selectedLevel).schedule();
                                        }, armElevator));
                                }

                //Manual Elevator Control (AUX)
                        // -------------------------------------------
                        // Aux PS4 joysticks for manual control of the arm and elevator:
                        //
                        // - Left Y-axis:
                        //     → Manually controls elevator movement
                        //     → Positive/negative stick input adjusts height up/down
                        //     → Applies a small deadband to prevent drift
                        //
                        // - Right Y-axis:
                        //     → Manually controls arm movement
                        //     → Positive/negative stick input rotates the arm up/down
                        //     → Also applies a deadband to ignore minor noise
                        //
                        // Releasing either stick will stop the respective motor.
                        // -------------------------------------------
                                if(ENABLE_MANUAL_CONTROL){
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
                        // This ensures more predictable heading control during simulation testing.drivebase
                        // -------------------------------------------
                                if(ENABLE_DRIVEBASE_SUBSYSTEM){
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
                                if(ENABLE_DRIVEBASE_SUBSYSTEM){
                                        driverPS4.triangle().onTrue(Commands.runOnce(drivebase::zeroGyro, drivebase));
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
