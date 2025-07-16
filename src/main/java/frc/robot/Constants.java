// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import swervelib.math.Matter;

/**
 * The Constants class provides a single location for storing numeric and boolean constants used
 * throughout the robot code. This class should not include any functional logic—only public static
 * declarations for constants.
 */
public final class Constants {

  // Overall robot mass settings (in kilograms). Subtract manipulator weight if needed.
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592;
  // We define our chassis as a "Matter" object, specifying the center of mass.
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);

  // Estimated loop time in seconds. Example: 20 ms plus some measured delay for Spark MAX.
  public static final double LOOP_TIME = 0.13;

  // Maximum robot speed in meters per second, used to constrain acceleration.
  public static final double MAX_SPEED = Units.feetToMeters(15.3);

  // If you wanted an AutonConstants block, you could define it here:
  // public static final class AutonConstants {
    // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  /**
   * Constants dedicated to the drivebase subsystem.
   */
  public static final class DrivebaseConstants {

    // Time (in seconds) to hold motor brake locks after the robot is disabled.
    public static final double WHEEL_LOCK_TIME = 10;

    public static final double VELOCITY_DRIVE_RATIO = 3.73;
  }

  /**
   * Constants for operator controls (joysticks, deadbands, etc.).
   */
  public static class OperatorConstants {

    // Standard joystick deadband to filter low-value noise.
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;

    // A scaling factor for turning sensitivity.
    public static final double TURN_CONSTANT = 6;
  }

  /**
   * Constants related to navigating or scoring near reefs using AprilTags.
   */
  public static final class ReefConstants {

    public static final List<Integer> REEF_RED_IDS = List.of(6, 7, 8, 9, 10, 11);
    public static final List<Integer> REEF_BLUE_IDS = List.of(17, 18, 19, 20, 21, 22);

    // Offsets in meters used when approaching or retreating from reef locations.
    public static final double BRANCH_OFFSET_METERS = 0.5; //TODO : adjust based on field measurements
    public static final double APPROACH_X_OFFSET_METERS = 0.0;  //TODO : adjust based on field measurements
    public static final double RETREAT_DISTANCE_METERS = 1.0; // TODO : adjust based on field measurements
  }

  /**
   * Constants for the Arm and Elevator subsystems, including motors, encoders, feedforward/PID
   * values, and preset positions.
   */
  public static final class ArmElevatorConstants {

    // Ratio used by the arm’s absolute encoder. If 1 sensor rotation = 360°, set to 1.0, etc.
    public static double ARM_ABS_ENC_RATIO = 0.611111;
    // Offset for the absolute encoder, if needed.
    public static double ARM_ABS_ENC_OFFSET = 10.18; 

    // CAN IDs for the elevator and arm motors (Falcons or Spark MAX, as appropriate).
    public static final int ELEVATOR_MOTOR_A_ID = 11;
    public static final int ELEVATOR_MOTOR_B_ID = 12;
    public static final int ARM_MOTOR_ID = 13;
    public static final int INTAKE_MOTOR_ID = 10; // The end effector motor.

    // Conversion factor from sensor ticks to elevator inches.
    public static double ELEV_TICKS_PER_INCH = 0.7290445833333333;

    // // public static double ARM_MIN_DEG = -167.0; // Retracted inside robot
    // // public static double ARM_MAX_DEG = 160.0;
    // // public static double ELEVATOR_MIN_INCHES = 0.0;
    // // public static final double ELEVATOR_MAX_INCHES = 84.0;

    // Angles and positions for funnel/loading presets.
    public static final double ARM_LOADING_DEG = -40;
    public static final double ARM_FUNNEL_DEG = -40;
    // // public static final double ELEV_FUNNEL_SAFE_MIN_INCHES = 15.25;
    // // public static final double ELEV_FUNNEL_SAFE_MAX_INCHES = 20.5;
    public static final double ELEVATOR_FUNNEL_INCHES = 15;
    public static final double ELEVATOR_FUNNEL_LOADING_INCHES = 9;

    // Stow positioning for the arm.
    public static final double ARM_STOW_DEG = 0.0;
    public static final double ARM_SCORE_DEG = 25.0;
    // // public static final double ARM_STOW_TOLERANCE_DEG = 4.0;
    // // public static final double ELEVATOR_SAFE_LOWER_THRESHOLD = 4.0;

    // PID and feedforward parameters for both the arm and the elevator.
    public static double ARM_kP = 0.03;
    public static double ARM_kI = 0.0;
    public static double ARM_kD = 0.0;

    public static double ELEVATOR_kP = 3.0;
    public static double ELEVATOR_kI = 0.0;
    public static double ELEVATOR_kD = 0.0;

    public static double ELEV_kS = 0.0;
    public static double ELEV_kG = 0.41;
    public static double ELEV_kV = 4.2;
    public static double ELEV_kA = 0.5;

    // Constraints for the elevator’s maximum velocity and acceleration (m/s and m/s^2).
    public static double ELEVATOR_MAX_VEL = 1.5;
    public static double ELEVATOR_MAX_ACC = 1;

    // Specific preset positions for the elevator (inches from zero reference).
    public static final double ELEVATOR_STOW_INCHES = 0.0;
    public static final double ELEVATOR_LEVEL1_INCHES = 8.25;
    public static final double ELEVATOR_LEVEL2_INCHES = 0;
    public static final double ELEVATOR_LEVEL3_INCHES = 19.5;
    public static final double ELEVATOR_LEVEL4_INCHES = 33.5;

    // Offsets applied for score positions at each elevator level.
    public static final double ELEVATOR_SCORE_OFFSET = 12.0;
    public static final double ELEVATOR_LEVEL1_SCORE_INCHES =
        ELEVATOR_LEVEL1_INCHES - ELEVATOR_SCORE_OFFSET;
    public static final double ELEVATOR_LEVEL2_SCORE_INCHES =
        ELEVATOR_LEVEL2_INCHES - ELEVATOR_SCORE_OFFSET;
    public static final double ELEVATOR_LEVEL3_SCORE_INCHES =
        ELEVATOR_LEVEL3_INCHES - ELEVATOR_SCORE_OFFSET;
    public static final double ELEVATOR_LEVEL4_SCORE_INCHES =
        ELEVATOR_LEVEL4_INCHES - ELEVATOR_SCORE_OFFSET;

    // Corresponding arm angles for each level preset.
    public static final double ARM_LEVEL1_DEG = 65.0;
    public static final double ARM_LEVEL2_DEG = 141;
    public static final double ARM_LEVEL3_DEG = 117;
    public static final double ARM_LEVEL4_DEG = 141;

    // // public static final double ARM_TOLERANCE_DEG = 2.0;
    // // public static final double ELEVATOR_TOLERANCE_INCH = 1.0;
    // // public static final double TILT_THRESHOLD_DEG = 10.0;
    // // public static final double ACCEL_LIMIT_SCALE = 0.01;

    // Intake constants
    public static final double INTAKE_SPEED = 0.7;
    public static final double INTAKE_STOPPED_RPM = 6000.0;
  }

  /**
   * Constants for the climbing subsystem (pivot angles, motor IDs, etc.).
   */
  public static final class ClimbConstants {
    // Climb motor CAN IDs.
    public static final int CLIMB_MOTOR_A_ID = 16;
    public static final int CLIMB_MOTOR_B_ID = 17;

    // Physical range for the climb pivot mechanism.
    public static final double CLIMB_MAX_POS = 100.0;
    public static final double CLIMB_MIN_POS = 0.0;

    // The ideal pivot angle we want to hold while climbing.
    public static final double CLIMB_HOLD_ANGLE = 50.0;

    // PID constants for controlling the climb pivot angle.
    public static final double CLIMB_kP = 0.1;
    public static final double CLIMB_kI = 0.0;
    public static final double CLIMB_kD = 0.0;
  }

  /**
   * Constants for the Algae Intake subsystem (motor IDs, angles, PID, etc.).
   */
  public static final class AlgaeIntakeConstants {
    // Motor IDs for the pivot (CIM, brushed) and the intake (NEO, brushless).
    public static final int PIVOT_MOTOR_ID = 14;
    public static final int INTAKE_MOTOR_ID = 15;

    // Limits and presets for the intake pivot angle.
    public static final double PIVOT_MIN_ANGLE = 0.0;
    public static final double PIVOT_MAX_ANGLE = 300.0;
    public static final double PIVOT_INTAKE_ANGLE = 75.0;

    // PID constants for pivot control.
    public static double PIVOT_kP = 0.05;
    public static double PIVOT_kI = 0.0;
    public static double PIVOT_kD = 0.0;

    // Intake speed (fraction of max).
    public static final double INTAKE_SPEED = 0.8;

    public static final double INTAKE_SLOW = 0.1;
  }

  /**
   * Constants associated with coral elements on the field or certain scoring areas.
   */
  public static final class CoralConstants {
    public static final List<Integer> CORAL_RED_IDS = List.of(1, 2);
    public static final List<Integer> CORAL_BLUE_IDS = List.of(12, 13);
    public static final double APPROACH_OFFSET_METERS = 0.3;
  }
}
