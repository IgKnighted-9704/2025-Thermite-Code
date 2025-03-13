// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Robot mass and other global constants
  // 32lbs * kg per pound, removing ~20.3 lbs for manipulator weight if desired.
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592;
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);

  // s, 20ms + 110ms Spark MAX velocity lag (example figure)
  public static final double LOOP_TIME = 0.13;

  // Maximum speed of the robot in meters per second, used to limit acceleration.
  public static final double MAX_SPEED = Units.feetToMeters(15.2);

  // If you had an AutonConstants block, you could keep it here as commented code:
  // public static final class AutonConstants
  // {
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  /**
   * Constants for the drivebase.
   */
  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  /**
   * Operator-related constants (joystick deadbands, etc.).
   */
  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  /**
   * Constants related to navigating or scoring near reefs (AprilTag sets).
   */
  public static final class ReefConstants {

    public static final List<Integer> REEF_RED_IDS = List.of(6, 7, 8, 9, 10, 11);
    public static final List<Integer> REEF_BLUE_IDS = List.of(17, 18, 19, 20, 21, 22);

    public static final double BRANCH_OFFSET_METERS = 0.5;
    public static final double APPROACH_X_OFFSET_METERS = 0.0;
    public static final double RETREAT_DISTANCE_METERS = 1.0;
  }

  /**
   * Constants for the Arm and Elevator subsystems.
   */
  public static final class ArmElevatorConstants {

    // Absolute encoder ratios: if 1 sensor rotation = 360 deg, set to 1.0, etc.
    public static double ARM_ABS_ENC_RATIO = 0.61111111;

    // Motor CAN IDs
    public static final int ELEVATOR_MOTOR_A_ID = 11;
    public static final int ELEVATOR_MOTOR_B_ID = 12;
    public static final int ARM_MOTOR_ID = 13;
    public static final int INTAKE_MOTOR_ID = 10; // The "end effector" Spark MAX

    // Sensor conversion factors
    public static double ELEV_TICKS_PER_INCH = 1.0;

    // Physical constraints for arm and elevator
    public static double ARM_MIN_DEG = -167.0; // Retracted inside robot
    public static double ARM_MAX_DEG = 160.0;
    public static double ELEVATOR_MIN_INCHES = 0.0;
    public static final double ELEVATOR_MAX_INCHES = 84.0;

    // Funnel or intake positioning
    // all elevator heights are -2
    public static final double ARM_LOADING_DEG = -27;
    public static final double ARM_FUNNEL_DEG = -30;
    public static final double ELEV_FUNNEL_SAFE_MIN_INCHES = 15.25;
    public static final double ELEV_FUNNEL_SAFE_MAX_INCHES = 20.5;
    public static final double ELEVATOR_FUNNEL_INCHES = 12.8125;
    public static final double ELEVATOR_FUNNEL_LOADING_INCHES = 16.5;

    // Stow position constants
    public static final double ARM_STOW_DEG = 0.0;
    public static final double ARM_STOW_TOLERANCE_DEG = 4.0;
    public static final double ELEVATOR_SAFE_LOWER_THRESHOLD = 4.0;

    // PID and feedforward values for the arm and elevator
    public static double ARM_kP = 0.03;
    public static double ARM_kI = 0.0;
    public static double ARM_kD = 0.0;
    public static double ELEVATOR_kP = 0.0;
    public static double ELEVATOR_kI = 0.0;
    public static double ELEVATOR_kD = 0.0;
    public static double ELEV_kS = 0.0;
    public static double ELEV_kG = 0.0;
    public static double ELEV_kV = 0.0;
    public static double ELEV_kA = 0.0;

    public static double ELEVATOR_MAX_VEL = 10.0;
    public static double ELEVATOR_MAX_ACC = 20.0;

    // Elevator position presets
    public static final double ELEVATOR_STOW_INCHES = 0.0;
    public static final double ELEVATOR_LEVEL1_INCHES = 8.25;
    public static final double ELEVATOR_LEVEL2_INCHES = 10.0625;
    public static final double ELEVATOR_LEVEL3_INCHES = 19.5;
    public static final double ELEVATOR_LEVEL4_INCHES = 22.8125;

    // Offset used to calculate "score" positions for each level
    public static final double ELEVATOR_SCORE_OFFSET = 10.0;
    public static final double ELEVATOR_LEVEL1_SCORE_INCHES = ELEVATOR_LEVEL1_INCHES - ELEVATOR_SCORE_OFFSET;
    public static final double ELEVATOR_LEVEL2_SCORE_INCHES = ELEVATOR_LEVEL2_INCHES - ELEVATOR_SCORE_OFFSET;
    public static final double ELEVATOR_LEVEL3_SCORE_INCHES = ELEVATOR_LEVEL3_INCHES - ELEVATOR_SCORE_OFFSET;
    public static final double ELEVATOR_LEVEL4_SCORE_INCHES = ELEVATOR_LEVEL4_INCHES - ELEVATOR_SCORE_OFFSET;

    // Corresponding arm angles for these levels
    public static final double ARM_LEVEL1_DEG = 65.0;
    public static final double ARM_LEVEL2_DEG = 99.5;
    public static final double ARM_LEVEL3_DEG = 117;
    public static final double ARM_LEVEL4_DEG = 141;

    // Tolerances for checking if the arm or elevator are at their targets
    public static final double ARM_TOLERANCE_DEG = 2.0;
    public static final double ELEVATOR_TOLERANCE_INCH = 1.0;

    // Safety measure: if the robot is tilted beyond this angle, we adjust
    // elevator/arm
    public static final double TILT_THRESHOLD_DEG = 10.0;

    // Intake constants
    public static final double INTAKE_SPEED = 0.8;
    public static final double INTAKE_STOPPED_RPM = 50.0;

    public static final double ACCEL_LIMIT_SCALE = 0.01;
  }

  /**
   * Climber-related constants.
   */
  public static final class ClimbConstants {
    // Motor IDs for the climbing mechanism
    public static final int CLIMB_MOTOR_A_ID = 16;
    public static final int CLIMB_MOTOR_B_ID = 17;

    // Physical limits for the climb pivot
    public static final double CLIMB_MAX_POS = 100.0;
    public static final double CLIMB_MIN_POS = 0.0;

    // Ideal hold angle when climbing
    public static final double CLIMB_HOLD_ANGLE = 50.0;

    // PID values to control the climb pivot
    public static final double CLIMB_kP = 0.1;
    public static final double CLIMB_kI = 0.0;
    public static final double CLIMB_kD = 0.0;
  }

  /**
   * Constants specific to the Algae Intake subsystem.
   */
  public static final class AlgaeIntakeConstants {
    // Motor IDs (pivot with a CIM in brushed mode, NEO in brushless mode for
    // intake)
    public static final int PIVOT_MOTOR_ID = 14;
    public static final int INTAKE_MOTOR_ID = 15;

    // Limits and preset angle for the intake pivot
    public static final double PIVOT_MIN_ANGLE = 0.0;
    public static final double PIVOT_MAX_ANGLE = 100.0;
    public static final double PIVOT_INTAKE_ANGLE = 30.0;

    // PID constants for pivot angle control
    public static double PIVOT_kP = 0.01;
    public static double PIVOT_kI = 0.0;
    public static double PIVOT_kD = 0.0;

    // Intake power level
    public static final double INTAKE_SPEED = 0.8;
  }

  /**
   * Additional constants for the coral portion of the field or scoring areas.
   */
  public static final class CoralConstants {
    public static final List<Integer> CORAL_RED_IDS = List.of(1, 2);
    public static final List<Integer> CORAL_BLUE_IDS = List.of(12, 13);
    public static final double APPROACH_OFFSET_METERS = 0.3;
  }
}
