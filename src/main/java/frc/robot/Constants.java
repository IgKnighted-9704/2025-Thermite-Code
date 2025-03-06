// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(15.2);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public final class ArmElevatorConstants {

    // ------------------------------
    // Motor CAN IDs
    // ------------------------------
    public static final int ELEVATOR_MOTOR_A_ID = 11;
    public static final int ELEVATOR_MOTOR_B_ID = 12;
    public static final int ARM_MOTOR_ID = 13;
    public static final int INTAKE_MOTOR_ID = 14;

    // ------------------------------
    // Sensor / Geometry Conversions
    // (ticks -> degrees, ticks -> inches)
    // ------------------------------
    public static final double ARM_TICKS_PER_DEG = 100.0; // FILL THIS
    public static final double ELEV_TICKS_PER_INCH = 50.0; // FILL THIS

    // ------------------------------
    // Physical Limits
    // ------------------------------
    public static final double ARM_MIN_DEG = 0.0; // e.g. stowed
    public static final double ARM_MAX_DEG = 180.0; // e.g. forward

    public static final double ELEVATOR_MIN_INCHES = 0.0;
    public static final double ELEVATOR_MAX_INCHES = 50.0;

    // If the arm wants to go behind the robot (funnel):
    public static final double ARM_BEHIND_DEG = -20.0;
    // Elevator range that’s safe for behind-robot arm:
    public static final double ELEV_BEHIND_SAFE_MIN_INCHES = 10.0;
    public static final double ELEV_BEHIND_SAFE_MAX_INCHES = 30.0;

    // ------------------------------
    // PID Gains + Feedforward
    // (placeholders, tune these)
    // ------------------------------
    public static final double ARM_kP = 0.1;
    public static final double ARM_kI = 0.0;
    public static final double ARM_kD = 0.0;

    public static final double ELEVATOR_kP = 0.1;
    public static final double ELEVATOR_kI = 0.0;
    public static final double ELEVATOR_kD = 0.0;

    public static final double ELEV_kS = 0.0;
    public static final double ELEV_kG = 0.0;
    public static final double ELEV_kV = 0.0;
    public static final double ELEV_kA = 0.0;

    // Elevator motion-profile constraints (inches/sec, inches/sec^2)
    public static final double ELEVATOR_MAX_VEL = 10.0;
    public static final double ELEVATOR_MAX_ACC = 20.0;

    // ------------------------------
    // Preset Positions
    // ------------------------------
    public static final double ARM_STOW_DEG = 0.0;
    public static final double ARM_INTAKE_DEG = -15.0; // behind
    public static final double ARM_LEVEL1_DEG = 30.0;
    public static final double ARM_LEVEL2_DEG = 60.0;
    public static final double ARM_LEVEL3_DEG = 90.0;
    public static final double ARM_LEVEL4_DEG = 120.0;

    public static final double ELEVATOR_STOW_INCHES = 0.0;
    public static final double ELEVATOR_INTAKE_INCHES = 10.0;
    public static final double ELEVATOR_LEVEL1_INCHES = 20.0;
    public static final double ELEVATOR_LEVEL2_INCHES = 30.0;
    public static final double ELEVATOR_LEVEL3_INCHES = 40.0;
    public static final double ELEVATOR_LEVEL4_INCHES = 50.0;

    // ------------------------------
    // Tilt Safety
    // ------------------------------
    public static final double TILT_THRESHOLD_DEG = 10.0;

    // ------------------------------
    // Tolerances
    // ------------------------------
    public static final double ARM_TOLERANCE_DEG = 2.0;
    public static final double ELEVATOR_TOLERANCE_INCH = 1.0;

    // ------------------------------
    // Intake Settings
    // ------------------------------
    /** Speed for auto or manual intake. */
    public static final double INTAKE_SPEED = 0.8; // Placeholder
    /** If the SparkMax velocity (RPM) goes below this, we consider it "stopped." */
    public static final double INTAKE_STOPPED_RPM = 50.0; // Placeholder threshold
  }
}
