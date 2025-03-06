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

  public static final class ArmElevatorConstants {
    // Motor CAN IDs
    public static final int ELEVATOR_MOTOR_A_ID = 11;
    public static final int ELEVATOR_MOTOR_B_ID = 12;
    public static final int ARM_MOTOR_ID = 13;
    public static final int INTAKE_MOTOR_ID = 14;

    // Sensor conversion factors
    public static final double ARM_TICKS_PER_DEG = 100.0;
    public static final double ELEV_TICKS_PER_INCH = 50.0;

    // Physical constraints for arm and elevator
    public static final double ARM_MIN_DEG = -30.0; // Retracted inside robot
    public static final double ARM_MAX_DEG = 180.0;
    public static final double ELEVATOR_MIN_INCHES = 0.0;
    public static final double ELEVATOR_MAX_INCHES = 50.0;

    // Funnel or intake positioning
    public static final double ARM_FUNNEL_DEG = -15.0;
    public static final double ELEV_FUNNEL_SAFE_MIN_INCHES = 10.0;
    public static final double ELEV_FUNNEL_SAFE_MAX_INCHES = 30.0;
    public static final double ELEVATOR_FUNNEL_INCHES = 10.0;

    // Stow position constants
    public static final double ARM_STOW_DEG = 0.0;
    public static final double ARM_STOW_TOLERANCE_DEG = 5.0;
    public static final double ELEVATOR_SAFE_LOWER_THRESHOLD = 5.0;

    // PID and feedforward values for the arm and elevator
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

    public static final double ELEVATOR_MAX_VEL = 10.0;
    public static final double ELEVATOR_MAX_ACC = 20.0;

    // Elevator position presets
    public static final double ELEVATOR_STOW_INCHES = 0.0;
    public static final double ELEVATOR_LEVEL1_INCHES = 20.0;
    public static final double ELEVATOR_LEVEL2_INCHES = 30.0;
    public static final double ELEVATOR_LEVEL3_INCHES = 40.0;
    public static final double ELEVATOR_LEVEL4_INCHES = 50.0;

    // Corresponding arm angles for these levels
    public static final double ARM_LEVEL1_DEG = 30.0;
    public static final double ARM_LEVEL2_DEG = 60.0;
    public static final double ARM_LEVEL3_DEG = 90.0;
    public static final double ARM_LEVEL4_DEG = 120.0;

    // Tolerances for checking if the arm or elevator are at their targets
    public static final double ARM_TOLERANCE_DEG = 2.0;
    public static final double ELEVATOR_TOLERANCE_INCH = 1.0;

    // Safety measure: if the robot is tilted beyond this angle, we adjust
    // elevator/arm
    public static final double TILT_THRESHOLD_DEG = 10.0;

    // Intake constants
    public static final double INTAKE_SPEED = 0.8;
    public static final double INTAKE_STOPPED_RPM = 50.0;
  }

  public static final class ClimbConstants {
    // Motor IDs for the climbing mechanism
    public static final int CLIMB_MOTOR_A_ID = 21;
    public static final int CLIMB_MOTOR_B_ID = 22;

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

  public static final class AlgaeIntakeConstants {
    // Motor IDs (CIM in brushed mode for pivot, NEO in brushless mode for intake)
    public static final int PIVOT_MOTOR_ID = 31;
    public static final int INTAKE_MOTOR_ID = 32;

    // Limits and preset angle for the intake pivot
    public static final double PIVOT_MIN_ANGLE = 0.0;
    public static final double PIVOT_MAX_ANGLE = 100.0;
    public static final double PIVOT_INTAKE_ANGLE = 30.0;

    // PID constants for pivot angle control
    public static final double PIVOT_kP = 0.1;
    public static final double PIVOT_kI = 0.0;
    public static final double PIVOT_kD = 0.0;

    // Intake power level
    public static final double INTAKE_SPEED = 0.8;
  }
}