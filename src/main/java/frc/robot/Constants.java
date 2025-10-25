// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.List;
import swervelib.math.Matter;

import frc.robot.subsystems.swervedrive.Cameras;

/**
 * The Constants class provides a single location for storing numeric and
 * boolean constants used
 * throughout the robot code. This class should not include any functional
 * logic—only public static
 * declarations for constants.
 */
public final class Constants {

  // Overall robot mass settings (in kilograms). Subtract manipulator weight if
  // // needed.
  // public static final double ROBOT_MASS = (148 - 20.3) * 0.453592;
  public static final double ROBOT_MASS = 46.720;
  // We define our chassis as a "Matter" object, specifying the center of mass.
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);

  // Estimated loop time in seconds. Example: 20 ms plus some measured delay for
  // Spark MAX.
  public static final double LOOP_TIME = 0.13;

  // Maximum robot speed in meters per second, used to constrain acceleration.
  public static final double MAX_SPEED = Units.feetToMeters(15.3);

  /**
   * Constants dedicated to the drivebase subsystem.
   */
  public static final class DrivebaseConstants {

    // Time (in seconds) to hold motor brake locks after the robot is disabled.
    public static final double WHEEL_LOCK_TIME = 10;

    public static final double VELOCITY_DRIVE_RATIO = 3.73;
  }

  public static final class PathplannerConstants {
    public static  double PATH_PLANNER_TRANSLATION_KP = 5.0;
    public static  double PATH_PLANNER_TRANSLATION_KD = 0;
    public static  double PATH_PLANNER_TRANSLATION_KI = 0.0;

    public static  double PATH_PLANNER_ROTATION_KP = 7.5;
    public static  double PATH_PLANNER_ROTATION_KD = 0.0;
    public static  double PATH_PLANNER_ROTATION_KI = 0.5;
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
    public static final double BRANCH_OFFSET_METERS = 0.5;
    public static final double APPROACH_X_OFFSET_METERS = 0.0;
    public static final double RETREAT_DISTANCE_METERS = 1.0;
  }

  /**
   * Constants for the Arm and Elevator subsystems, including motors, encoders,
   * feedforward/PID
   * values, and preset positions.
   */
  public static final class ArmElevatorConstants {
    // Offset applied to the arm’s absolute encoder reading to align it with the
    // robot's zero.
    public static double ARM_ABS_ENC_OFFSET = 86.456;

    // CAN IDs for the elevator and arm motors (Falcons or Spark MAX, as
    // appropriate).
    public static final int ELEVATOR_MOTOR_A_ID = 11;
    public static final int ELEVATOR_MOTOR_B_ID = 12;
    public static final int ARM_MOTOR_ID = 13;
    public static final int INTAKE_MOTOR_ID = 10; // The end effector motor.
    public static final int ARM_ENCODER_ID = 17;
    public static final int CORAL_FUNNEL_SENSOR_ID = 1;
    public static final int CORAL_IN_ENDEFFECTOR_SENSOR_ID = 2;

    // Conversion factor from sensor ticks to elevator inches.
    public static double ELEV_TICKS_PER_INCH = 0.7290445833333333;

    // Angles and positions for funnel/loading presets.
    public static final double ARM_LOADING_DEG = -53.5; //TODO : Fix Loading Angle
    public static final double ARM_FUNNEL_DEG = -53.5; //TODO : Fix Funnel Angle

    public static final double ELEVATOR_FUNNEL_INCHES = 14.790200;
    public static final double ELEVATOR_FUNNEL_LOADING_INCHES = 12.67; //TODO : Fix Loading Height

    // Stow positioning for the arm.
    public static final double ARM_STOW_DEG = 0;
    public static final double ARM_SCORE_DEG_OFFSET = 24;

    // PID and feedforward parameters for both the arm and the elevator.
    public static double ARM_kP = 0.0375;
    public static double ARM_kI = 0.0; 
    public static double ARM_kD = 0.0;

    public static double ELEVATOR_kP = 10.0;
    public static double ELEVATOR_kI = 0.0;
    public static double ELEVATOR_kD = 0.0;

    public static double ELEV_kS = 0.0;
    public static double ELEV_kG = 0.41;
    public static double ELEV_kV = 4.0;
    public static double ELEV_kA = 0.5;

    // Constraints for the elevator’s maximum velocity and acceleration (m/s an d
    // m/s^2).
    public static double ELEVATOR_MAX_VEL = 1.5;
    public static double ELEVATOR_MAX_ACC = 1;

    // Specific preset positions for the elevator (inches from zero reference).
    public static final double ELEVATOR_STOW_INCHES = 1;
    public static final double ELEVATOR_LEVEL1_INCHES = 8.25;
    public static final double ELEVATOR_LEVEL2_INCHES = 0;
    public static final double ELEVATOR_LEVEL3_INCHES = 19.5;
    public static final double ELEVATOR_LEVEL4_INCHES = 33.5;
    public static final double ELEVATOR_DEALGAELEVEL3_INCHES = ELEVATOR_LEVEL3_INCHES; 
    public static final double ELEVATOR_DEALGAELEVEL2_INCHES = 0.540; 

    // Offsets applied for score positions at each elevator level.
    public static final double ELEVATOR_SCORE_OFFSET = 12.0;
    public static final double ELEVATOR_LEVEL1_SCORE_INCHES = ELEVATOR_LEVEL1_INCHES - ELEVATOR_SCORE_OFFSET;
    public static final double ELEVATOR_LEVEL2_SCORE_INCHES = ELEVATOR_LEVEL2_INCHES - ELEVATOR_SCORE_OFFSET;
    public static final double ELEVATOR_LEVEL3_SCORE_INCHES = ELEVATOR_LEVEL3_INCHES - ELEVATOR_SCORE_OFFSET;
    public static final double ELEVATOR_LEVEL4_SCORE_INCHES = ELEVATOR_LEVEL4_INCHES - ELEVATOR_SCORE_OFFSET;

    // Corresponding arm angles for each level preset.
    public static final double ARM_LEVEL1_DEG = 0; //TODO : Fix Arm Angles L1
    public static final double ARM_LEVEL2_DEG = 190; //TODO : Fix Arm Angles L2
    public static final double ARM_LEVEL3_DEG = 197.67; //TODO : Fix Arm Angles L3
    public static final double ARM_LEVEL4_DEG = 225; //TODO : Fix Arm Angles L4
    public static final double ARM_DEALGAELEVEL2_DEG = 190;
    public static final double ARM_DEALGAELEVEL3_DEG = 190;

    //Tolerance Constants
    public static final double ELEVATOR_POSITION_TOLERANCE_INCHES = 0.75; //TODO : Fix Tolerance Elevator Height
    public static final double ARM_POSITION_TOLERANCE_DEG = 2.0; //TODO : Fix Tolerance Arm Angle
    public static final double ELEVATOR_MAX_HEIGHT_INCHES = 34.0; //TODO : Fix Max Elevator Height
    public static final double ELEVATOR_MIN_HEIGHT_INCHES = 0.0; //TODO : Fix Min Elevator Height
    public static final double ARM_MAX_ANGLE_DEG = 200.0; //TODO : Fix Max Arm Angle
    public static final double ARM_MIN_ANGLE_DEG = -60.0;  //TODO : Fix Min Arm Angle


    // Intake constants
    public static final double INTAKE_SPEED = 0.5;
    public static final double INTAKE_STOPPED_RPM = 2.1;
    
    //Sensor Constants
    public static final boolean TestEndEffectorSensor = false;
  }

  public static final class VisionConstants {
     public static final Cameras leftCam = new Cameras(
        "CAMERA_LEFT",
        new Rotation3d(Math.toRadians(0), Math.toRadians(25), Math.toRadians(0)),
        new Translation3d(Units.inchesToMeters(27), Units.inchesToMeters(-23), Units.inchesToMeters(8.75)),
        VecBuilder.fill(4, 4, 8),
        VecBuilder.fill(0.5, 0.5, 1)
    );

    public static final Cameras rightCam = new Cameras(
        "CAMERA_RIGHT",
        new Rotation3d(Math.toRadians(0), Math.toRadians(155), Math.toRadians(0)),
        new Translation3d(Units.inchesToMeters(27), Units.inchesToMeters(23), Units.inchesToMeters(8.75)),
        VecBuilder.fill(4, 4, 8),
        VecBuilder.fill(0.5, 0.5, 1)
    );

    public static final ArrayList<Cameras> cameraList = new ArrayList<Cameras>(List.of(leftCam, rightCam));
  }
}
