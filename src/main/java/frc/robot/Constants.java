// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);

  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class CANIDConstants {
    public static final int ElevatorA_CANID = 1;
    public static final int ElevatorB_CANID = 2;

    public static final int ClimMotorA_CANID = 3;
    public static final int ClimMotorB_CANID = 4;
    public static final int Dealgify_CANID = 5;
    
    public static final int CoralIntake_CANID = 6;

    public static final int Arm_CANID = 7;

    public static final String CANBUS_Name = "canbus";
  }

  public static class PositionRestraintConstants {
    public static final double armMaxPos = 0;
    public static final double armMinPos = 0;

    public static final double elevatorMaxPos = 0;
    public static final double elevatorMinPos = 0;

    public static final double climbMaxPos = 0;
    public static final double climbMinPos = 0;
  }

  public static class ElevatorPositions{
    public static final double level2 = 0;
    public static final double level3 = 0;
    public static final double level4 = 0;
  }

  public static class ArmPositions{
    public static final double coralOuttakePos = 0;
    public static final double reefIntakePos = 0;
  }

  public static class ClimbPositions {
    public static final double climbEstimatePos = 0;
  }

  public static class VisionTranslationCostants{
    public static final double reefStationAprilTagID1 = 0; //blue -> 17 red -> 6
    public static final double reefStationAprilTagID2 = 0; //blue -> 18 red -> 7
    public static final double reefStationAprilTagID3 = 0; //blue -> 19 red -> 8
    public static final double reefStationAprilTagID4 = 0; //blue -> 20 red -> 9
    public static final double reefStationAprilTagID5 = 0; //blue -> 21 red -> 10
    public static final double reefStationAprilTagID6 = 0; //blue -> 22 red -> 11

    public static final double coralStationAprilTagID = 0; //blue -> 12, 13 red -> 1,2
  }
}
