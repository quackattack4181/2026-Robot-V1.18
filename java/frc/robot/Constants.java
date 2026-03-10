// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  public static final class AutonConstants
  {

   public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; 
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
    public static final boolean TWO_CONTROLLER_MODE = true;
    public static final boolean CLIMBER_ENABLED = true;
    public static final boolean ENABLE_AUTO_FINISH_180_SPIN = false;
    public static final double AUTO_FINISH_SPIN_TIMEOUT_SECONDS = 1.75;
  }
  public static class VisionConstants
  {
    public static final String LIMELIGHT_NAME = "limelight";
    public static final double AIM_KP = 0.08;
    public static final double AIM_KI = 0.001;
    public static final double AIM_KD = 0.002;
    public static final double AIM_TOLERANCE_DEGREES = 0.5;
    public static final double AIM_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 4.0;
    public static final int[] SHOOTING_AIM_TAG_IDS = {5, 8, 9, 10, 11, 12, 18, 27, 26, 25, 21, 24};
    public static final int[] ALLOWED_AIM_TAG_IDS = SHOOTING_AIM_TAG_IDS;
    public static final int[] CLIMBER_APPROVED_TAG_IDS = {10, 26, 12};
    // Linear calibration for Limelight distance inches: calibrated = raw * scale + offset.
    // Offset is currently adjusted for camera relocation (~15 in forward, ~7 in upward).
    public static final double LIMELIGHT_DISTANCE_SCALE = 1.0422535;
    public static final double LIMELIGHT_DISTANCE_OFFSET_INCHES = 15.0;
  }
  public static class CustomConstants
  {


  }

  public static final class ClimbSetupConstants
  {
    // Update these values after parking robot where you want to start climb.
    public static final double LEFT_TARGET_X_METERS = 1.50;
    public static final double LEFT_TARGET_Y_METERS = 6.20;
    public static final double RIGHT_TARGET_LATERAL_OFFSET_INCHES = 32.0;
    // Limelight snapshot target for left climb lineup (from dashboard screenshot).
    public static final double LEFT_TARGET_TX_DEGREES = 5.08;
    public static final double LEFT_TARGET_TY_DEGREES = -12.53;
    public static final double LEFT_TARGET_TA_PERCENT = 0.351;

    // Right-side pose is the same as left, shifted 32 inches to the right (field -Y direction).
    public static final double RIGHT_TARGET_X_METERS = LEFT_TARGET_X_METERS;
    public static final double RIGHT_TARGET_Y_METERS = LEFT_TARGET_Y_METERS
        - Units.inchesToMeters(RIGHT_TARGET_LATERAL_OFFSET_INCHES);
    // Right-side snapshot derived from left. Keep ty/ta same; tx mirrored by sign.
    public static final double RIGHT_TARGET_TX_DEGREES = -LEFT_TARGET_TX_DEGREES;
    public static final double RIGHT_TARGET_TY_DEGREES = LEFT_TARGET_TY_DEGREES;
    public static final double RIGHT_TARGET_TA_PERCENT = LEFT_TARGET_TA_PERCENT;

    // Forward-facing lineup heading (same orientation as right-stick aiming forward).
    public static final double TARGET_HEADING_DEGREES = 0.0;

    public static final Pose2d LEFT_TARGET_POSE = new Pose2d(
        LEFT_TARGET_X_METERS,
        LEFT_TARGET_Y_METERS,
        Rotation2d.fromDegrees(TARGET_HEADING_DEGREES));

    public static final Pose2d RIGHT_TARGET_POSE = new Pose2d(
        RIGHT_TARGET_X_METERS,
        RIGHT_TARGET_Y_METERS,
        Rotation2d.fromDegrees(TARGET_HEADING_DEGREES));

    public static final double APPROACH_TIMEOUT_SECONDS = 5.0;
    public static final double TAG_ACQUIRE_TIMEOUT_SECONDS = 2.0;
    public static final double TAG_ALIGN_TIMEOUT_SECONDS = 3;
    // Final climb dial-in tolerance: start around 6-12 inches and tune from there.
    public static final double POSITION_TOLERANCE_INCHES = 1.0;
    public static final double HEADING_TOLERANCE_DEGREES = 5.0;
    public static final double TAG_LINEUP_HEADING_TOLERANCE_DEGREES = 5.0;
    public static final double LIMELIGHT_TX_TOLERANCE_DEGREES = 1.0;
    public static final double LIMELIGHT_TY_TOLERANCE_DEGREES = 1.5;
    public static final double LIMELIGHT_TA_TOLERANCE_PERCENT = 0.10;
    // Driver test-mode lineup behavior for X/B climb buttons.
    // X/B test-lineup target distance from tag.
    public static final double TAG_LINEUP_X_TARGET_DISTANCE_FEET = 7.75;
    public static final double TAG_LINEUP_X_LATERAL_OFFSET_INCHES = -12.0;
    public static final double TAG_LINEUP_B_LATERAL_OFFSET_INCHES =
        TAG_LINEUP_X_LATERAL_OFFSET_INCHES + RIGHT_TARGET_LATERAL_OFFSET_INCHES;
    public static final double TAG_LINEUP_DISTANCE_TOLERANCE_INCHES = 1.0;
    public static final double TAG_LINEUP_DISTANCE_KP = 1.25;
    public static final double TAG_LINEUP_MAX_FORWARD_MPS = 1.2;
    public static final double TAG_LINEUP_STRAFE_KP = 0.09;
    public static final double TAG_LINEUP_MAX_STRAFE_MPS = 1.0;
    public static final double TAG_LINEUP_MIN_STRAFE_MPS = 0.15;
    public static final double DRIVER_BACKUP_SPEED_MPS = 0.4;
    public static final double DRIVER_BACKUP_DURATION_SECONDS = 2.0;
  }

  public static final class ShooterConstants
  {
    public static final int SHOOTER_INTAKE_MOTOR_ID = 20;
    public static final int MIDDLE_SHOOTER_MOTOR_ID = 21;

    public static final boolean SHOOTER_INTAKE_INVERTED = false;
    public static final boolean MIDDLE_SHOOTER_INVERTED = true;

    public static final int CURRENT_LIMIT_AMPS = 40;
    public static final int SPARKFLEX_CURRENT_LIMIT_AMPS = 80;
    public static final double SHOOTER_INTAKE_POWER = 0.99;
    public static final double SHOOTER_CAL_POINT_NEAR_DISTANCE_FEET = 6.15;
    public static final double SHOOTER_CAL_POINT_NEAR_POWER = 0.495;
    public static final double SHOOTER_CAL_POINT_MID_DISTANCE_FEET = 7.75;
    public static final double SHOOTER_CAL_POINT_MID_POWER = 0.515;
    public static final double SHOOTER_CAL_POINT_FAR_DISTANCE_FEET = 8.34;
    public static final double SHOOTER_CAL_POINT_FAR_POWER = 0.545;
    public static final double SHOOTER_POWER_NO_TAG_DEFAULT = 0.50;
    // Fixed shooter power modes for driver bumpers (independent of Limelight distance).
    public static final double SHOOTER_FIXED_POWER_DRIVER = 0.90;
    public static final double SHOOTER_FIXED_POWER_DRIVER_LOW = 0.50;
    // If true, keep flywheel running at default power when not overridden by driver controls.
    public static final boolean SHOOTER_ALWAYS_ON_ENABLED = true;
    public static final double SHOOTER_ALWAYS_ON_DEFAULT_POWER = SHOOTER_FIXED_POWER_DRIVER_LOW;
    public static final boolean SHOOTER_CALIBRATION_MODE_ENABLED = false;
    public static final double SHOOTER_CALIBRATION_DEFAULT_POWER = 0.40;
    public static final double SHOOTER_POWER_TOLERANCE = 0.02;
    public static final double SHOOTER_INTAKE_START_DELAY_SECONDS = 0.10;

    // OLD (currently unused): legacy shooter PID/F feedforward placeholders.
    public static final double SHOOTER_KP = 0.00003;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.0001;
    public static final double SHOOTER_KF = 0.00025;

    public static final int AGITATOR_MOTOR_ONE_ID = 25;
    public static final int AGITATOR_MOTOR_TWO_ID = 26;
    public static final boolean AGITATOR_MOTOR_ONE_INVERTED = false;
    public static final boolean AGITATOR_MOTOR_TWO_INVERTED = true;
    public static final boolean AGITATOR_ENABLED = false;
    public static final double AGITATOR_POWER = 1.0;

    // OLD (currently unused): legacy intake PID/F placeholders for shooter feed motor.
    public static final double SHOOTER_INTAKE_KP = 1.0;
    public static final double SHOOTER_INTAKE_KI = 0.0;
    public static final double SHOOTER_INTAKE_KD = 0.0;
    public static final double SHOOTER_INTAKE_KF = 0.000;
  }


  public static final class ClimberConstants
  {
    public static final int LEFT_CLIMBER_MOTOR_ID = 40;
    public static final int RIGHT_CLIMBER_MOTOR_ID = 41;
    public static final int ABSOLUTE_ENCODER_CHANNEL = 8;
    public static final boolean LEFT_CLIMBER_INVERTED = false;
    public static final boolean RIGHT_CLIMBER_INVERTED = true;
    public static final int SPARKFLEX_CURRENT_LIMIT_AMPS = 80;
    public static final double MAX_ALLOWED_POWER = 1.00;
    public static final double CLIMBER_POWER = 0.95;
    // Absolute encoder offset in degrees. Set this so your chosen climber zero reads 0.0.
    // Current tuned offset. OLD previous tuned value kept for reference after // .
    public static final double CLIMBER_ABSOLUTE_ENCODER_OFFSET_DEGREES = -47.0;// 19.12;
    // If true, flip encoder sign so climber angle increases/decreases opposite direction.
    public static final boolean CLIMBER_ENCODER_DIRECTION_INVERTED = true;
    public static final boolean CLIMBER_LIMITS_ENABLED = true;                               // Climber Limits
    // Manual climber soft limits after offset/inversion are applied.
    public static final double FORWARD_MAX_ANGLE_DEGREES = 10.0;
    public static final double BACKWARD_MAX_ANGLE_DEGREES = -135.0;

    // Preset climber positions in offset-adjusted degrees.
    public static final double CLIMBER_DOWN_POSITION_DEGREES = -125.0;
    public static final double CLIMBER_LEVEL_1_POSITION_DEGREES = 0.0;
    public static final double CLIMBER_LEVEL_2_POSITION_DEGREES = -27.8; 
    public static final double CLIMBER_POSITION_TOLERANCE_DEGREES = 1.0;
    public static final double CLIMBER_POSITION_KP = 0.03;
    public static final double CLIMBER_POSITION_TIMEOUT_SECONDS = 2.5;
  }

  public static final class IntakeConstants
  {
    public static final int PIVOT_MOTOR_ID = 30;
    public static final int WHEEL_MOTOR_ID = 31;
    public static final boolean PIVOT_INVERTED = true;
    public static final boolean WHEEL_INVERTED = false;
    public static final int PIVOT_CURRENT_LIMIT_AMPS = 40;
    public static final int WHEEL_SPARKFLEX_CURRENT_LIMIT_AMPS = 80;
    public static final double PIVOT_POWER = 0.30;
    public static final boolean PIVOT_LIMITS_ENABLED = true;
    // Absolute encoder offset in degrees. Set this so your desired "zero" angle reads 0.0.
    public static final double PIVOT_ABSOLUTE_ENCODER_OFFSET_DEGREES = -141.0;
    // If true, flip encoder sign so angle increases/decreases opposite direction.
    public static final boolean PIVOT_ENCODER_DIRECTION_INVERTED = false;
    // Manual soft limits (degrees) after applying offset.
    // Typical setup requested: inward = 0.0, outward = positive value.
    public static final double PIVOT_MAX_INWARD_ANGLE = 0.0;
    public static final double PIVOT_MAX_OUTWARD_ANGLE = 96.0;
    public static final double PIVOT_ANGLE_TOLERANCE_DEGREES = 0.5;
    public static final double WHEEL_POWER = 0.99;
    // If true, keep intake wheels running in teleop unless overridden/toggled off.
    public static final boolean INTAKE_WHEELS_ALWAYS_ON_ENABLED = true;
    public static final double INTAKE_WHEELS_ALWAYS_ON_POWER = WHEEL_POWER;
  }
}
