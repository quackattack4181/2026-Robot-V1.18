package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase implements AutoCloseable {
  private final SparkFlex leftClimberMotor;
  private final SparkFlex rightClimberMotor;
  private final DutyCycleEncoder absoluteEncoder;
  private final NetworkTableEntry climberAngleDegreesEntry =
      NetworkTableInstance.getDefault().getTable("Elastic").getEntry("Climber Angle (deg)");

  public Climber() {
    leftClimberMotor = new SparkFlex(ClimberConstants.LEFT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
    rightClimberMotor = new SparkFlex(ClimberConstants.RIGHT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
    absoluteEncoder = new DutyCycleEncoder(ClimberConstants.ABSOLUTE_ENCODER_CHANNEL);

    SparkFlexConfig leftConfig = new SparkFlexConfig();
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.smartCurrentLimit(ClimberConstants.SPARKFLEX_CURRENT_LIMIT_AMPS);
    leftConfig.inverted(ClimberConstants.LEFT_CLIMBER_INVERTED);
    leftClimberMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig rightConfig = new SparkFlexConfig();
    rightConfig.idleMode(IdleMode.kBrake);
    rightConfig.smartCurrentLimit(ClimberConstants.SPARKFLEX_CURRENT_LIMIT_AMPS);
    rightConfig.inverted(ClimberConstants.RIGHT_CLIMBER_INVERTED);
    rightClimberMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setClimberPower(double power) {
    double clampedPower = Math.max(-ClimberConstants.MAX_ALLOWED_POWER,
        Math.min(ClimberConstants.MAX_ALLOWED_POWER, power));

    if (ClimberConstants.CLIMBER_LIMITS_ENABLED) {
      double currentAngle = getClimberAngleDegrees();
      double backwardLimit = Math.min(ClimberConstants.BACKWARD_MAX_ANGLE_DEGREES,
          ClimberConstants.FORWARD_MAX_ANGLE_DEGREES);
      double forwardLimit = Math.max(ClimberConstants.BACKWARD_MAX_ANGLE_DEGREES,
          ClimberConstants.FORWARD_MAX_ANGLE_DEGREES);

      // Normal stop at configured limits.
      if (clampedPower > 0.0 && currentAngle >= forwardLimit) {
        clampedPower = 0.0;
      }
      if (clampedPower < 0.0 && currentAngle <= backwardLimit) {
        clampedPower = 0.0;
      }

      // If outside limits, only allow motion back into range.
      if (currentAngle < backwardLimit && clampedPower < 0.0) {
        clampedPower = 0.0;
      }
      if (currentAngle > forwardLimit && clampedPower > 0.0) {
        clampedPower = 0.0;
      }
    }

    leftClimberMotor.set(clampedPower);
    rightClimberMotor.set(clampedPower);
  }

  private double wrapToSignedDegrees(double degrees) {
    return ((degrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
  }

  public double getClimberAngleDegrees() {
    double absoluteDegrees = absoluteEncoder.get() * 360.0;
    double angle = wrapToSignedDegrees(absoluteDegrees - ClimberConstants.CLIMBER_ABSOLUTE_ENCODER_OFFSET_DEGREES);
    return ClimberConstants.CLIMBER_ENCODER_DIRECTION_INVERTED ? -angle : angle;
  }

  public void stop() {
    leftClimberMotor.stopMotor();
    rightClimberMotor.stopMotor();
  }

  public Command runClimberPower(double power) {
    return runEnd(() -> setClimberPower(power), this::stop);
  }

  public Command moveToAngleCommand(double targetAngleDegrees) {
    return run(() -> {
      double errorDegrees = targetAngleDegrees - getClimberAngleDegrees();
      double power = MathUtil.clamp(errorDegrees * ClimberConstants.CLIMBER_POSITION_KP,
                                    -ClimberConstants.CLIMBER_POWER,
                                    ClimberConstants.CLIMBER_POWER);
      setClimberPower(power);
    })
        .until(() -> Math.abs(targetAngleDegrees - getClimberAngleDegrees())
            <= ClimberConstants.CLIMBER_POSITION_TOLERANCE_DEGREES)
        .withTimeout(ClimberConstants.CLIMBER_POSITION_TIMEOUT_SECONDS)
        .finallyDo(this::stop);
  }

  public Command moveToDownPositionCommand() {
    return moveToAngleCommand(ClimberConstants.CLIMBER_DOWN_POSITION_DEGREES);
  }

  public Command moveToLevel1PositionCommand() {
    return moveToAngleCommand(ClimberConstants.CLIMBER_LEVEL_1_POSITION_DEGREES);
  }

  public Command moveToLevel2PositionCommand() {
    return moveToAngleCommand(ClimberConstants.CLIMBER_LEVEL_2_POSITION_DEGREES);
  }

  @Override
  public void periodic() {
    // Publish adjusted climber angle (includes configured offset and optional inversion).
    climberAngleDegreesEntry.setDouble(getClimberAngleDegrees());
  }

  @Override
  public void close() {
    leftClimberMotor.close();
    rightClimberMotor.close();
    absoluteEncoder.close();
  }
}
