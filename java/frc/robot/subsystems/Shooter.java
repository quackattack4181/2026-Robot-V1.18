package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CalibrationConstants;
import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase implements AutoCloseable {
  private final SparkFlex middleShooterMotor;
  private final SparkMax shooterIntakeMotor;
  private final SparkMax agitatorMotorOne;
  private final SparkMax agitatorMotorTwo;
  private final NetworkTableEntry shooterCalibrationPowerEntry;
  private double shooterPidKp = CalibrationConstants.SHOOTER_KP;
  private double shooterPidKi = CalibrationConstants.SHOOTER_KI;
  private double shooterPidKd = CalibrationConstants.SHOOTER_KD;
  private double shooterPidKf = CalibrationConstants.SHOOTER_KF;
  private boolean robotCalibrationModeEnabled = CalibrationConstants.ROBOT_CALIBRATION_MODE_ENABLED;

  private double currentShooterPower = 0.0;

  public Shooter() {
    shooterIntakeMotor = new SparkMax(ShooterConstants.SHOOTER_INTAKE_MOTOR_ID, MotorType.kBrushless);
    middleShooterMotor = new SparkFlex(ShooterConstants.MIDDLE_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    agitatorMotorOne = new SparkMax(ShooterConstants.AGITATOR_MOTOR_ONE_ID, MotorType.kBrushless);
    agitatorMotorTwo = new SparkMax(ShooterConstants.AGITATOR_MOTOR_TWO_ID, MotorType.kBrushless);
    shooterCalibrationPowerEntry = NetworkTableInstance.getDefault()
        .getTable("Elastic")
        .getEntry("Shooter Calibration Power");
    shooterCalibrationPowerEntry.setDouble(ShooterConstants.SHOOTER_CALIBRATION_DEFAULT_POWER);

    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kCoast);
    intakeConfig.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    intakeConfig.inverted(ShooterConstants.SHOOTER_INTAKE_INVERTED);
    shooterIntakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig middleConfig = new SparkFlexConfig();
    middleConfig.idleMode(IdleMode.kCoast);
    middleConfig.smartCurrentLimit(ShooterConstants.SPARKFLEX_CURRENT_LIMIT_AMPS);
    middleConfig.inverted(ShooterConstants.MIDDLE_SHOOTER_INVERTED);
    middleShooterMotor.configure(middleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig agitatorOneConfig = new SparkMaxConfig();
    agitatorOneConfig.idleMode(IdleMode.kCoast);
    agitatorOneConfig.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    agitatorOneConfig.inverted(ShooterConstants.AGITATOR_MOTOR_ONE_INVERTED);
    agitatorMotorOne.configure(agitatorOneConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig agitatorTwoConfig = new SparkMaxConfig();
    agitatorTwoConfig.idleMode(IdleMode.kCoast);
    agitatorTwoConfig.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    agitatorTwoConfig.inverted(ShooterConstants.AGITATOR_MOTOR_TWO_INVERTED);
    agitatorMotorTwo.configure(agitatorTwoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void stop() {
    shooterIntakeMotor.stopMotor();
    middleShooterMotor.stopMotor();
    agitatorMotorOne.stopMotor();
    agitatorMotorTwo.stopMotor();
    currentShooterPower = 0.0;
  }

  public void setShooterPower(double power) {
    double requestedPower = Double.isFinite(power)
        ? power
        : ShooterConstants.SHOOTER_POWER_NO_TAG_DEFAULT;
    currentShooterPower = MathUtil.clamp(requestedPower, -1.0, 1.0);
    middleShooterMotor.set(currentShooterPower);

    if (Math.abs(currentShooterPower) > 1e-3) {
      setAgitatorPower(ShooterConstants.AGITATOR_POWER);
    } else {
      setAgitatorPower(0.0);
    }
  }

  public void setFlywheelOnlyPower(double power) {
    double requestedPower = Double.isFinite(power)
        ? power
        : ShooterConstants.SHOOTER_POWER_NO_TAG_DEFAULT;
    currentShooterPower = MathUtil.clamp(requestedPower, -1.0, 1.0);
    middleShooterMotor.set(currentShooterPower);
    // Keep feeder/agitator off in always-on flywheel mode.
    shooterIntakeMotor.stopMotor();
    setAgitatorPower(0.0);
  }

  public void setShooterIntakePower(double power) {
    shooterIntakeMotor.set(MathUtil.clamp(power, -1.0, 1.0));
  }

  public void setAgitatorPower(double power) {
    if (!ShooterConstants.AGITATOR_ENABLED) {
      agitatorMotorOne.stopMotor();
      agitatorMotorTwo.stopMotor();
        return;
    }

    double clampedPower = MathUtil.clamp(power, -1.0, 1.0);
    agitatorMotorOne.set(clampedPower);
    agitatorMotorTwo.set(clampedPower);
  }

  public double getShooterPower() {
    return currentShooterPower;
  }

  public boolean atPower() {
    return Math.abs(getShooterPower() - ShooterConstants.SHOOTER_POWER_NO_TAG_DEFAULT)
        <= ShooterConstants.SHOOTER_POWER_TOLERANCE;
  }


  public void setRobotCalibrationModeEnabled(boolean enabled) {
    robotCalibrationModeEnabled = enabled;
  }

  public void setShooterPidConstants(double kp, double ki, double kd, double kf) {
    shooterPidKp = kp;
    shooterPidKi = ki;
    shooterPidKd = kd;
    shooterPidKf = kf;
  }

  public double getShooterPidKp() {
    return shooterPidKp;
  }

  public double getShooterPidKi() {
    return shooterPidKi;
  }

  public double getShooterPidKd() {
    return shooterPidKd;
  }

  public double getShooterPidKf() {
    return shooterPidKf;
  }
  public double getTargetPowerForDistanceInches(double distanceInches) {
    if (robotCalibrationModeEnabled) {
      return MathUtil.clamp(
          shooterCalibrationPowerEntry.getDouble(ShooterConstants.SHOOTER_CALIBRATION_DEFAULT_POWER),
          -1.0,
          1.0);
    }

    if (Double.isNaN(distanceInches) || Double.isInfinite(distanceInches)) {
      return ShooterConstants.SHOOTER_POWER_NO_TAG_DEFAULT;
    }

    double distanceFeet = distanceInches / 12.0;

    double x0 = ShooterConstants.SHOOTER_CAL_POINT_NEAR_DISTANCE_FEET;
    double x1 = ShooterConstants.SHOOTER_CAL_POINT_MID_DISTANCE_FEET;
    double x2 = ShooterConstants.SHOOTER_CAL_POINT_FAR_DISTANCE_FEET;

    double p0 = ShooterConstants.SHOOTER_CAL_POINT_NEAR_POWER;
    double p1 = ShooterConstants.SHOOTER_CAL_POINT_MID_POWER;
    double p2 = ShooterConstants.SHOOTER_CAL_POINT_FAR_POWER;

    if (distanceFeet <= x0) {
      return MathUtil.clamp(p0, -1.0, 1.0);
    }

    if (distanceFeet >= x2) {
      return MathUtil.clamp(p2, -1.0, 1.0);
    }

    // Quadratic (non-linear) interpolation through 3/6/9 ft points.
    double l0 = ((distanceFeet - x1) * (distanceFeet - x2)) / ((x0 - x1) * (x0 - x2));
    double l1 = ((distanceFeet - x0) * (distanceFeet - x2)) / ((x1 - x0) * (x1 - x2));
    double l2 = ((distanceFeet - x0) * (distanceFeet - x1)) / ((x2 - x0) * (x2 - x1));

    double interpolated = p0 * l0 + p1 * l1 + p2 * l2;
    return MathUtil.clamp(interpolated, -1.0, 1.0);
  }

  public Command runAgitatorPower(double power) {
    return startEnd(() -> setAgitatorPower(power), () -> setAgitatorPower(0.0));
  }

  public Command runShooterPower(double shooterPower) {
    return runShooterPower(() -> shooterPower, () -> ShooterConstants.SHOOTER_INTAKE_POWER);
  }

  public Command runShooterPower(DoubleSupplier shooterPowerSupplier) {
    return runShooterPower(shooterPowerSupplier, () -> ShooterConstants.SHOOTER_INTAKE_POWER);
  }

  public Command runShooterPower(DoubleSupplier shooterPowerSupplier, DoubleSupplier intakePowerSupplier) {
    final double[] startTimestamp = {-1.0};
    return runEnd(
        () -> {
          if (startTimestamp[0] < 0.0) {
            startTimestamp[0] = Timer.getFPGATimestamp();
          }

          setShooterPower(shooterPowerSupplier.getAsDouble());

          if (Timer.getFPGATimestamp() - startTimestamp[0]
              >= ShooterConstants.SHOOTER_INTAKE_START_DELAY_SECONDS) {
            setShooterIntakePower(intakePowerSupplier.getAsDouble());
          } else {
            shooterIntakeMotor.stopMotor();
          }
        },
        () -> {
          startTimestamp[0] = -1.0;
          stop();
        });
  }

  public Command runShooterPower() {
    return runShooterPower(ShooterConstants.SHOOTER_POWER_NO_TAG_DEFAULT);
  }

  public Command spinUpForDistanceCommand(DoubleSupplier distanceInchesSupplier) {
    return runOnce(() -> setShooterPower(getTargetPowerForDistanceInches(distanceInchesSupplier.getAsDouble())));
  }

  public Command runShooterForSeconds(double seconds, DoubleSupplier distanceInchesSupplier) {
    return runShooterPower(() -> getTargetPowerForDistanceInches(distanceInchesSupplier.getAsDouble()))
        .withTimeout(seconds)
        .andThen(runOnce(this::stop));
  }

  public Command stopShooterCommand() {
    return runOnce(this::stop);
  }

  @Override
  public void close() {
    shooterIntakeMotor.close();
    middleShooterMotor.close();
    agitatorMotorOne.close();
    agitatorMotorTwo.close();
  }
}
