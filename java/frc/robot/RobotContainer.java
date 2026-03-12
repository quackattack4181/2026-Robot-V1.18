// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CalibrationConstants;
import frc.robot.Constants.ClimbSetupConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private static final String AUTO_SELECTED_KEY = "Auto Selected";
  private static final String AUTO_OPTIONS_KEY = "Auto Options";
  private final NetworkTableEntry autoSelectedEntry;
  private final NetworkTableEntry autoOptionsEntry;
  private final NetworkTableEntry calibrationModeEnabledEntry;
  private final NetworkTableEntry swerveHeadingKpEntry;
  private final NetworkTableEntry swerveHeadingKiEntry;
  private final NetworkTableEntry swerveHeadingKdEntry;
  private final NetworkTableEntry swerveAimKpEntry;
  private final NetworkTableEntry swerveAimKiEntry;
  private final NetworkTableEntry swerveAimKdEntry;
  private final NetworkTableEntry shooterKpEntry;
  private final NetworkTableEntry shooterKiEntry;
  private final NetworkTableEntry shooterKdEntry;
  private final NetworkTableEntry shooterKfEntry;
  private final NetworkTableEntry teleopTranslationScaleEntry;
  private final NetworkTableEntry teleopStrafeScaleEntry;
  private final NetworkTableEntry teleopRotationScaleEntry;
  private final NetworkTableEntry teleopLeftXDeadbandEntry;
  private final NetworkTableEntry teleopLeftYDeadbandEntry;
  private final NetworkTableEntry teleopRightXDeadbandEntry;
  private final Map<String, Command> autoOptions = new LinkedHashMap<>();
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
  "swerve/neo"));
  private final Shooter shooter = new Shooter();
  private final IntakePivot intakePivot = new IntakePivot();
  private final Climber climber = OperatorConstants.CLIMBER_ENABLED ? new Climber() : null;
  private boolean shooterAlwaysOnEnabled = ShooterConstants.SHOOTER_ALWAYS_ON_ENABLED;
  private boolean intakeWheelsAlwaysOnEnabled = IntakeConstants.INTAKE_WHEELS_ALWAYS_ON_ENABLED;
  private double teleopTranslationScale = CalibrationConstants.TELEOP_TRANSLATION_SCALE;
  private double teleopStrafeScale = CalibrationConstants.TELEOP_STRAFE_SCALE;
  private double teleopRotationScale = CalibrationConstants.TELEOP_ROTATION_SCALE;
  private double teleopLeftXDeadband = CalibrationConstants.TELEOP_LEFT_X_DEADBAND;
  private double teleopLeftYDeadband = CalibrationConstants.TELEOP_LEFT_Y_DEADBAND;
  private double teleopRightXDeadband = CalibrationConstants.TELEOP_RIGHT_X_DEADBAND;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverOne = new CommandXboxController(0);
  private final CommandXboxController driverTwo = new CommandXboxController(1);



  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      () -> teleopTranslationScale * MathUtil.applyDeadband(-driverOne.getLeftY(), teleopLeftYDeadband),
      () -> teleopStrafeScale * MathUtil.applyDeadband(-driverOne.getLeftX(), teleopLeftXDeadband),
      () -> -teleopRotationScale * MathUtil.applyDeadband(driverOne.getRightX(), teleopRightXDeadband),
      () -> -driverOne.getRightY());

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
      () -> teleopTranslationScale * MathUtil.applyDeadband(driverOne.getLeftY() * -1, teleopLeftYDeadband),
      () -> teleopStrafeScale * MathUtil.applyDeadband(driverOne.getLeftX() * -1, teleopLeftXDeadband),
      () -> -teleopRotationScale * MathUtil.applyDeadband(driverOne.getRightX(), teleopRightXDeadband));

  Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
      () -> teleopTranslationScale * MathUtil.applyDeadband(driverOne.getLeftY(), teleopLeftYDeadband),
      () -> teleopStrafeScale * MathUtil.applyDeadband(driverOne.getLeftX(), teleopLeftXDeadband),
      () -> driverOne.getRawAxis(2));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //startUSBCamera();  // Enable USB Camera for dashboard

    // Configure the trigger bindings
    configureBindings();
    SmartDashboard.putBoolean("Shooter Always On Enabled", shooterAlwaysOnEnabled);
    SmartDashboard.putBoolean("Intake Wheels Always On Enabled", intakeWheelsAlwaysOnEnabled);

    NetworkTable elasticTable = NetworkTableInstance.getDefault().getTable("Elastic");
    autoSelectedEntry = elasticTable.getEntry(AUTO_SELECTED_KEY);
    autoOptionsEntry = elasticTable.getEntry(AUTO_OPTIONS_KEY);

    calibrationModeEnabledEntry = elasticTable.getEntry("Robot Calibration Mode Enabled");
    swerveHeadingKpEntry = elasticTable.getEntry("Swerve Heading PID kP");
    swerveHeadingKiEntry = elasticTable.getEntry("Swerve Heading PID kI");
    swerveHeadingKdEntry = elasticTable.getEntry("Swerve Heading PID kD");
    swerveAimKpEntry = elasticTable.getEntry("Swerve Aim PID kP");
    swerveAimKiEntry = elasticTable.getEntry("Swerve Aim PID kI");
    swerveAimKdEntry = elasticTable.getEntry("Swerve Aim PID kD");
    shooterKpEntry = elasticTable.getEntry("Shooter PID kP");
    shooterKiEntry = elasticTable.getEntry("Shooter PID kI");
    shooterKdEntry = elasticTable.getEntry("Shooter PID kD");
    shooterKfEntry = elasticTable.getEntry("Shooter PID kF");
    teleopTranslationScaleEntry = elasticTable.getEntry("Teleop Translation Scale");
    teleopStrafeScaleEntry = elasticTable.getEntry("Teleop Strafe Scale");
    teleopRotationScaleEntry = elasticTable.getEntry("Teleop Rotation Scale");
    teleopLeftXDeadbandEntry = elasticTable.getEntry("Teleop LeftX Deadband");
    teleopLeftYDeadbandEntry = elasticTable.getEntry("Teleop LeftY Deadband");
    teleopRightXDeadbandEntry = elasticTable.getEntry("Teleop RightX Deadband");

    calibrationModeEnabledEntry.setBoolean(CalibrationConstants.ROBOT_CALIBRATION_MODE_ENABLED);
    swerveHeadingKpEntry.setDouble(CalibrationConstants.SWERVE_HEADING_KP);
    swerveHeadingKiEntry.setDouble(CalibrationConstants.SWERVE_HEADING_KI);
    swerveHeadingKdEntry.setDouble(CalibrationConstants.SWERVE_HEADING_KD);
    swerveAimKpEntry.setDouble(CalibrationConstants.SWERVE_AIM_KP);
    swerveAimKiEntry.setDouble(CalibrationConstants.SWERVE_AIM_KI);
    swerveAimKdEntry.setDouble(CalibrationConstants.SWERVE_AIM_KD);
    shooterKpEntry.setDouble(CalibrationConstants.SHOOTER_KP);
    shooterKiEntry.setDouble(CalibrationConstants.SHOOTER_KI);
    shooterKdEntry.setDouble(CalibrationConstants.SHOOTER_KD);
    shooterKfEntry.setDouble(CalibrationConstants.SHOOTER_KF);
    teleopTranslationScaleEntry.setDouble(CalibrationConstants.TELEOP_TRANSLATION_SCALE);
    teleopStrafeScaleEntry.setDouble(CalibrationConstants.TELEOP_STRAFE_SCALE);
    teleopRotationScaleEntry.setDouble(CalibrationConstants.TELEOP_ROTATION_SCALE);
    teleopLeftXDeadbandEntry.setDouble(CalibrationConstants.TELEOP_LEFT_X_DEADBAND);
    teleopLeftYDeadbandEntry.setDouble(CalibrationConstants.TELEOP_LEFT_Y_DEADBAND);
    teleopRightXDeadbandEntry.setDouble(CalibrationConstants.TELEOP_RIGHT_X_DEADBAND);
    updateCalibrationFromDashboard();

    NamedCommands.registerCommand("runAlignToTag", drivebase.aimAtLimelightTarget(VisionConstants.LIMELIGHT_NAME));
    NamedCommands.registerCommand("runRotate0", Commands.defer(
        () -> {
          double deltaDegrees = MathUtil.inputModulus(0.0 - drivebase.getHeading().getDegrees(), -180.0, 180.0);
          return drivebase.rotateByDegreesCommand(deltaDegrees, 2.0);
        },
        Set.of(drivebase)));
    NamedCommands.registerCommand("runRotate180", Commands.defer(
        () -> {
          double deltaDegrees = MathUtil.inputModulus(-180.0 - drivebase.getHeading().getDegrees(), -180.0, 180.0);
          return drivebase.rotateByDegreesCommand(deltaDegrees, 2.0);
        },
        Set.of(drivebase)));
    NamedCommands.registerCommand("runIntakeDown", createAutoIntakeDownCommand());
    NamedCommands.registerCommand("runIntakeUp", createAutoIntakeUpCommand());
    // Simple auto aim+shoot commands: runShoot1 .. runShoot10 (seconds).
    NamedCommands.registerCommand("runShoot1", createAutoAimAndShootCommand(1.0));
    NamedCommands.registerCommand("runShoot2", createAutoAimAndShootCommand(2.0));
    NamedCommands.registerCommand("runShoot3", createAutoAimAndShootCommand(3.0));
    NamedCommands.registerCommand("runShoot4", createAutoAimAndShootCommand(4.0));
    NamedCommands.registerCommand("runShoot5", createAutoAimAndShootCommand(5.0));
    NamedCommands.registerCommand("runShoot6", createAutoAimAndShootCommand(6.0));
    NamedCommands.registerCommand("runShoot7", createAutoAimAndShootCommand(7.0));
    NamedCommands.registerCommand("runShoot8", createAutoAimAndShootCommand(8.0));
    NamedCommands.registerCommand("runShoot9", createAutoAimAndShootCommand(9.0));
    NamedCommands.registerCommand("runShoot10", createAutoAimAndShootCommand(10.0));
    NamedCommands.registerCommand("runPause5", Commands.waitSeconds(5.0));
    NamedCommands.registerCommand("runDriveToClimbSetupLeft", createAutoDriveToClimbSetupCommand(
        ClimbSetupConstants.LEFT_TARGET_POSE,
        ClimbSetupConstants.LEFT_TARGET_TX_DEGREES,
        ClimbSetupConstants.LEFT_TARGET_TY_DEGREES,
        ClimbSetupConstants.LEFT_TARGET_TA_PERCENT).withTimeout(3.0));
    NamedCommands.registerCommand("runDriveToClimbSetupRight", createAutoDriveToClimbSetupCommand(
        ClimbSetupConstants.RIGHT_TARGET_POSE,
        ClimbSetupConstants.RIGHT_TARGET_TX_DEGREES,
        ClimbSetupConstants.RIGHT_TARGET_TY_DEGREES,
        ClimbSetupConstants.RIGHT_TARGET_TA_PERCENT).withTimeout(3.0));
    NamedCommands.registerCommand("runDriveToClimbSetup", createAutoDriveToClimbSetupCommand(
        ClimbSetupConstants.LEFT_TARGET_POSE,
        ClimbSetupConstants.LEFT_TARGET_TX_DEGREES,
        ClimbSetupConstants.LEFT_TARGET_TY_DEGREES,
        ClimbSetupConstants.LEFT_TARGET_TA_PERCENT).withTimeout(3.0));
    if (OperatorConstants.CLIMBER_ENABLED && climber != null) {
      NamedCommands.registerCommand("runClimberDownPosition", climber.moveToDownPositionCommand());
      NamedCommands.registerCommand("runClimberLevel1Position", climber.moveToLevel1PositionCommand());
      NamedCommands.registerCommand("runClimberLevel2Position", climber.moveToLevel2PositionCommand());
    }
    NamedCommands.registerCommand("runBackup6Inches", Commands.defer(
        () -> drivebase.driveBackwardRobotRelativeCommand(
            ClimbSetupConstants.DRIVER_BACKUP_SPEED_MPS,
            ClimbSetupConstants.DRIVER_BACKUP_DURATION_SECONDS),
        Set.of(drivebase)));

    // Auto-discover PathPlanner autos/paths from deploy and publish to Elastic.
    loadAutoOptions();
    
  }

  // USB Camera and it's settings
//  private void startUSBCamera() {
//        UsbCamera camera = CameraServer.startAutomaticCapture(0);
//        camera.setResolution(320, 240); // Adjust resolution if needed
//        camera.setFPS(25); // Adjust FPS for efficiency
//    }

  private Command runIntakeWheelsWithoutRequirements(double power) {
    return Commands.startEnd(
        () -> intakePivot.setWheelPower(power),
        intakePivot::stopWheels);
  }



  private Command createAutoIntakeDownCommand() {
    final double targetAngle = IntakeConstants.PIVOT_MAX_OUTWARD_ANGLE;
    return intakePivot.runEnd(
        () -> {
          double currentAngle = intakePivot.getPivotAngleDegrees();
          double error = targetAngle - currentAngle;
          if (Math.abs(error) <= IntakeConstants.PIVOT_ANGLE_TOLERANCE_DEGREES) {
            intakePivot.stop();
          } else {
            intakePivot.setPivotPower(Math.signum(error) * Math.abs(IntakeConstants.PIVOT_POWER));
          }

          if (currentAngle >= IntakeConstants.INTAKE_WHEELS_ALWAYS_ON_MIN_PIVOT_ANGLE_DEGREES) {
            intakePivot.setWheelPower(IntakeConstants.INTAKE_WHEELS_ALWAYS_ON_POWER);
          } else {
            intakePivot.stopWheels();
          }
        },
        () -> {
          intakePivot.stop();
          if (intakePivot.getPivotAngleDegrees() >= IntakeConstants.INTAKE_WHEELS_ALWAYS_ON_MIN_PIVOT_ANGLE_DEGREES) {
            intakePivot.setWheelPower(IntakeConstants.INTAKE_WHEELS_ALWAYS_ON_POWER);
          } else {
            intakePivot.stopWheels();
          }
        })
        .until(() -> intakePivot.isNearAngle(targetAngle))
        .withTimeout(2.5);
  }

  private Command createAutoIntakeUpCommand() {
    final double targetAngle = IntakeConstants.PIVOT_MAX_INWARD_ANGLE;
    return intakePivot.runEnd(
        () -> {
          double currentAngle = intakePivot.getPivotAngleDegrees();
          double error = targetAngle - currentAngle;
          if (Math.abs(error) <= IntakeConstants.PIVOT_ANGLE_TOLERANCE_DEGREES) {
            intakePivot.stop();
          } else {
            intakePivot.setPivotPower(Math.signum(error) * Math.abs(IntakeConstants.PIVOT_POWER));
          }

          if (currentAngle >= IntakeConstants.INTAKE_WHEELS_ALWAYS_ON_MIN_PIVOT_ANGLE_DEGREES) {
            intakePivot.setWheelPower(IntakeConstants.INTAKE_WHEELS_ALWAYS_ON_POWER);
          } else {
            intakePivot.stopWheels();
          }
        },
        () -> {
          intakePivot.stop();
          if (intakePivot.getPivotAngleDegrees() < IntakeConstants.INTAKE_WHEELS_ALWAYS_ON_MIN_PIVOT_ANGLE_DEGREES) {
            intakePivot.stopWheels();
          }
        })
        .until(() -> intakePivot.isNearAngle(targetAngle))
        .withTimeout(2.5);
  }

  private void setShooterAlwaysOnEnabled(boolean enabled) {
    shooterAlwaysOnEnabled = enabled;
    SmartDashboard.putBoolean("Shooter Always On Enabled", shooterAlwaysOnEnabled);
    SmartDashboard.putBoolean("Intake Wheels Always On Enabled", intakeWheelsAlwaysOnEnabled);
    if (!shooterAlwaysOnEnabled) {
      shooter.stop();
    }
  }

  private void setIntakeWheelsAlwaysOnEnabled(boolean enabled) {
    intakeWheelsAlwaysOnEnabled = enabled;
    SmartDashboard.putBoolean("Intake Wheels Always On Enabled", intakeWheelsAlwaysOnEnabled);
    if (!intakeWheelsAlwaysOnEnabled) {
      intakePivot.stopWheels();
    }
  }

  private void toggleAlwaysOnModes() {
    boolean nextEnabled = !shooterAlwaysOnEnabled;
    setShooterAlwaysOnEnabled(nextEnabled);
    setIntakeWheelsAlwaysOnEnabled(nextEnabled);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

  private void configureBindings() {

    //========================================
    //        Driver One Controls #1
    //========================================

    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);

    // Keep driver-one trigger semantics explicit:
    // LT = aim only, RT = distance shot, RB = fixed-power shot.
    final double triggerThreshold = 0.2;

    // Driver two keeps normal direct intake-wheel control; driver one shooting also spins wheels.
    driverTwo.rightTrigger(triggerThreshold)
        .onTrue(Commands.runOnce(() -> intakePivot.setWheelPower(IntakeConstants.WHEEL_POWER)))
        .onFalse(Commands.runOnce(intakePivot::stopWheels));

    driverOne.leftTrigger(triggerThreshold)
             .whileTrue(drivebase.driveFieldOrientedWithLimelight(
                 () -> teleopTranslationScale * MathUtil.applyDeadband(-driverOne.getLeftY(), teleopLeftYDeadband),
                 () -> teleopStrafeScale * MathUtil.applyDeadband(-driverOne.getLeftX(), teleopLeftXDeadband),
                 VisionConstants.LIMELIGHT_NAME));

    driverOne.rightTrigger(triggerThreshold).whileTrue(
        Commands.parallel(
            shooter.runShooterPower(
                () -> shooter.getTargetPowerForDistanceInches(
                    drivebase.getLimelightTargetDistanceInches(VisionConstants.LIMELIGHT_NAME))),
            runIntakeWheelsWithoutRequirements(IntakeConstants.WHEEL_POWER)));

    // Driver one alternate shot mode: fixed shooter power on right bumper (no left trigger required).
    driverOne.rightBumper().whileTrue(
        Commands.parallel(
            shooter.runShooterPower(ShooterConstants.SHOOTER_FIXED_POWER_DRIVER),
            runIntakeWheelsWithoutRequirements(IntakeConstants.WHEEL_POWER)));


    // Driver one climb lineup test: hold X to center on climb tag and hold ~9ft distance.
    driverOne.x().whileTrue(drivebase.lineUpToTagAtDistance(
        VisionConstants.LIMELIGHT_NAME,
        VisionConstants.CLIMBER_APPROVED_TAG_IDS,
        ClimbSetupConstants.TAG_LINEUP_X_TARGET_DISTANCE_FEET,
        ClimbSetupConstants.TAG_LINEUP_X_LATERAL_OFFSET_INCHES));
    driverOne.b().whileTrue(drivebase.lineUpToTagAtDistance(
        VisionConstants.LIMELIGHT_NAME,
        VisionConstants.CLIMBER_APPROVED_TAG_IDS,
        ClimbSetupConstants.TAG_LINEUP_X_TARGET_DISTANCE_FEET,
        ClimbSetupConstants.TAG_LINEUP_B_LATERAL_OFFSET_INCHES));
    driverOne.y().onTrue(Commands.defer(
        () -> drivebase.driveBackwardRobotRelativeCommand(
            ClimbSetupConstants.DRIVER_BACKUP_SPEED_MPS,
            ClimbSetupConstants.DRIVER_BACKUP_DURATION_SECONDS),
        Set.of(drivebase)));

    // Driver one manual gyro zero: current facing becomes forward.
    driverOne.start().onTrue(Commands.runOnce(drivebase::zeroGyro));

    // Press left bumper to toggle always-on flywheel mode on/off.
    // This can disable always-on even when the constant default is true.
    new Trigger(driverOne.getHID()::getLeftBumperButton)
        .onTrue(Commands.runOnce(this::toggleAlwaysOnModes));

    shooter.setDefaultCommand(shooter.run(() -> {
      if (shooterAlwaysOnEnabled) {
        shooter.setFlywheelOnlyPower(ShooterConstants.SHOOTER_ALWAYS_ON_DEFAULT_POWER);
      } else {
        shooter.stop();
      }
    }));

    intakePivot.setDefaultCommand(intakePivot.run(() -> {
      intakePivot.stop();
      if (intakeWheelsAlwaysOnEnabled
          && intakePivot.getPivotAngleDegrees() >= IntakeConstants.INTAKE_WHEELS_ALWAYS_ON_MIN_PIVOT_ANGLE_DEGREES) {
        intakePivot.setWheelPower(IntakeConstants.INTAKE_WHEELS_ALWAYS_ON_POWER);
      } else {
        intakePivot.stopWheels();
      }
    }));

    // Pivot manual control moved from POV to intake controller left stick Y.
    // Forward stick (negative Y) behaves like previous POV up.
    // Backward stick (positive Y) behaves like previous POV down.
    new Trigger(() -> driverTwo.getLeftY() < -0.5)
        .whileTrue(intakePivot.runPivotPower(IntakeConstants.PIVOT_POWER));
    new Trigger(() -> driverTwo.getLeftY() > 0.5)
        .whileTrue(intakePivot.runPivotPower(-IntakeConstants.PIVOT_POWER));

    // Driver two pivot agitation: hold Back to spin intake wheels and oscillate pivot +/-30 degrees.
    driverTwo.back().whileTrue(intakePivot.runPivotAgitation(30.0, IntakeConstants.WHEEL_POWER));
    driverTwo.start().whileTrue(intakePivot.holdAtAngleCommand(IntakeConstants.PIVOT_MAX_OUTWARD_ANGLE));
    driverTwo.a().whileTrue(intakePivot.holdAtAngleCommand(IntakeConstants.PIVOT_MAX_INWARD_ANGLE));

    if (OperatorConstants.CLIMBER_ENABLED && climber != null) {
      // Climber controls on driver two: bumpers for manual, B/X/Y for preset positions.
      driverTwo.leftBumper().whileTrue(climber.runClimberPower(ClimberConstants.CLIMBER_POWER));
      driverTwo.rightBumper().whileTrue(climber.runClimberPower(-ClimberConstants.CLIMBER_POWER));
      driverTwo.b().onTrue(climber.moveToDownPositionCommand());
      driverTwo.x().onTrue(climber.moveToLevel1PositionCommand());
      driverTwo.y().onTrue(climber.moveToLevel2PositionCommand());
    }


    //========================================
    //        Driver Two Controls #2
    //========================================

  }

  public void zeroDriverHeading() {
    drivebase.zeroGyro();
  }

  private Command createAutoDriveToClimbSetupCommand(Pose2d requestedPose,
                                                      double snapshotTx,
                                                      double snapshotTy,
                                                      double snapshotTa) {
    Command rotateForward = Commands.defer(
        () -> {
          double currentHeading = drivebase.getHeading().getDegrees();
          double deltaDegrees = MathUtil.inputModulus(
              ClimbSetupConstants.TARGET_HEADING_DEGREES - currentHeading,
              -180.0,
              180.0);
          return drivebase.rotateByDegreesCommand(deltaDegrees, ClimbSetupConstants.TAG_ALIGN_TIMEOUT_SECONDS);
        },
        java.util.Set.of(drivebase));

    Command waitForClimbTag = Commands.waitUntil(
        () -> drivebase.hasAnyLimelightTargetFromList(
            VisionConstants.LIMELIGHT_NAME,
            VisionConstants.CLIMBER_APPROVED_TAG_IDS))
        .withTimeout(ClimbSetupConstants.TAG_ACQUIRE_TIMEOUT_SECONDS);

    Command driveToClimbPose = Commands.defer(
        () -> {
          Pose2d climbPose = new Pose2d(
              requestedPose.getX(),
              requestedPose.getY(),
              Rotation2d.fromDegrees(ClimbSetupConstants.TARGET_HEADING_DEGREES));

          double positionToleranceMeters = edu.wpi.first.math.util.Units
              .inchesToMeters(ClimbSetupConstants.POSITION_TOLERANCE_INCHES);

          return drivebase.driveToPose(climbPose)
              .until(() -> {
                boolean nearPose = drivebase.isNearPose(
                    climbPose,
                    positionToleranceMeters,
                    ClimbSetupConstants.HEADING_TOLERANCE_DEGREES);

                boolean nearSnapshot = drivebase.isNearLimelightSnapshot(
                    VisionConstants.LIMELIGHT_NAME,
                    VisionConstants.CLIMBER_APPROVED_TAG_IDS,
                    snapshotTx,
                    snapshotTy,
                    snapshotTa,
                    ClimbSetupConstants.LIMELIGHT_TX_TOLERANCE_DEGREES,
                    ClimbSetupConstants.LIMELIGHT_TY_TOLERANCE_DEGREES,
                    ClimbSetupConstants.LIMELIGHT_TA_TOLERANCE_PERCENT);

                return nearPose || nearSnapshot;
              })
              .withTimeout(ClimbSetupConstants.APPROACH_TIMEOUT_SECONDS);
        },
        java.util.Set.of(drivebase));

    return Commands.sequence(
            waitForClimbTag,
            rotateForward,
            driveToClimbPose);
  }

  private Command createAutoAimAndShootCommand(double seconds) {
    Command autoAim = drivebase.driveFieldOrientedWithLimelight(
        () -> 0.0,
        () -> 0.0,
        VisionConstants.LIMELIGHT_NAME);
    Command autoShoot = shooter.runShooterForSeconds(
        seconds,
        () -> drivebase.getLimelightTargetDistanceInches(VisionConstants.LIMELIGHT_NAME));
    Command autoIntakeWheels = intakePivot.runWheelsPower(IntakeConstants.WHEEL_POWER);

    return Commands.deadline(autoShoot, autoAim, autoIntakeWheels);
  }

  private void loadAutoOptions() {
    autoOptions.clear();

    Path pathplannerDir = Filesystem.getDeployDirectory().toPath().resolve("pathplanner");
    Path autosDir = pathplannerDir.resolve("autos");

    if (Files.isDirectory(autosDir)) {
      try (var files = Files.list(autosDir).filter(path -> path.toString().endsWith(".auto"))
          .sorted(Comparator.comparing(path -> path.getFileName().toString()))) {
        files.forEach(path -> {
          String fileName = path.getFileName().toString();
          String autoName = fileName.substring(0, fileName.length() - 5);
          autoOptions.put(autoName, AutoBuilder.buildAuto(autoName));
        });
      } catch (IOException e) {
        System.err.println("Failed to read PathPlanner autos: " + e.getMessage());
      }
    }

    if (Files.isDirectory(pathplannerDir)) {
      try (var files = Files.list(pathplannerDir).filter(path -> path.toString().endsWith(".path"))
          .sorted(Comparator.comparing(path -> path.getFileName().toString()))) {
        files.forEach(path -> {
          String fileName = path.getFileName().toString();
          String pathName = fileName.substring(0, fileName.length() - 5);
          String optionName = "Path: " + pathName;
          try {
            autoOptions.putIfAbsent(
                optionName,
                AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName)));
          } catch (Exception e) {
            System.err.println("Failed to load PathPlanner path '" + pathName + "': " + e.getMessage());
          }
        });
      } catch (IOException e) {
        System.err.println("Failed to read PathPlanner paths: " + e.getMessage());
      }
    }

    String[] optionNames = autoOptions.keySet().toArray(new String[0]);
    autoOptionsEntry.setStringArray(optionNames);
    if (autoOptions.isEmpty()) {
      autoSelectedEntry.setString("");
      SmartDashboard.putData("Auto Chooser", autoChooser);
      return;
    }

    String defaultSelection = optionNames[0];
    autoChooser.setDefaultOption(defaultSelection, defaultSelection);
    for (String option : optionNames) {
      if (!option.equals(defaultSelection)) {
        autoChooser.addOption(option, option);
      }
    }
    SmartDashboard.putData("Auto Chooser", autoChooser);
    autoSelectedEntry.setString(defaultSelection);
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    if (autoOptions.isEmpty()) {
      return null;
    }

    String fallbackAuto = autoOptions.keySet().iterator().next();

    // Elastic single-source selection by auto name.
    String selectedByName = autoSelectedEntry.getString(fallbackAuto);
    if (selectedByName != null && autoOptions.containsKey(selectedByName)) {
      return autoOptions.get(selectedByName);
    }

    // Optional local chooser fallback if Elastic selection is invalid.
    String chooserSelection = autoChooser.getSelected();
    if (chooserSelection != null && autoOptions.containsKey(chooserSelection)) {
      autoSelectedEntry.setString(chooserSelection);
      return autoOptions.get(chooserSelection);
    }

    autoSelectedEntry.setString(fallbackAuto);
    return autoOptions.get(fallbackAuto);
  }


  public void updateCalibrationFromDashboard() {
    boolean calibrationEnabled = calibrationModeEnabledEntry.getBoolean(
        CalibrationConstants.ROBOT_CALIBRATION_MODE_ENABLED);

    shooter.setRobotCalibrationModeEnabled(calibrationEnabled);

    teleopTranslationScale = MathUtil.clamp(
        teleopTranslationScaleEntry.getDouble(CalibrationConstants.TELEOP_TRANSLATION_SCALE),
        0.0,
        1.5);
    teleopStrafeScale = MathUtil.clamp(
        teleopStrafeScaleEntry.getDouble(CalibrationConstants.TELEOP_STRAFE_SCALE),
        0.0,
        1.5);
    teleopRotationScale = MathUtil.clamp(
        teleopRotationScaleEntry.getDouble(CalibrationConstants.TELEOP_ROTATION_SCALE),
        0.0,
        1.5);
    teleopLeftXDeadband = MathUtil.clamp(
        teleopLeftXDeadbandEntry.getDouble(CalibrationConstants.TELEOP_LEFT_X_DEADBAND),
        0.0,
        0.5);
    teleopLeftYDeadband = MathUtil.clamp(
        teleopLeftYDeadbandEntry.getDouble(CalibrationConstants.TELEOP_LEFT_Y_DEADBAND),
        0.0,
        0.5);
    teleopRightXDeadband = MathUtil.clamp(
        teleopRightXDeadbandEntry.getDouble(CalibrationConstants.TELEOP_RIGHT_X_DEADBAND),
        0.0,
        0.5);

    if (!calibrationEnabled) {
      return;
    }

    drivebase.setHeadingPid(
        swerveHeadingKpEntry.getDouble(CalibrationConstants.SWERVE_HEADING_KP),
        swerveHeadingKiEntry.getDouble(CalibrationConstants.SWERVE_HEADING_KI),
        swerveHeadingKdEntry.getDouble(CalibrationConstants.SWERVE_HEADING_KD));

    drivebase.setLimelightAimPid(
        swerveAimKpEntry.getDouble(CalibrationConstants.SWERVE_AIM_KP),
        swerveAimKiEntry.getDouble(CalibrationConstants.SWERVE_AIM_KI),
        swerveAimKdEntry.getDouble(CalibrationConstants.SWERVE_AIM_KD));

    shooter.setShooterPidConstants(
        shooterKpEntry.getDouble(CalibrationConstants.SHOOTER_KP),
        shooterKiEntry.getDouble(CalibrationConstants.SHOOTER_KI),
        shooterKdEntry.getDouble(CalibrationConstants.SHOOTER_KD),
        shooterKfEntry.getDouble(CalibrationConstants.SHOOTER_KF));
  }


  public Command getAutoFinishSpinCommand() {
    return drivebase.rotateByDegreesCommand(180.0, OperatorConstants.AUTO_FINISH_SPIN_TIMEOUT_SECONDS);
  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

}
