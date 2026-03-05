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
import frc.robot.Constants.ClimbSetupConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
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
  private final Map<String, Command> autoOptions = new LinkedHashMap<>();
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
  "swerve/neo"));
  private final Shooter shooter = new Shooter();
  private final IntakePivot intakePivot = new IntakePivot();
  private final Climber climber = OperatorConstants.CLIMBER_ENABLED ? new Climber() : null;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverOne = new CommandXboxController(0);
  private final CommandXboxController driverTwo = new CommandXboxController(1);



  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(-driverOne.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), // <<<===== CHANGED from -
      () -> MathUtil.applyDeadband(-driverOne.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), // <<<===== CHANGED from -
      () -> -driverOne.getRightX(), // <<<===== CHANGED from -
      () -> -driverOne.getRightY()); // <<<===== CHANGED from -

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverOne.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverOne.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
      () -> driverOne.getRightX() * -1);

  Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
      () -> MathUtil.applyDeadband(driverOne.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverOne.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driverOne.getRawAxis(2));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //startUSBCamera();  // Enable USB Camera for dashboard

    // Configure the trigger bindings
    configureBindings();

    NetworkTable elasticTable = NetworkTableInstance.getDefault().getTable("Elastic");
    autoSelectedEntry = elasticTable.getEntry(AUTO_SELECTED_KEY);
    autoOptionsEntry = elasticTable.getEntry(AUTO_OPTIONS_KEY);

    NamedCommands.registerCommand("runAlignToTag", drivebase.aimAtLimelightTarget(VisionConstants.LIMELIGHT_NAME));
    NamedCommands.registerCommand("runIntakePivotOut", intakePivot.moveToOutAngleCommand());
    NamedCommands.registerCommand("runIntakePivotIn", intakePivot.moveToInAngleCommand());
    NamedCommands.registerCommand("runIntakeDown", intakePivot.moveToOutAngleCommand());
    NamedCommands.registerCommand("runIntakeUp", intakePivot.moveToInAngleCommand());
    // Intentionally no subsystem requirements here so PathPlanner can parallel this with pivot movement.
    NamedCommands.registerCommand("runIntakeWheelsOn", Commands.startEnd(
        () -> intakePivot.setWheelPower(IntakeConstants.WHEEL_POWER),
        intakePivot::stopWheels).withTimeout(5.0));
    // "runShooterOn" now mimics driver behavior: auto-aim while shooting.
    NamedCommands.registerCommand("runShooterOn", createAutoAimAndShootCommand(5.0));
    NamedCommands.registerCommand("runShooterOff", shooter.stopShooterCommand());
    NamedCommands.registerCommand("runShooterFor1Sec", createAutoAimAndShootCommand(1.0));
    NamedCommands.registerCommand("runShooterFor2Sec", createAutoAimAndShootCommand(2.0));
    NamedCommands.registerCommand("runShooterFor3Sec", createAutoAimAndShootCommand(3.0));
    NamedCommands.registerCommand("runAimAndShootFor3Sec", createAutoAimAndShootCommand(3.0));
    NamedCommands.registerCommand("runShooterFor4Sec", createAutoAimAndShootCommand(4.0));
    NamedCommands.registerCommand("runAimAndShootFor4Sec", createAutoAimAndShootCommand(4.0));
    NamedCommands.registerCommand("runShooterFor5Sec", createAutoAimAndShootCommand(5.0));
    NamedCommands.registerCommand("runAimAndShootFor5Sec", createAutoAimAndShootCommand(5.0));
    NamedCommands.registerCommand("runShooterFor6Sec", createAutoAimAndShootCommand(6.0));
    NamedCommands.registerCommand("runAimAndShootFor6Sec", createAutoAimAndShootCommand(6.0));
    NamedCommands.registerCommand("runShooterFor7Sec", createAutoAimAndShootCommand(7.0));
    NamedCommands.registerCommand("runAimAndShootFor7Sec", createAutoAimAndShootCommand(7.0));
    NamedCommands.registerCommand("runPause5", Commands.waitSeconds(5.0));
    NamedCommands.registerCommand("runDriveToClimbSetupLeft", createAutoDriveToClimbSetupCommand(
        ClimbSetupConstants.LEFT_TARGET_POSE,
        ClimbSetupConstants.LEFT_TARGET_TX_DEGREES,
        ClimbSetupConstants.LEFT_TARGET_TY_DEGREES,
        ClimbSetupConstants.LEFT_TARGET_TA_PERCENT));
    NamedCommands.registerCommand("runDriveToClimbSetupRight", createAutoDriveToClimbSetupCommand(
        ClimbSetupConstants.RIGHT_TARGET_POSE,
        ClimbSetupConstants.RIGHT_TARGET_TX_DEGREES,
        ClimbSetupConstants.RIGHT_TARGET_TY_DEGREES,
        ClimbSetupConstants.RIGHT_TARGET_TA_PERCENT));
    NamedCommands.registerCommand("runDriveToClimbSetup", createAutoDriveToClimbSetupCommand(
        ClimbSetupConstants.LEFT_TARGET_POSE,
        ClimbSetupConstants.LEFT_TARGET_TX_DEGREES,
        ClimbSetupConstants.LEFT_TARGET_TY_DEGREES,
        ClimbSetupConstants.LEFT_TARGET_TA_PERCENT));
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

    CommandXboxController intakeController = OperatorConstants.TWO_CONTROLLER_MODE ? driverTwo : driverOne;

    // Intake wheel controls can be assigned to driver one or two via OperatorConstants.TWO_CONTROLLER_MODE.
    intakeController.rightTrigger(0.5)
        .onTrue(Commands.runOnce(() -> intakePivot.setWheelPower(IntakeConstants.WHEEL_POWER)))
        .onFalse(Commands.runOnce(intakePivot::stopWheels));

    driverOne.leftTrigger(0.5)
             .whileTrue(drivebase.driveFieldOrientedWithLimelight(
                 () -> MathUtil.applyDeadband(-driverOne.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                 () -> MathUtil.applyDeadband(-driverOne.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                 VisionConstants.LIMELIGHT_NAME));

    driverOne.leftTrigger(0.5).and(driverOne.rightTrigger(0.5)).whileTrue(
        Commands.parallel(
            shooter.runShooterPower(
                () -> shooter.getTargetPowerForDistanceInches(
                    drivebase.getLimelightTargetDistanceInches(VisionConstants.LIMELIGHT_NAME))),
            intakePivot.runWheelsPower(IntakeConstants.WHEEL_POWER)));

    // Driver one alternate shot mode: fixed shooter power on right bumper (no left trigger required).
    driverOne.rightBumper().whileTrue(
        Commands.parallel(
            shooter.runShooterPower(() -> ShooterConstants.SHOOTER_FIXED_POWER_DRIVER),
            intakePivot.runWheelsPower(IntakeConstants.WHEEL_POWER)));

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

    intakePivot.setDefaultCommand(intakePivot.run(intakePivot::stop));

    // Pivot manual control moved from POV to intake controller left stick Y.
    // Forward stick (negative Y) behaves like previous POV up.
    // Backward stick (positive Y) behaves like previous POV down.
    new Trigger(() -> intakeController.getLeftY() < -0.5)
        .whileTrue(intakePivot.runPivotPower(IntakeConstants.PIVOT_POWER));
    new Trigger(() -> intakeController.getLeftY() > 0.5)
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
            driveToClimbPose)
        .andThen(Commands.idle(drivebase));
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
