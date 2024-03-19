package frc.robot;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CommandsFactory;
import frc.robot.commands.CommandsFactory.LedMode;
import frc.robot.commands.DriveTrajectory;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.led.LedConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.vision.AprilTagDetector;
import frc.robot.subsystems.vision.NoteDetector;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TrajectoryLoader;
import java.util.Map;
import java.util.function.Function;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // subsystems
  private final Drive drive;
  private final NoteDetector noteDetector = new NoteDetector();
  private final AprilTagDetector aprilTagDetector = new AprilTagDetector();
  private final Intake intake;
  private final Pivot pivot;
  private final Shooter shooter;
  private final Led led = new Led(LedConstants.LENGTH, LedConstants.PWM_PORT);
  private final Climb climb;

  // factory for common commands
  private final CommandsFactory commands;

  // loaded choreo trajectories
  private final Map<String, ChoreoTrajectory> trajectories;

  // driver controller
  private final CommandXboxController controllerDriver = new CommandXboxController(0);

  // dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  // spotless:off
  private static final LoggedTunableNumber endgameAlert1 = new LoggedTunableNumber("Endgame/Alert 1", 30.0);
  private static final LoggedTunableNumber endgameAlert2 = new LoggedTunableNumber("Endgame/Alert 2", 10.0);

  private static final LoggedTunableNumber shooterSubwooferSpeed = new LoggedTunableNumber("Shooter/Subwoofer Speed", 3000);
  // spotless:on

  // pre-created command instances to be scheduled manually
  // when the robot is in the disabled state
  private Command commandLedLogging;
  private Command commandLedDisabled;
  private Command commandLedTrackingTags;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(CanID.GYRO_CONTROLLER, true),
                (Integer index) ->
                    new ModuleIOSparkMax(
                        DriveConstants.moduleConfigs[index],
                        DriveConstants.moduleAngularOffsets[index],
                        DriveConstants.drivetrainConfig));

        intake = new Intake(new IntakeIOSparkMax());
        pivot = new Pivot(new PivotIOSparkMax());
        shooter = new Shooter(new ShooterIOSparkMax());
        climb = new Climb(new ClimbIOSparkMax());
        break;

      case SIM:
        // sim robot, instantiate physics sim IO implementations
        drive = new Drive(new GyroIO() {}, (Integer index) -> new ModuleIOSim());
        intake = new Intake(new IntakeIOSim());
        pivot = new Pivot(new PivotIOSim());
        shooter = new Shooter(new ShooterIOSim());
        climb = new Climb(new ClimbIOSim());
        break;

      default:
        // replayed robot, disable IO implementations
        drive = new Drive(new GyroIO() {}, (Integer index) -> new ModuleIO() {});
        intake = new Intake(new IntakeIO() {});
        pivot = new Pivot(new PivotIO() {});
        shooter = new Shooter(new ShooterIO() {});
        climb = new Climb(new ClimbIO() {});
        break;
    }

    // load trajectories from the file system
    trajectories = TrajectoryLoader.loadChoreoTrajectories();

    // commands factory (must come before bindings, etc.)
    commands =
        new CommandsFactory(
            drive, led, intake, pivot, shooter, climb, aprilTagDetector, noteDetector);

    commandLedDisabled = commands.led(LedMode.DISABLED);
    commandLedLogging = commands.led(LedMode.LOGGING);
    commandLedTrackingTags = commands.led(LedMode.TRACKING_TAGS);

    configureBindings();
    configureTriggers();
    configureAutoRoutines();
    configureSmartDashboard();
  }

  /** Creates new command that drives specified trajectory with intake running. */
  private Command driveAndIntake(String trajectoryName, double timeoutSeconds) {
    return new DriveTrajectory(trajectories.get(trajectoryName), true, true, drive)
        .alongWith(commands.intake())
        .withTimeout(timeoutSeconds);
  }

  /** Creates new command sequence for a default auto with specific trajectories. */
  private Command defaultAuto(String trajectory1, String trajectory2) {
    return Commands.sequence(
        commands.scoreInSpeaker(),
        driveAndIntake(trajectory1, 2),
        commands.scoreInSpeaker(),
        driveAndIntake(trajectory2, 3));
  }

  /** Sets up our auto chooser and auto objects. */
  private void configureAutoRoutines() {
    autoChooser.addOption("Amp Auto 1", defaultAuto("InitAmpToNote", "AmpToNote1"));
    autoChooser.addOption("Amp Auto 2", defaultAuto("InitAmpToNote", "AmpToNote2"));
    autoChooser.addOption("Center Auto 1", defaultAuto("InitCenterToNote", "CenterToNote1"));
    autoChooser.addOption("Center Auto 2", defaultAuto("InitCenterToNote", "CenterToNote2"));
    autoChooser.addOption("Center Auto 3", defaultAuto("InitCenterToNote", "CenterToNote3"));
    autoChooser.addOption("Center Auto 4", defaultAuto("InitCenterToNote", "CenterToNote4"));
    autoChooser.addOption("Load Auto 5", defaultAuto("InitLoadToNote", "StageToNote5"));

    autoChooser.addOption(
        "Wheel Diameter Characterization", commands.wheelDiameterCharacterization());

    trajectories.forEach(
        (k, v) -> autoChooser.addOption("Path " + k, new DriveTrajectory(v, true, true, drive)));

    autoChooser.addOption("Just Shoot", commands.scoreInSpeaker());
    autoChooser.addOption("Do Nothing", Commands.none());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    drive.setDefaultCommand(
        commands.driveTeleop(
            () -> -controllerDriver.getLeftX(),
            () -> -controllerDriver.getLeftY(),
            () -> -controllerDriver.getRightX()));

    led.setDefaultCommand(commands.led(LedMode.DEFAULT));

    controllerDriver.x().whileTrue(Commands.run(drive::stopWithX));
    controllerDriver
        .start()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drive.setPose(new Pose2d(new Translation2d(), new Rotation2d()));
                      drive.reset();
                    })
                .ignoringDisable(true));

    // orientation locks
    controllerDriver
        .leftBumper()
        .onTrue(
            Commands.either(
                Commands.runOnce(() -> drive.toggleOrientationLock(Math.toRadians(90))),
                Commands.runOnce(() -> drive.toggleOrientationLock(Math.toRadians(-90))),
                () -> DriverStation.getAlliance().get() == Alliance.Blue));

    controllerDriver
        .rightBumper()
        .onTrue(
            Commands.either(
                Commands.runOnce(() -> drive.toggleOrientationLock(Math.toRadians(-45))),
                Commands.runOnce(() -> drive.toggleOrientationLock(Math.toRadians(45))),
                () -> DriverStation.getAlliance().get() == Alliance.Blue));

    // intake
    controllerDriver.leftTrigger().whileTrue(commands.intake());

    // eject
    controllerDriver.y().whileTrue(commands.eject());

    // speaker shot
    controllerDriver.rightTrigger().whileTrue(commands.scoreInSpeaker());

    // subwoofer shot
    controllerDriver
        .b()
        .whileTrue(
            commands
                .moveArmToPosition(50)
                .alongWith(commands.shoot(shooterSubwooferSpeed.get(), pivot::isAtGoal)))
        .onFalse(commands.moveArmToPosition(20, 2));

    // amp shot
    controllerDriver
        .a()
        .onTrue(
            Commands.parallel(
                    commands.moveArmToPosition(107, 5),
                    commands.shoot(900, pivot::isAtGoal).until(() -> !shooter.hasGamePiece()))
                .andThen(commands.moveArmToPosition(20, 3))
                .deadlineWith(commands.led(LedMode.AMP)));

    // move to idle
    controllerDriver
        .povDown()
        .onTrue(
            Commands.parallel(
                commands.moveArmToPosition(20), Commands.runOnce(shooter::stop, shooter)));

    controllerDriver.povLeft().whileTrue(commands.reindex().withTimeout(2));

    // climb commands
    controllerDriver.povUp().onTrue(commands.homeHooks().andThen(commands.raiseHooks()));
    controllerDriver.back().onTrue(commands.retractHooks());
  }

  /** Use this method to put commands on the SmartDashboard. */
  private void configureSmartDashboard() {
    SmartDashboard.putData("Climb/Home", commands.homeHooks());
    SmartDashboard.putData("Climb/Release Ratchet", commands.releaseRatchet());
    SmartDashboard.putData("Climb/Engage Ratchet", commands.engageRatchet());
  }

  /**
   * Use this method to configure haptic feedback to the driver / operator when certain game
   * milestones have been reached, e.g. end game.
   */
  private void configureTriggers() {
    Function<Double, Command> controllerRumbleCommandFactory =
        time ->
            Commands.sequence(
                Commands.runOnce(
                    () -> controllerDriver.getHID().setRumble(RumbleType.kBothRumble, 1.0)),
                Commands.waitSeconds(time),
                Commands.runOnce(
                    () -> controllerDriver.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

    new Trigger(() -> shooter.hasGamePiece())
        .onTrue(commands.led(LedMode.ACQUIRED_NOTE).withTimeout(1.5));

    new Trigger(() -> intake.isIntaking() && shooter.isFrontBeamTripped())
        .onTrue(controllerRumbleCommandFactory.apply(0.3));

    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(controllerRumbleCommandFactory.apply(0.5));

    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            Commands.sequence(
                controllerRumbleCommandFactory.apply(0.2),
                Commands.waitSeconds(0.1),
                controllerRumbleCommandFactory.apply(0.2),
                Commands.waitSeconds(0.1),
                controllerRumbleCommandFactory.apply(0.2)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void onDisable() {
    CommandScheduler.getInstance().schedule(commands.disablePivot());
    CommandScheduler.getInstance().schedule(commandLedDisabled);
  }

  public void setLoggingIndicator(boolean logging) {
    if (logging) {
      CommandScheduler.getInstance().schedule(commandLedLogging);
    } else if (aprilTagDetector.areTagsTracked(7, 4, 2)) {
      CommandScheduler.getInstance().schedule(commandLedTrackingTags);
    } else {
      CommandScheduler.getInstance().schedule(commandLedDisabled);
    }
  }
}
