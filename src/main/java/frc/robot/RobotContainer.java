package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Function;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // subsystems
  private final Drive drive;
  // private final LedSubsystem led = new LedSubsystem(LedConstants.LENGTH, LedConstants.PWM_PORT);

  // driver controller
  private final CommandXboxController controllerDriver = new CommandXboxController(0);

  // dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private static final LoggedTunableNumber endgameAlert1 =
      new LoggedTunableNumber("Endgame/Alert 1", 30.0);

  private static final LoggedTunableNumber endgameAlert2 =
      new LoggedTunableNumber("Endgame/Alert 2", 10.0);

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
        break;

      case SIM:
        // sim robot, instantiate physics sim IO implementations
        drive = new Drive(new GyroIO() {}, (Integer index) -> new ModuleIOSim());
        break;

      default:
        // replayed robot, disable IO implementations
        drive = new Drive(new GyroIO() {}, (Integer index) -> new ModuleIO() {});
        break;
    }

    // set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    configureBindings();
    configureEndgameAlerts();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controllerDriver.getLeftX(),
            () -> -controllerDriver.getLeftY(),
            () -> -controllerDriver.getRightX()));

    controllerDriver.x().whileTrue(Commands.run(drive::stopWithX));
    controllerDriver
        .b()
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
        .onTrue(Commands.runOnce(() -> drive.toggleOrientationLock(Units.degreesToRadians(180))));

    controllerDriver.rightBumper().onTrue(Commands.runOnce(() -> drive.toggleOrientationLock(0)));
  }

  /**
   * Use this method to configure haptic feedback to the driver / operator when certain game
   * milestones have been reached, e.g. end game.
   */
  private void configureEndgameAlerts() {
    Function<Double, Command> controllerRumbleCommandFactory =
        time ->
            Commands.sequence(
                Commands.runOnce(
                    () -> {
                      controllerDriver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                    }),
                Commands.waitSeconds(time),
                Commands.runOnce(
                    () -> {
                      controllerDriver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                    }));

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
}
