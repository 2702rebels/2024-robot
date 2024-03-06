// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO subsystemIO;
  private final ShooterIOInputsAutoLogged subsystemInputs = new ShooterIOInputsAutoLogged();

  private final LoggedTunableNumber kp = new LoggedTunableNumber("Shooter/FlywheelPID/P", 0.0);
  private final LoggedTunableNumber ki = new LoggedTunableNumber("Shooter/FlywheelPID/I", 0.0);
  private final LoggedTunableNumber kd = new LoggedTunableNumber("Shooter/FlywheelPID/D", 0.0);

  private final LoggedTunableNumber kp_feed = new LoggedTunableNumber("Shooter/FeedPID/P", 0.0);
  private final LoggedTunableNumber ki_feed = new LoggedTunableNumber("Shooter/FeedPID/I", 0.0);
  private final LoggedTunableNumber kd_feed = new LoggedTunableNumber("Shooter/FeedPID/D", 0.0);

  private final LoggedTunableNumber atSpeedAllowance =
      new LoggedTunableNumber("Shooter/AtSpeedAllowance", 50);
  private final LoggedTunableNumber feedRPM = new LoggedTunableNumber("Shooter/FeedRPM", 3000);

  /** Creates a new Shooter Subsystem. */
  public Shooter(ShooterIO shooter) {
    subsystemIO = shooter;
    subsystemIO.configureFlywheelPID(kp.get(), ki.get(), kd.get());
    subsystemIO.configureFeedPID(kp_feed.get(), ki_feed.get(), kd_feed.get());
  }

  @Override
  public void periodic() {
    subsystemIO.updateInputs(subsystemInputs);
    Logger.processInputs("Shooter/Inputs", subsystemInputs);

    final var callerId = hashCode();
    if (kp.hasChanged(callerId) || ki.hasChanged(callerId) || kd.hasChanged(callerId)) {
      System.out.println("Flywheel velocity PID parameters have changed. Reconfiguring them.");
      subsystemIO.configureFlywheelPID(kp.get(), ki.get(), kd.get());
    }

    if (kp_feed.hasChanged(callerId)
        || ki_feed.hasChanged(callerId)
        || kd_feed.hasChanged(callerId)) {
      System.out.println("Feed velocity PID parameters have changed. Reconfiguring them.");
      subsystemIO.configureFeedPID(kp_feed.get(), ki_feed.get(), kd_feed.get());
    }
  }

  public boolean isAtSpeed(double rpm) {
    if (subsystemInputs.flywheelVelocity > rpm - atSpeedAllowance.get()
        && subsystemInputs.flywheelVelocity < rpm + atSpeedAllowance.get()) {
      return true;
    }
    return false;
  }

  /**
   * Spin up the flywheel and will feed the game piece into the flywheel when it's both at speed and
   * the addtional okToShoot boolean supplier resolves to true.
   */
  public Command shoot(double rpm, BooleanSupplier okToShoot) {
    return new FunctionalCommand(
        () -> {
          subsystemIO.setFlywheelSpeed(rpm);
        },
        () -> {
          if (isAtSpeed(rpm) && okToShoot.getAsBoolean()) {
            subsystemIO.setFeedSpeed(feedRPM.get());
          }
        },
        (interupted) -> {
          subsystemIO.stopFlywheel();
          subsystemIO.stopFeed();
        },
        null,
        this);
  }

  public Command feed() {
    return new StartEndCommand(
            () -> {
              subsystemIO.setFeedSpeed(feedRPM.get());
            },
            () -> {
              subsystemIO.stopFeed();
            },
            this)
        .until(
            () -> {
              return subsystemInputs.feedBeamBreak;
            });
  }
}
