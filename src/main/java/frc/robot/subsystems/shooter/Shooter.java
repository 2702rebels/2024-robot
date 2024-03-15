// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO subsystemIO;
  private final ShooterIOInputsAutoLogged subsystemInputs = new ShooterIOInputsAutoLogged();

  private final LoggedTunableNumber kp = new LoggedTunableNumber("Shooter/FlywheelPID/P", 0.0002);
  private final LoggedTunableNumber ki = new LoggedTunableNumber("Shooter/FlywheelPID/I", 0.0);
  private final LoggedTunableNumber kd = new LoggedTunableNumber("Shooter/FlywheelPID/D", 0.0);
  private final LoggedTunableNumber kff =
      new LoggedTunableNumber("Shooter/FlywheelPID/FF", 0.000183);
  private final LoggedTunableNumber staticGain =
      new LoggedTunableNumber("Shooter/FlywheelPID/StaticGain", 0.2265);

  private final LoggedTunableNumber atSpeedAllowance =
      new LoggedTunableNumber("Shooter/AtSpeedAllowance", 50);

  private final SysIdRoutine sysId;

  private boolean feeding = false;

  /** Creates a new Shooter Subsystem. */
  public Shooter(ShooterIO shooter) {
    subsystemIO = shooter;
    subsystemIO.configureFlywheelPID(kp.get(), ki.get(), kd.get(), kff.get(), staticGain.get());

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                Units.Seconds.of(15),
                state -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Units.Volts)), null, this));
  }

  @Override
  public void periodic() {
    subsystemIO.updateInputs(subsystemInputs);
    Logger.processInputs("Shooter/Inputs", subsystemInputs);

    final var callerId = hashCode();
    if (kp.hasChanged(callerId)
        || ki.hasChanged(callerId)
        || kd.hasChanged(callerId)
        || kff.hasChanged(callerId)
        || staticGain.hasChanged(callerId)) {
      System.out.println("Flywheel velocity PID parameters have changed. Reconfiguring them.");
      subsystemIO.configureFlywheelPID(kp.get(), ki.get(), kd.get(), kff.get(), staticGain.get());
    }
  }

  /** Runs the open loop at the specified voltage. */
  public void runVolts(double volts) {
    subsystemIO.setVoltage(volts);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public boolean isAtSpeed(double rpm) {
    if (subsystemInputs.flywheelVelocity > rpm - atSpeedAllowance.get()
        && subsystemInputs.flywheelVelocity < rpm + atSpeedAllowance.get()) {
      return true;
    }
    return false;
  }

  public boolean hasGamePiece() {
    return subsystemInputs.beamBreakFront || subsystemInputs.beamBreakBack;
  }

  public boolean isFrontBeamTripped() {
    return subsystemInputs.beamBreakFront;
  }

  public boolean isBackBeamTripped() {
    return subsystemInputs.beamBreakBack;
  }

  public boolean isFeeding() {
    return feeding;
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
            subsystemIO.setFeedPercentOutput(0.7);
          }
        },
        (interupted) -> {
          subsystemIO.stopFlywheel();
          subsystemIO.stopFeed();
        },
        () -> {
          return false;
        },
        this);
  }

  public void spinUp(double speed) {
    subsystemIO.setFlywheelSpeed(speed);
  }

  public void feedOut() {
    subsystemIO.setFeedPercentOutput(0.7);
  }

  public void startFeed() {
    subsystemIO.setFeedPercentOutput(0.3);
    feeding = true;
  }

  public void stopFeed() {
    subsystemIO.stopFeed();
    feeding = false;
  }

  public Command feedIn() {
    return new StartEndCommand(
            () -> {
              subsystemIO.setFeedPercentOutput(0.3);
              feeding = true;
            },
            () -> {
              subsystemIO.stopFeed();
              feeding = false;
            },
            this)
        .until(
            () -> {
              return subsystemInputs.beamBreakFront;
            });
  }

  /** Stops shooter and feed. */
  public void stop() {
    subsystemIO.stopFeed();
    subsystemIO.stopFlywheel();
  }
}
