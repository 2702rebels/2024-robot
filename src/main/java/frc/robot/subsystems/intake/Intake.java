// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  IntakeIO subsystemIO;
  IntakeIOInputsAutoLogged subsystemInputs = new IntakeIOInputsAutoLogged();

  private boolean intaking = false;

  LoggedTunableNumber intakeSpeed = new LoggedTunableNumber("Intake/IntakeSpeed", 0.7);

  /** Creates a new Intake. */
  public Intake(IntakeIO intake) {
    subsystemIO = intake;
  }

  @Override
  public void periodic() {
    subsystemIO.updateInputs(subsystemInputs);
    Logger.processInputs("Intake/Motor", subsystemInputs);
  }

  public void start() {
    subsystemIO.setSpeed(intakeSpeed.get());
    intaking = true;
  }

  public void stop() {
    subsystemIO.setSpeed(0);
    intaking = false;
  }

  public void reverse() {
    subsystemIO.setSpeed(-intakeSpeed.get());
  }

  public boolean isIntaking() {
    return intaking;
  }

  public boolean isBeamTripped() {
    return subsystemInputs.beamBreak;
  }

  /** Runs the intake inwards while the command is scheduled. */
  public Command intake() {
    return new StartEndCommand(
        () -> {
          start();
        },
        () -> {
          stop();
        },
        this);
  }

  /** Runs the intake outwards while the command is scheduled. */
  public Command outake() {
    return new StartEndCommand(
        () -> {
          reverse();
        },
        () -> {
          stop();
        },
        this);
  }
}
