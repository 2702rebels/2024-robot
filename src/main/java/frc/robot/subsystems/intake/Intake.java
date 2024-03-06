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

  LoggedTunableNumber intakeSpeed = new LoggedTunableNumber("Intake/IntakeSpeed", 0.0);

  /** Creates a new Intake. */
  public Intake(IntakeIO intake) {
    subsystemIO = intake;
  }

  @Override
  public void periodic() {
    subsystemIO.updateInputs(subsystemInputs);
    Logger.processInputs("Intake/Motor", subsystemInputs);
  }

  public Command intake() {
    return new StartEndCommand(
        () -> {
          subsystemIO.setSpeed(intakeSpeed.get());
        },
        () -> {
          subsystemIO.setSpeed(0);
        },
        this);
  }

  public Command outake() {
    return new StartEndCommand(
        () -> {
          subsystemIO.setSpeed(-intakeSpeed.get());
        },
        () -> {
          subsystemIO.setSpeed(0);
        },
        this);
  }
}
