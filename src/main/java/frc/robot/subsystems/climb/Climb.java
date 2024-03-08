// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  ClimbIO subsystemio;
  ClimbIOInputsAutoLogged subsystemInputs = new ClimbIOInputsAutoLogged();

  /** Creates a new Climb. */
  public Climb(ClimbIO climb) {
    subsystemio = climb;
  }

  @Override
  public void periodic() {
    subsystemio.updateInputs(subsystemInputs);
  }
}
