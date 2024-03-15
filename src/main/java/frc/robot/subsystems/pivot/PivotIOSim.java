// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class PivotIOSim implements PivotIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private final DCMotorSim motorSim = new DCMotorSim(DCMotor.getNEO(1), 9 * 5 * (44 / 15), 0);

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    motorSim.update(LOOP_PERIOD_SECS);

    inputs.measuredPositionRadians = motorSim.getAngularPositionRotations();
    inputs.measuredVelocityRadiansPerSecond = motorSim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
  }
}
