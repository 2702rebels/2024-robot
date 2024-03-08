// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public double position = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void zeroEncoder() {}

  public default void setPercentOutput(double speed) {}

  public default void setPosition(double position) {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default void setservoposition(double position) {}
}
