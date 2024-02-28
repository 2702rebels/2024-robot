// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public Rotation2d measuredPosition = new Rotation2d();
    public double measuredVelocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;

    // public double calculatedPosition = 0.0;
    // public double calculatedVelocity = 0.0;
    // public double calculatedArmFeedForwards = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PivotIOInputs inputs) {}

  /** Enables or Disables brake mode on the joint. */
  public default void setBrakeMode(boolean enable) {}

  /** Confiure the PID for the joint. */
  public default void configureJointPID(double kP, double kI, double kD) {}

  //   /** Configure the ArmFeedForward for the joint. */
  //   public default void configureArmFeedForward(double ks, double kg, double kv, double ka) {}

  //   /** Configure the TrapezoidProfile for the joint. */
  //   public default void configureTrapezoidProfile(double kV, double ka) {}

  /** Sets the target angle for the joint PID. */
  public default void setJointAngle(double angleRadians) {}

  /** Sets the target angle for the joint PID with a FF term. */
  public default void setJointAngle(double angleRadians, double ff) {}

  /** Stops the motor. */
  public default void stop() {}
}
