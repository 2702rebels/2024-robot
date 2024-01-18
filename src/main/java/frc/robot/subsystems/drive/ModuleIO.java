package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double drivePositionMeter = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};

    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrentAmps = new double[] {};

    public double[] odometryDrivePositionsMeter = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Runs the drive motor at the specified voltage. */
  public default void setDriveVoltage(double volts) {}

  /** Runs the turn motor at the specified voltage. */
  public default void setTurnVoltage(double volts) {}

  /** Enables or disables brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enables or disables brake mode on the turn motor. */
  public default void setTurnBrakeMode(boolean enable) {}

  /** Configures PID for the drive motor. */
  public default void configureDrivePID(double kP, double kI, double kD, double kFF) {}

  /** Configures PID for the turn motor. */
  public default void configureTurnPID(double kP, double kI, double kD, double kFF) {}

  /** Sets the drive motor target velocity via PID. */
  public default void setDriveVelocity(double speedMetersPerSecond) {}

  /** Sets the turn motor target angle via PID. */
  public default void setTurnAngle(double angleRadians) {}

  /** Sets the turn motor target angle and FF via PID. */
  public default void setTurnAngleAndFF(double angleRadians, double ff) {}

  /** Resets encoders position. */
  public default void resetEncoders() {}
}
