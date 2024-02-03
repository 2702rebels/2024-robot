package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Misc;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final Rotation2d angularOffset;

  private static final LoggedTunableNumber driveP = new LoggedTunableNumber("Drive/Module/Drive P");
  private static final LoggedTunableNumber driveI = new LoggedTunableNumber("Drive/Module/Drive I");
  private static final LoggedTunableNumber driveD = new LoggedTunableNumber("Drive/Module/Drive D");
  private static final LoggedTunableNumber driveFF =
      new LoggedTunableNumber("Drive/Module/Drive FF");

  private static final LoggedTunableNumber turnP = new LoggedTunableNumber("Drive/Module/Turn P");
  private static final LoggedTunableNumber turnI = new LoggedTunableNumber("Drive/Module/Turn I");
  private static final LoggedTunableNumber turnD = new LoggedTunableNumber("Drive/Module/Turn D");
  private static final LoggedTunableNumber turnFF = new LoggedTunableNumber("Drive/Module/Turn FF");

  static {
    driveP.initDefault(DriveConstants.moduleFactors.driveMotor().kP());
    driveI.initDefault(DriveConstants.moduleFactors.driveMotor().kI());
    driveD.initDefault(DriveConstants.moduleFactors.driveMotor().kD());
    driveFF.initDefault(DriveConstants.moduleFactors.driveMotor().kFF());
    turnP.initDefault(DriveConstants.moduleFactors.turnMotor().kP());
    turnI.initDefault(DriveConstants.moduleFactors.turnMotor().kI());
    turnD.initDefault(DriveConstants.moduleFactors.turnMotor().kD());
    turnFF.initDefault(DriveConstants.moduleFactors.turnMotor().kFF());
  }

  public Module(ModuleIO io, int index, Rotation2d chassisAngularOffset) {
    this.io = io;
    this.index = index;
    angularOffset = chassisAngularOffset;

    setBrakeMode(true);
  }

  /**
   * Updates inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
  }

  public void periodic() {
    final var callerId = hashCode();

    // update controllers if tunable numbers have changed
    if (driveP.hasChanged(callerId)
        || driveI.hasChanged(callerId)
        || driveD.hasChanged(callerId)
        || driveFF.hasChanged(callerId)) {
      io.configureDrivePID(driveP.get(), driveI.get(), driveD.get(), driveFF.get());
    }

    if (turnP.hasChanged(callerId)
        || turnI.hasChanged(callerId)
        || turnD.hasChanged(callerId)
        || turnFF.hasChanged(callerId)) {
      io.configureTurnPID(turnP.get(), turnI.get(), turnD.get(), turnFF.get());
    }
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // optimize state based on current angle
    final var optimizedState = adjustAndOptimize(state, getAngle(), angularOffset);

    // command driving and turning SPARK MAX towards their respective setpoints
    io.setDriveVelocity(optimizedState.speedMetersPerSecond);

    // TODO: support for second-order kinematics
    // var desiredOmega = optimizedState.omega.getRadians(); // radians per second
    // var turningFF =
    //     DriveConstants.drivetrainConfig.turnMotor().nominalVoltage()
    //         * desiredOmega
    //         / DriveConstants.drivetrainConfig.turnMotorMaxAngularSpeed();
    // io.setTurnAngleAndFF(optimizedState.angle.getRadians(), turningFF);

    io.setTurnAngle(optimizedState.angle.getRadians());
    return optimizedState;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /** Returns recently recorded sequence of positions fusing both drive and turn odometries. */
  public SwerveModulePosition[] getPositions() {
    final var minOdometryPositions =
        Math.min(inputs.odometryDrivePositionsMeter.length, inputs.odometryTurnPositions.length);
    final var positions = new SwerveModulePosition[minOdometryPositions];
    for (var i = 0; i < minOdometryPositions; i++) {
      positions[i] =
          new SwerveModulePosition(
              inputs.odometryDrivePositionsMeter[i], inputs.odometryTurnPositions[i]);
    }
    return positions;
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current turn angle of the module adjusted for offset. */
  public Rotation2d getAdjustedAngle() {
    return Misc.subtract(inputs.turnPosition, angularOffset);
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeter() {
    return inputs.drivePositionMeters;
  }

  /** Returns the current drive velocity of the module in meters/sec. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityMetersPerSec;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeter(), getAdjustedAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAdjustedAngle());
  }

  /** Resets encoders position. */
  public void resetEncoders() {
    io.resetEncoders();
  }

  /**
   * Replacement for FRC WPILIB SwerveModuleState.optimize method with additional logic to improve
   * numerical stability and reduce allocations. This also adjusts the desired state to account for
   * the angular offset.
   */
  private static SwerveModuleState adjustAndOptimize(
      SwerveModuleState desiredState, Rotation2d currentAngle, Rotation2d angularOffset) {
    final var desiredAngle = desiredState.angle.getRadians() + angularOffset.getRadians();

    final var delta = Math.abs(desiredAngle - currentAngle.getRadians());
    if (delta < 0.1) {
      return new SwerveModuleState(
          desiredState.speedMetersPerSecond, Rotation2d.fromRadians(desiredAngle));
    }

    // a bit over 90 degrees to avoid instability
    final double threshold = Math.PI * 0.5 * 1.01;
    if (delta > threshold) {
      return new SwerveModuleState(
          -desiredState.speedMetersPerSecond,
          Rotation2d.fromRadians(desiredAngle + Math.toRadians(180.0)));
    }

    return new SwerveModuleState(
        desiredState.speedMetersPerSecond, Rotation2d.fromRadians(desiredAngle));
  }
}
