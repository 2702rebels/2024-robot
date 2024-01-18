package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Misc;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final Rotation2d angularOffset;

  private double lastPositionMeters = 0.0; // used for delta calculation
  private SwerveModulePosition[] positionDeltas = new SwerveModulePosition[] {};

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
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        driveP.initDefault(Constants.DRIVE_MOTOR_P);
        driveI.initDefault(Constants.DRIVE_MOTOR_I);
        driveD.initDefault(Constants.DRIVE_MOTOR_D);
        driveFF.initDefault(Constants.DRIVE_MOTOR_FF);
        turnP.initDefault(Constants.TURN_MOTOR_P);
        turnI.initDefault(Constants.TURN_MOTOR_I);
        turnD.initDefault(Constants.TURN_MOTOR_D);
        turnFF.initDefault(Constants.TURN_MOTOR_FF);
        break;
      case SIM:
        driveP.initDefault(0.0);
        driveI.initDefault(0.0);
        driveD.initDefault(0.0);
        driveFF.initDefault(0.0);
        turnP.initDefault(0.0);
        turnI.initDefault(0.0);
        turnD.initDefault(0.0);
        turnFF.initDefault(0.0);
        break;
    }
  }

  public Module(ModuleIO io, int index, double chassisAngularOffsetRadians) {
    this.io = io;
    this.index = index;
    angularOffset = Rotation2d.fromRadians(chassisAngularOffsetRadians);

    setBrakeMode(true);
  }

  /**
   * Updates inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void periodic() {
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

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

    // TODO: use constant size buffers to minimize allocations
    // calculate position deltas for odometry
    final var deltaCount =
        Math.min(inputs.odometryDrivePositionsMeter.length, inputs.odometryTurnPositions.length);

    positionDeltas = new SwerveModulePosition[deltaCount];
    for (var i = 0; i < deltaCount; i++) {
      final var positionMeters = inputs.odometryDrivePositionsMeter[i];
      final var angle = inputs.odometryTurnPositions[i];
      positionDeltas[i] = new SwerveModulePosition(positionMeters - lastPositionMeters, angle);
      lastPositionMeters = positionMeters;
    }
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // optimize state based on current angle
    final var optimizedState = adjustAndOptimize(state, getAngle());

    // command driving and turning SPARK MAX towards their respective setpoints
    io.setDriveVelocity(optimizedState.speedMetersPerSecond);

    // TODO: support for second-order kinematics
    // var desiredOmega = optimizedState.omega.getRadians(); // radians per second
    // var turningFF =
    //     Constants.TURN_MOTOR_NOMINAL_VOLTAGE
    //         * desiredOmega
    //         / Constants.TURN_MOTOR_MAX_ANGULAR_VELOCITY;
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
    return inputs.drivePositionMeter;
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

  /** Returns the module position deltas received this cycle. */
  public SwerveModulePosition[] getPositionDeltas() {
    return positionDeltas;
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
  private SwerveModuleState adjustAndOptimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
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
