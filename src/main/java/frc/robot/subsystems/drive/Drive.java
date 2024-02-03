package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Function;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, RL, RR

  private final DriveOdometryTracker odometryTracker;
  private final RobotState robotState;

  private final ChassisSpeeds ZeroChassisSpeeds = new ChassisSpeeds();
  private final SwerveModuleState[] stopXstates =
      new SwerveModuleState[] {
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      };

  private static final LoggedTunableNumber orientationP =
      new LoggedTunableNumber("Drive/Orientation/P", 0.5);

  // orientation to maintain while lock is in place
  private double orientationLockAngleRad;
  private boolean orientationLockEnabled;
  private final PIDController orientationController;

  private boolean stopX;

  public Drive(GyroIO gyroIO, Function<Integer, ModuleIO> moduleBuilder) {
    // initialize swerve modules
    for (var index = 0; index < modules.length; ++index) {
      modules[index] =
          new Module(moduleBuilder.apply(index), index, DriveConstants.moduleAngularOffsets[index]);
    }

    this.gyroIO = gyroIO;
    this.robotState = frc.robot.RobotState.getInstance();
    this.odometryTracker = DriveOdometryTracker.getInstance();

    orientationController = new PIDController(orientationP.get(), 0, 0);
    orientationController.enableContinuousInput(-Math.PI, Math.PI);

    // configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> DriveConstants.kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            DriveConstants.drivetrainConfig.maxLinearSpeed(),
            DriveConstants.drivetrainConfig.driveBaseRadius(),
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);

    // Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  /**
   * Updates loggable inputs from IO modules impacting odometry. Must be invoked under
   * DriveOdometryTracker lock.
   */
  private void updateOdometryInputs() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (var module : modules) {
      module.updateInputs();
    }
  }

  /** Updates robot state based from odometry observations. */
  private void updateOdometry() {
    final var timestamps = odometryTracker.read(this::updateOdometryInputs);
    final var yawPositions = gyroInputs.odometryYawPositions;

    final var modulePositions = new SwerveModulePosition[modules.length][];
    var modulePositionsCount = Integer.MAX_VALUE;
    for (var i = 0; i < modules.length; i++) {
      modulePositions[i] = modules[i].getPositions();
      modulePositionsCount = Math.min(modulePositionsCount, modulePositions[i].length);
    }

    var index = 0;
    while (true) {
      if (index >= timestamps.length
          || gyroInputs.connected && index >= yawPositions.length
          || index >= modulePositionsCount) {
        break;
      }

      final var timestamp = timestamps[index];
      final var yaw = gyroInputs.connected ? yawPositions[index] : null;
      final var wheelPositions =
          new SwerveDriveWheelPositions(
              new SwerveModulePosition[] {
                modulePositions[0][index],
                modulePositions[1][index],
                modulePositions[2][index],
                modulePositions[3][index]
              });

      robotState.addOdometryObservation(
          new RobotState.OdometryObservation(wheelPositions, yaw, timestamp));

      index++;
    }
  }

  public void periodic() {
    final var callerId = hashCode();

    // update controllers if tunable numbers have changed
    if (orientationP.hasChanged(callerId)) {
      orientationController.setP(orientationP.get());
    }

    for (var module : modules) {
      module.periodic();
    }

    updateOdometry();

    // stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }

      // log empty setpoint states when disabled
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    SwerveModuleState[] setpointStates;

    // if all commanded velocities are effectively zero and
    // robot must be stopped in X wheel arrangement, use custom setpoints
    if (stopX
        && Math.abs(speeds.vxMetersPerSecond) <= Constants.epsilon
        && Math.abs(speeds.vyMetersPerSecond) <= Constants.epsilon
        && Math.abs(speeds.omegaRadiansPerSecond) <= Constants.epsilon) {
      setpointStates = stopXstates;
    } else {
      // calculate module setpoints
      final var discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
      setpointStates = DriveConstants.kinematics.toSwerveModuleStates(discreteSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(
          setpointStates, DriveConstants.drivetrainConfig.maxLinearSpeed());
    }

    // send setpoints to modules
    final var optimizedSetpointStates = new SwerveModuleState[4];
    for (var i = 0; i < modules.length; i++) {
      // the module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // log chassis speeds and setpoint states
    Logger.recordOutput("Drive/Speeds", speeds);
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(ZeroChassisSpeeds);
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    stopX = true;
    stop();
  }

  /**
   * If the robot is last stopped in an X arrangement, determines whether to resume normal movement
   * based on the magnitudes of input velocities.
   *
   * @param linearMagnitude linear velocity magnitude
   * @param angularMagnitude angular velocity magnitude
   */
  public void resumeFromX(double linearMagnitude, double angularMagnitude) {
    if (stopX) {
      final var movementRequested =
          Math.abs(linearMagnitude) > Constants.epsilon
              || Math.abs(angularMagnitude) > Constants.epsilon;

      if (movementRequested) {
        stopX = false;
      }
    }
  }

  /** Returns the average drive velocity in meters/sec. */
  public double getCharacterizationVelocity() {
    var driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getVelocityMetersPerSec();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    final var states = new SwerveModuleState[4];
    for (var i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the current odometry rotation */
  public Rotation2d getRotation() {
    return robotState.getEstimatedPose().getRotation();
  }

  /** Returns the maximum linear speed in meters per sec */
  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.drivetrainConfig.maxLinearSpeed();
  }

  /** Returns the maximum angular speed in radians per sec */
  public double getMaxAngularSpeedRadPerSec() {
    return DriveConstants.drivetrainConfig.maxAngularSpeed();
  }

  /** Enables orientation lock. */
  public void enableOrientationLock(double angleRadians) {
    orientationLockEnabled = true;
    orientationLockAngleRad = angleRadians;
    orientationController.setSetpoint(angleRadians);
  }

  /** Toggles orientation lock. */
  public void toggleOrientationLock(double angleRadians) {
    if (orientationLockEnabled && orientationLockAngleRad == angleRadians) {
      disableOrientationLock();
    } else {
      enableOrientationLock(angleRadians);
    }
  }

  /** Disables orientation lock. */
  public void disableOrientationLock() {
    orientationLockEnabled = false;
    orientationLockAngleRad = 0;
    orientationController.setSetpoint(0);
  }

  /** Returns rotation input required to maintain orientation lock. */
  public double getOrientationLockRotation() {
    return orientationController.calculate(getRotation().getRadians());
  }

  /** Determines whether orientation lock is enabled. */
  public boolean hasOrientationLock() {
    return orientationLockEnabled;
  }

  /** Gets robot pose. */
  public Pose2d getPose() {
    return robotState.getEstimatedPose();
  }

  /** Sets robot pose. */
  public void setPose(Pose2d initialPose) {
    robotState.resetPose(initialPose);
  }

  /** Reset encoders. */
  public void reset() {
    for (var module : modules) {
      module.resetEncoders();
    }
  }
}
