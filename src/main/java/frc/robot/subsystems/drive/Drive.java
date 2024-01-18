package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  public static final Lock odometryLock = new ReentrantLock();

  private final boolean useFrcOdometry;
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, RL, RR
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());

  private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(kinematics, gyroInputs.yawPosition, getModulePositions());

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
  private Pose2d pose = new Pose2d();
  private Rotation2d lastGyroRotation = new Rotation2d();

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO rlModuleIO,
      ModuleIO rrModuleIO,
      double[] chassisAngularOffsetRad,
      boolean useFrcOdometry) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, chassisAngularOffsetRad[0]);
    modules[1] = new Module(frModuleIO, 1, chassisAngularOffsetRad[1]);
    modules[2] = new Module(rlModuleIO, 2, chassisAngularOffsetRad[2]);
    modules[3] = new Module(rrModuleIO, 3, chassisAngularOffsetRad[3]);

    // determines whether to use odometry based on FRC wpilib SwerveDriveOdometry
    // or custom odometry tracking
    this.useFrcOdometry = useFrcOdometry;

    orientationController = new PIDController(orientationP.get(), 0, 0);
    orientationController.enableContinuousInput(-Math.PI, Math.PI);

    // configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            Constants.MAX_LINEAR_SPEED, Constants.DRIVE_BASE_RADIUS, new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);

    Pathfinding.setPathfinder(new LocalADStarAK());
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

  public void periodic() {
    final var callerId = hashCode();

    // update controllers if tunable numbers have changed
    if (orientationP.hasChanged(callerId)) {
      orientationController.setP(orientationP.get());
    }

    odometryLock.lock(); // prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();

    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // update odometry (FRC odometry)
    if (useFrcOdometry) {
      odometry.update(gyroInputs.yawPosition, getModulePositions());
    }

    // update odometry (Advantage Kit example)
    var deltaCount =
        gyroInputs.connected ? gyroInputs.odometryYawPositions.length : Integer.MAX_VALUE;

    for (var i = 0; i < 4; i++) {
      deltaCount = Math.min(deltaCount, modules[i].getPositionDeltas().length);
    }

    for (var deltaIndex = 0; deltaIndex < deltaCount; deltaIndex++) {
      // read wheel deltas from each module
      final var wheelDeltas = new SwerveModulePosition[4];
      for (var moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        wheelDeltas[moduleIndex] = modules[moduleIndex].getPositionDeltas()[deltaIndex];
      }

      // the twist represents the motion of the robot since the last
      // sample in x, y, and theta based only on the modules, without
      // the gyro; the gyro is always disconnected in simulation
      var twist = kinematics.toTwist2d(wheelDeltas);
      if (gyroInputs.connected) {
        // if the gyro is connected, replace the theta component of the twist
        // with the change in angle since the last sample
        final var gyroRotation = gyroInputs.odometryYawPositions[deltaIndex];
        twist = new Twist2d(twist.dx, twist.dy, gyroRotation.minus(lastGyroRotation).getRadians());
        lastGyroRotation = gyroRotation;
      }

      // apply the twist (change since last sample) to the current pose
      pose = pose.exp(twist);
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
        && Math.abs(speeds.vxMetersPerSecond) <= Constants.Epsilon
        && Math.abs(speeds.vyMetersPerSecond) <= Constants.Epsilon
        && Math.abs(speeds.omegaRadiansPerSecond) <= Constants.Epsilon) {
      setpointStates = stopXstates;
    } else {
      // calculate module setpoints
      final var discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
      setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.MAX_LINEAR_SPEED);
    }

    // send setpoints to modules
    final var optimizedSetpointStates = new SwerveModuleState[4];
    for (var i = 0; i < 4; i++) {
      // the module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // log setpoint states
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
          Math.abs(linearMagnitude) > Constants.Epsilon
              || Math.abs(angularMagnitude) > Constants.Epsilon;

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
    for (var i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions to use with FRC odometry implementations. */
  private SwerveModulePosition[] getModulePositions() {
    final var positions = new SwerveModulePosition[4];
    for (var i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /** Returns the current odometry pose */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return useFrcOdometry ? odometry.getPoseMeters() : pose;
  }

  /** Returns the current odometry rotation */
  public Rotation2d getRotation() {
    return useFrcOdometry ? getPose().getRotation() : pose.getRotation();
  }

  /** Resets the current odometry pose to the specified value */
  public void setPose(Pose2d pose) {
    this.pose = pose;
  }

  /** Returns the maximum linear speed in meters per sec */
  public double getMaxLinearSpeedMetersPerSec() {
    return Constants.MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec */
  public double getMaxAngularSpeedRadPerSec() {
    return Constants.MAX_ANGULAR_SPEED;
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

  /** Returns an array of module translations */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(Constants.TRACK_WIDTH_Y_M / 2.0, Constants.TRACK_WIDTH_X_M / 2.0),
      new Translation2d(Constants.TRACK_WIDTH_Y_M / 2.0, -Constants.TRACK_WIDTH_X_M / 2.0),
      new Translation2d(-Constants.TRACK_WIDTH_Y_M / 2.0, Constants.TRACK_WIDTH_X_M / 2.0),
      new Translation2d(-Constants.TRACK_WIDTH_Y_M / 2.0, -Constants.TRACK_WIDTH_X_M / 2.0)
    };
  }
}
