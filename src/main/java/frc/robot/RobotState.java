package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  private Pose2d odometryPose = new Pose2d();
  private Pose2d estimatedPose = new Pose2d();

  private static final double poseBufferSizeSeconds = 2.0;
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);

  private Rotation2d lastGyroAngle = new Rotation2d();
  private SwerveDriveWheelPositions lastWheelPositions =
      new SwerveDriveWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          });

  private RobotState() {
    kinematics = DriveConstants.kinematics;
    odometry =
        new SwerveDriveOdometry(
            DriveConstants.kinematics, lastGyroAngle, lastWheelPositions.positions);
  }

  /** Adds new odometry observation and updates robot pose. */
  public void addOdometryObservation(OdometryObservation observation) {
    final var lastOdometryPose = odometryPose;

    // the twist represents the motion of the robot since the last
    // sample in x, y, and theta based only on the modules, without
    // the gyro; the gyro is always disconnected in simulation
    var twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
    lastWheelPositions = observation.wheelPositions();

    if (observation.gyroAngle != null) {
      // update dtheta for twist if gyro is connected and angle is provided
      final var gyroDeltaRadians =
          observation.gyroAngle().getRadians() - lastGyroAngle.getRadians();
      twist = new Twist2d(twist.dx, twist.dy, gyroDeltaRadians);
      lastGyroAngle = observation.gyroAngle();
    }

    // apply twist to the current odometry pose
    odometryPose = odometryPose.exp(twist);

    // add pose to buffer at timestamp
    poseBuffer.addSample(observation.timestamp(), odometryPose);

    // calculate diff from last odometry pose and add onto pose estimate
    estimatedPose = estimatedPose.exp(lastOdometryPose.log(odometryPose));

    // FRC odometry
    odometry.update(observation.gyroAngle(), observation.wheelPositions());
  }

  /** Resets pose. */
  public void resetPose(Pose2d initialPose) {
    estimatedPose = initialPose;
    odometryPose = initialPose;
    poseBuffer.clear();
  }

  /** Returns the current (estimated) robot pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/OdometryPose")
  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  /** Returns the current FRC odometry pose. */
  @AutoLogOutput(key = "Odometry/FrcOdometryPose")
  public Pose2d getFrcOdometryPose() {
    return odometry.getPoseMeters();
  }

  /** Odometry observation. */
  public record OdometryObservation(
      SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, double timestamp) {}
}
