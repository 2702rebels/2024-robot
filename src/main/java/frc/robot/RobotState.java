package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.Kinematics;
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

  private final Kinematics<?, SwerveDriveWheelPositions> kinematics;

  private Pose2d odometryPose = new Pose2d();
  private Pose2d estimatedPose = new Pose2d();

  private static final double poseBufferSizeSeconds = 2.0;
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);

  private Rotation2d prevGyroAngle = new Rotation2d();
  private SwerveDriveWheelPositions prevWheelPositions =
      new SwerveDriveWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          });

  private RobotState() {
    kinematics = DriveConstants.kinematics;
  }

  /** Adds new odometry observation and updates robot pose. */
  public void addOdometryObservation(
      SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, double timestamp) {
    final var prevOdometryPose = odometryPose;

    // the twist represents the motion of the robot since the last
    // sample in x, y, and theta based only on the modules, without
    // the gyro; the gyro is always disconnected in simulation
    var twist = kinematics.toTwist2d(prevWheelPositions, wheelPositions);

    // populate previous wheel positions (no extra allocations)
    for (var i = 0; i < wheelPositions.positions.length; ++i) {
      prevWheelPositions.positions[i].distanceMeters = wheelPositions.positions[i].distanceMeters;
      prevWheelPositions.positions[i].angle = wheelPositions.positions[i].angle;
    }

    // update dtheta for twist if gyro is connected and angle is provided
    if (gyroAngle != null) {
      twist.dtheta = gyroAngle.getRadians() - prevGyroAngle.getRadians();
      prevGyroAngle = gyroAngle;
    }

    // apply twist to the current odometry pose
    odometryPose = odometryPose.exp(twist);

    // add pose to buffer at timestamp
    poseBuffer.addSample(timestamp, odometryPose);

    // calculate diff from last odometry pose and add onto pose estimate
    estimatedPose = estimatedPose.exp(prevOdometryPose.log(odometryPose));
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
}
