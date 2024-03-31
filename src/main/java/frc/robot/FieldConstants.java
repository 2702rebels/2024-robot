package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Supplier;

public final class FieldConstants {
  public static final double fieldLengthMeters = 16.5410515;

  /** Returns the pose mirrored across the field midline. */
  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(
        fieldLengthMeters - pose.getX(),
        pose.getY(),
        Rotation2d.fromRadians(Math.PI - pose.getRotation().getRadians()));
  }

  /** Returns initial pose based on the selected alliance. */
  public static Supplier<Pose2d> getInitialPose(Pose2d pose) {
    return () -> {
      final var flipped =
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
      return flipped ? flip(pose) : pose;
    };
  }

  // in meters
  public static final Pose2d initialAmpSide =
      new Pose2d(1.3, 7.02, Rotation2d.fromDegrees(45)); // blue

  public static final Pose2d initialCenterSide =
      new Pose2d(1.383, 5.576, Rotation2d.fromDegrees(0)); // blue

  public static final Pose2d initialSourceSide =
      new Pose2d(1.3, 4.1, Rotation2d.fromDegrees(-45)); // blue
}
