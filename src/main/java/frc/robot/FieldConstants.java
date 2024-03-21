package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Supplier;

public final class FieldConstants {
  /** Returns initial pose based on the selected alliance. */
  public static Supplier<Pose2d> getInitialPose(Pose2d[] poses) {
    return () ->
        poses[
            DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red
                ? 1
                : 0];
  }

  // in meters
  public static final Pose2d[] initialAmpSide = {
    new Pose2d(1.3, 7.02, Rotation2d.fromDegrees(45)), // blue
    new Pose2d(15.25, 7.02, Rotation2d.fromDegrees(135)) // red
  };
}
