package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public final class Misc {
  private static final double TwoPI = 2 * Math.PI;

  /** Adds angles with minimum allocations. */
  public static Rotation2d add(Rotation2d a, Rotation2d b) {
    return new Rotation2d(a.getRadians() + b.getRadians());
  }

  /** Subtracts angles with minimum allocations. */
  public static Rotation2d subtract(Rotation2d a, Rotation2d b) {
    return new Rotation2d(a.getRadians() - b.getRadians());
  }

  /** Returns angle in radians clamped to (-PI, PI] range. */
  public static double clampAngle(double angleRadians) {
    while (angleRadians <= -Math.PI) {
      angleRadians += TwoPI;
    }

    while (angleRadians > Math.PI) {
      angleRadians -= TwoPI;
    }

    return angleRadians;
  }

  /**
   * Returns angle in radians representing the shortest arc between two inputs. The sign of returned
   * value indicates the direction of the rotation: positive is CCW, negative is CW. For example:
   *
   * <pre>
   * from -> to = delta
   * 30° -> -30° = -60°
   * 30° -> 150° = 120°
   * 30° -> 300° = -90°
   * -150° -> 150° = -60°
   * -30° -> -270° = 120°
   * </pre>
   */
  public static double shortestArc(double fromRadians, double toRadians) {
    return clampAngle(toRadians - fromRadians);
  }
}
