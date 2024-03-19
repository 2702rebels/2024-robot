package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(651.223);
  public static final double fieldWidth = Units.inchesToMeters(323.277);
  public static final double wingX = Units.inchesToMeters(229.201);
  public static final double podiumX = Units.inchesToMeters(126.75);
  public static final double startingLineX = Units.inchesToMeters(74.111);

  // in meters
  public static final Translation2d NoteSpeakerBlue = new Translation2d(2.89, 5.55);
}
