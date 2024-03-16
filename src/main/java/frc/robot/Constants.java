package frc.robot;

public final class Constants {
  /** Robot settings can be tuned via NT (SmartDashboard) */
  public static final boolean tuningMode = true;

  /** Robot code operation mode */
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot */
    REAL,

    /** Running a physics simulator */
    SIM,

    /** Replaying from a log file */
    REPLAY,
  }

  /** Default numerical threshold for "zero" values */
  public static final double epsilon = 1e-6;

  /** Robot main loop cycle period in seconds */
  public static final double cyclePeriodSeconds = 0.02;

  /** Odometry frequency (Hz) */
  public static final int odometryFrequency = 200;

  public static final int CLIMB_SERVO = 2;

  /** Digital IO port the climb limit switch is plugged into. */
  public static final int CLIMB_LIMITSWITCH = 5;

  /** Digital IO port the beambreak is plugged into. */
  public static final int SHOOTER_BEAMBREAKFRONT = 8;

  public static final int SHOOTER_BEAMBREAKBACK = 7;
  public static final int SHOOTER_BEAMBREAKINTAKE = 6;
}
