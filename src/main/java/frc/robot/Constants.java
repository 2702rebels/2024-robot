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

  public static final class IntakeConstants {
    public static final int MotorControllerCANId = 11;
  }

      public static final class ShooterConstants{
      public static final int kShooterFeed = 23;
        public static final int kShooterFlywheel1 = 24;
        public static final int kShooterFlywheel2 = 25;
        public static final double kTalonCountsToRPM = 600.0 / 2048.0;
    }
}
