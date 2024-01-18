package frc.robot;

import edu.wpi.first.math.util.Units;

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
  public static final double Epsilon = 1e-6;

  /** Odometry frequency (Hz) */
  public static final int ODOMETRY_FREQUENCY = 250;

  /** Drive wheel diameter (m) */
  public static final double WHEEL_DIAMETER_M = Units.inchesToMeters(3.0);

  /** Distance between right and left wheels */
  public static final double TRACK_WIDTH_X_M = Units.inchesToMeters(21.5);

  /** Distance between front and rear wheels */
  public static final double TRACK_WIDTH_Y_M = Units.inchesToMeters(21.5);

  /** Drive base radius */
  public static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X_M / 2.0, TRACK_WIDTH_Y_M / 2.0);

  /** Maximum linear speed (meters/sec) */
  public static final double MAX_LINEAR_SPEED = 4.0;

  /** Maximum angular speed (rad/s) */
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  /** Angular positions of FL, FR, RL, RR modules with respect to the chassis */
  public static final double[] CHASSIS_ANGULAR_OFFSET_RAD = {
    -Math.PI / 2, 0, Math.PI, Math.PI / 2,
  };

  public static final double SLEW_RATE_DIRECTION = Math.PI / 2; // radians per second
  public static final double SLEW_RATE_ACCELRATE = 0.8; // percent per second (1 = 100%)
  public static final double SLEW_RATE_DECELERATE = 2.0; // percent per second (1 = 100%)
  public static final double SLEW_RATE_ROTATION = 1.5; // percent per second (1 = 100%)

  public static final int DRIVE_ENCODER_MEASUREMENT_PERIOD = 10;
  public static final int DRIVE_ENCODER_MEASUREMENT_DEPTH = 4;
  public static final int TURN_ENCODER_MEASUREMENT_DEPTH = 2;

  /**
   * The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T. This
   * changes the drive speed of the module (a pinion gear with more teeth will result in a robot
   * that drives faster).
   */
  public static final double DRIVE_MOTOR_PINION_TEETH = 14.0;

  /**
   * 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
   * bevel pinion
   */
  public static final double DRIVE_MOTOR_REDUCTION =
      (45.0 * 22.0) / (DRIVE_MOTOR_PINION_TEETH * 15.0);

  /** NEO free speed is 5676 RPM */
  public static final double DRIVE_MOTOR_FREE_SPEED_RPS = 5676.0 / 60;

  /**
   * Factor to convert internal encoder rotations into distance based on the wheel size and gear
   * reduction (meters)
   */
  public static final double DRIVE_ENCODER_POSITION_FACTOR =
      (Math.PI * WHEEL_DIAMETER_M) / DRIVE_MOTOR_REDUCTION;

  /** Factor to convert internal encoder rotations into linear velocity (meters/sec) */
  public static final double DRIVE_ENCODER_VELOCITY_FACTOR = DRIVE_ENCODER_POSITION_FACTOR / 60.0;

  public static final double DRIVE_WHEEL_FREE_SPEED_RPS =
      DRIVE_MOTOR_FREE_SPEED_RPS * DRIVE_ENCODER_POSITION_FACTOR;

  /** Azimuth ratio from REV specification https://www.revrobotics.com/rev-21-3005/ */
  public static final double TURN_MOTOR_REDUCTION = 9424.0 / 203.0;

  /** NEO550 free speed is 11000 RPM */
  public static final double TURN_MOTOR_FREE_SPEED_RPS = 11000.0 / 60;

  /**
   * Invert the turning encoder, since the output shaft rotates in the opposite direction of the
   * steering motor in the MAXSwerve module
   */
  public static final boolean TURN_ENCODER_INVERTED = true;

  /** Factor to convert internal encoder rotations to radians (radians) */
  public static final double TURN_ENCODER_POSITION_FACTOR = 2 * Math.PI;

  /** Factor to convert internal encoder rotations to angular velocity (radians/sec) */
  public static final double TURN_ENCODER_VELOCITY_FACTOR = TURN_ENCODER_POSITION_FACTOR / 60.0;

  public static final double TURN_MOTOR_MAX_ANGULAR_VELOCITY =
      TURN_ENCODER_POSITION_FACTOR * TURN_MOTOR_FREE_SPEED_RPS / TURN_MOTOR_REDUCTION;

  public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
  public static final int TURN_MOTOR_CURRENT_LIMIT = 20;
  public static final int DRIVE_MOTOR_NOMINAL_VOLTAGE = 12;
  public static final int TURN_MOTOR_NOMINAL_VOLTAGE = 12;

  public static final double DRIVE_MOTOR_P = 0.1;
  public static final double DRIVE_MOTOR_I = 0.0;
  public static final double DRIVE_MOTOR_D = 0.0;
  public static final double DRIVE_MOTOR_FF =
      (1 / DRIVE_WHEEL_FREE_SPEED_RPS) * 1.1; // add +10% to overshoot and pull down with P

  public static final double TURN_MOTOR_P = 2.5;
  public static final double TURN_MOTOR_I = 0;
  public static final double TURN_MOTOR_D = 0;
  public static final double TURN_MOTOR_FF = 0;
}
