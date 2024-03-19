package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.AprilTag;
import frc.robot.subsystems.vision.AprilTagDetector;
import org.littletonrobotics.junction.Logger;

public class ScoreInSpeaker extends Command {
  private final Drive drive;
  private final Shooter shooter;
  private final Pivot pivot;
  private final AprilTagDetector aprilTagDetector;
  private final Led leds;
  private final RobotState state;
  private AprilTag tag;

  private boolean isStillTracking = false;

  public ScoreInSpeaker(
      Drive drive, Shooter shooter, Pivot pivot, AprilTagDetector aprilTagDetector, Led leds) {
    this.drive = drive;
    this.shooter = shooter;
    this.pivot = pivot;
    this.aprilTagDetector = aprilTagDetector;
    this.leds = leds;
    this.state = RobotState.getInstance();

    addRequirements(shooter, pivot, leds);
  }

  private Translation2d getOdometryCorrectedTagVector() {
    final var timestamp = Timer.getFPGATimestamp();
    return state.correctViaOdometry(new Translation2d(tag.x, tag.z), tag.timestamp, timestamp);
  }

  @Override
  public void initialize() {
    // get the tag we need to be scoring on
    tag = aprilTagDetector.getTag(DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7);

    pivot.setRequestedAngle(30);
    shooter.setSpeed(4000);

    leds.setTracking(false);
    isStillTracking = false;
  }

  @Override
  public void execute() {
    final var vector = getOdometryCorrectedTagVector();
    final var z = vector.getY();
    final var x = vector.getX();

    Logger.recordOutput(
        "TrackedShooting/Target",
        drive.getPose().getTranslation().plus(vector.rotateBy(drive.getPose().getRotation())));

    final var pivotAngle = Math.toDegrees(Math.atan2(-2.3, z));
    Logger.recordOutput("TrackedShooting/PivotAngle", pivotAngle);

    final var turnAngle = Math.atan2(x, z);
    Logger.recordOutput("TrackedShooting/TurnAngle", turnAngle);

    final var speed = z > 4.0 ? 4000 : 3000;
    Logger.recordOutput("TrackedShooting/Speed", speed);

    if (tag.tracked) {
      if (!isStillTracking) {
        isStillTracking = true;
        leds.setTracking(true);
      }

      drive.enableOrientationLock(drive.getRotation().getRadians() - turnAngle);

      // Hack to update drive train directly if running in auto. Instead of the hack to piggyback
      // off the teleop drive command.
      if (DriverStation.isAutonomous()) {
        final var speeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                0,
                0,
                drive.getOrientationLockRotation() * drive.getMaxAngularSpeedRadPerSec(),
                drive.getRotation());

        drive.runVelocity(speeds);
      }

      pivot.setRequestedAngle(-pivotAngle);
      shooter.setSpeed(speed);

      if (pivot.isAtGoal()
          && shooter.isAtSpeed(speed)
          && Math.abs((Math.toDegrees(turnAngle))) < 3) {
        shooter.feedOut();
      }
    } else {
      if (isStillTracking) {
        isStillTracking = false;
        leds.setTracking(false);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    pivot.setRequestedAngle(20);
  }

  @Override
  public boolean isFinished() {
    return !shooter.hasGamePiece();
  }
}
