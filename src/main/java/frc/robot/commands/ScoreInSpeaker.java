package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
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
  private final RobotState state;
  private AprilTag tag;

  public ScoreInSpeaker(
      Drive drive, Shooter shooter, Pivot pivot, AprilTagDetector aprilTagDetector) {
    this.drive = drive;
    this.shooter = shooter;
    this.pivot = pivot;
    this.aprilTagDetector = aprilTagDetector;
    this.state = RobotState.getInstance();

    addRequirements(shooter, pivot);
  }

  @Override
  public void initialize() {
    // get the tag we need to be scoring on
    tag = aprilTagDetector.getTag(DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7);

    pivot.setRequestedAngle(30);
    shooter.setSpeed(4000);
  }

  @Override
  public void execute() {
    final var timestamp = Timer.getFPGATimestamp();

    var z = tag.z;
    var x = tag.x;

    // if we have historical poses, use them to correct the vector
    // based on the difference between current and historic poses
    if (state.hasSampledPoses()) {
      final var poseThen = state.getPoseAtTimestamp(tag.timestamp);
      final var poseNow = state.getPoseAtTimestamp(timestamp);
      final var translationThen = poseThen.getTranslation();
      final var translationNow = poseNow.getTranslation();
      final var rotationThen = poseThen.getRotation();
      final var rotationNow = poseNow.getRotation();

      // convert from vision frame of reference to field frame of reference
      final var fieldTranslation = new Translation2d(-z, x).rotateBy(rotationThen);
      final var dx = translationNow.getX() - translationThen.getX();
      final var dy = translationNow.getY() - translationThen.getY();

      // vector to the vision target in field frame of reference
      final var targetVector =
          new Translation2d(fieldTranslation.getX() + dx, fieldTranslation.getY() + dy);

      Logger.recordOutput("TrackedShooting/Target", translationNow.plus(targetVector));

      // convert target vector back to the vision frame of reference
      final var t = targetVector.rotateBy(rotationNow.unaryMinus());
      x = t.getY();
      z = -t.getX();
    }

    final var pivotAngle = Math.toDegrees(Math.atan2(-2.3, z + 0.1));
    Logger.recordOutput("TrackedShooting/PivotAngle", pivotAngle);

    final var turnAngle = Math.atan2(x, z);
    Logger.recordOutput("TrackedShooting/TurnAngle", Math.toDegrees(turnAngle));

    final var speed = z > 4.0 ? 4000 : 3000;
    Logger.recordOutput("TrackedShooting/Speed", speed);

    // consider tag acquired if we see one or we have seen it within the last 2 seconds
    // so that we can use recent odometry compensation to steer us towards it
    final var targetAcquired = tag.tracked || (timestamp - tag.timestamp) < 2;
    if (targetAcquired) {
      drive.steeringLock(turnAngle);

      pivot.setRequestedAngle(-pivotAngle);
      shooter.setSpeed(speed);

      if (pivot.isAtGoal()
          && shooter.isAtSpeed(speed)
          && Math.abs((Math.toDegrees(turnAngle))) < 3) {
        shooter.feedOut();
      }
    } else {
      drive.steeringUnlock();
    }

    // runs active steering if needed in autonomous mode
    if (DriverStation.isAutonomous()) {
      drive.runVelocity(new ChassisSpeeds());
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.steeringUnlock();
    shooter.stop();
    pivot.setRequestedAngle(20);
  }

  @Override
  public boolean isFinished() {
    return !shooter.hasGamePiece();
  }
}
