// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.AprilTag;
import frc.robot.subsystems.vision.AprilTagDetector;
import org.littletonrobotics.junction.Logger;

public class ScoreInSpeaker extends Command {
  private Drive drive;
  private Shooter shooter;
  private Pivot pivot;
  private AprilTagDetector aprilTagDetector;
  private AprilTag tag;

  /** Creates a new ScoreInSpeaker. */
  public ScoreInSpeaker(
      Drive drive, Shooter shooter, Pivot pivot, AprilTagDetector aprilTagDetector) {
    this.drive = drive;
    this.shooter = shooter;
    this.pivot = pivot;
    this.aprilTagDetector = aprilTagDetector;

    addRequirements(shooter, pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Get the tag we need to be scoring on.
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      tag = aprilTagDetector.getTag(7);
    } else {
      tag = aprilTagDetector.getTag(3); // ?
    }

    shooter.spinUp(4000);
    pivot.setRequestedAngle(30);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final var pivotAngle = Math.atan2(-2.3, tag.z) * (180 / Math.PI);
    Logger.recordOutput("TrackedShooting/PivotAngle", pivotAngle);

    final var turnAngle = Math.atan2(tag.x, tag.z);
    Logger.recordOutput("TrackedShooting/TurnAngle", turnAngle);

    if (tag.tracked) {
      pivot.setRequestedAngle(-pivotAngle);
      drive.enableOrientationLock(drive.getRotation().getRadians() - turnAngle);

      if (pivot.isAtGoal()
          && shooter.isAtSpeed(4000)
          && Math.abs((turnAngle * 180 / Math.PI)) < 3) {
        shooter.feedOut();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    pivot.setRequestedAngle(20);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return !shooter.hasGamePiece();
  }
}
