package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class IntakeCommand extends Command {
  private final Intake intake;
  private final Pivot pivot;
  private final Shooter shooter;

  public IntakeCommand(Intake intake, Pivot pivot, Shooter shooter) {
    this.intake = intake;
    this.pivot = pivot;
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
    this.intake.start();
    this.pivot.setRequestedAngle(90);
    this.shooter.startFeed();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    this.intake.stop();
    this.pivot.setRequestedAngle(20);
    this.shooter.stopFeed();
  }

  @Override
  public boolean isFinished() {
    return this.shooter.isFrontBeamTripped();
  }
}
