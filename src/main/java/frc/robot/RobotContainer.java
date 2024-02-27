// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  ShooterSubsystem m_shooter = new ShooterSubsystem();
  private CommandXboxController m_driverController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //Command shoot = Commands.sequence(Commands.runOnce(() -> {m_shooter.runFlywheel();}, m_shooter), Commands.waitSeconds(2), Commands.runOnce(() -> {m_shooter.runFeed();}));
    Command shoot2 = Commands.runOnce(() -> {m_shooter.runFlywheel();}, m_shooter).andThen(Commands.waitSeconds(1)).andThen(() -> {m_shooter.runFeed();}, m_shooter);
    Command feedTillSensor = Commands.startEnd(() -> {m_shooter.runFeed();}, () -> {m_shooter.stopFeed();}, m_shooter).until(() -> { return m_shooter.isGamePeice();});
    
    //Commands.runOnce(() -> {m_shooter.runFeed();}, m_shooter).(null).andThen(() -> {m_shooter.stopFeed();}, m_shooter);

    m_driverController.rightTrigger()
      .onTrue(shoot2)
      .onFalse(new InstantCommand(() -> {m_shooter.stop();}, m_shooter));

    m_driverController.a().whileTrue(feedTillSensor);
  }

  public void onDisable() {
    m_shooter.stop();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
