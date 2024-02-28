// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax m_flywheelLeader =
      new CANSparkMax(ShooterConstants.kShooterFlywheel1, MotorType.kBrushless);
  private CANSparkMax m_flywheelFollower =
      new CANSparkMax(ShooterConstants.kShooterFlywheel2, MotorType.kBrushless);
  private CANSparkMax m_feederMotorController =
      new CANSparkMax(ShooterConstants.kShooterFeed, MotorType.kBrushless);
  private RelativeEncoder m_flywheelEncoder;
  private SparkPIDController m_flywheelPID;

  private double periodicrpmTarget = 0;

  private DigitalInput beambreak = new DigitalInput(8);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_flywheelEncoder = m_flywheelLeader.getEncoder();
    m_flywheelPID = m_flywheelLeader.getPIDController();

    // Configure and save settings to leader motor controller.
    m_flywheelLeader.restoreFactoryDefaults();
    m_flywheelLeader.setSmartCurrentLimit(60);
    m_flywheelLeader.setIdleMode(IdleMode.kCoast);

    m_flywheelPID.setP(0.0006, 0);
    m_flywheelPID.setI(0, 0);
    m_flywheelPID.setD(0, 0);
    m_flywheelPID.setFF(0.00017, 0);

    m_flywheelLeader.burnFlash();

    // Configure and save settings to follower motor controller.
    m_flywheelFollower.restoreFactoryDefaults();
    m_flywheelFollower.follow(m_flywheelLeader, true);
    m_flywheelFollower.setSmartCurrentLimit(60);
    m_flywheelFollower.setIdleMode(IdleMode.kCoast);
    m_flywheelFollower.burnFlash();

    // Configure and save settings to the feeder motor controller.
    m_feederMotorController.restoreFactoryDefaults();
    m_feederMotorController.setInverted(true);
    m_feederMotorController.setIdleMode(IdleMode.kBrake);
    m_feederMotorController.burnFlash();

    SmartDashboard.putNumber("ConfigFlywheelRPM", 3000);
    SmartDashboard.putNumber("ConfigFeedPercent", 0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double flyWheelRPM = (m_flywheelEncoder.getVelocity());
    SmartDashboard.putNumber("Flywheel Speed", flyWheelRPM);
    SmartDashboard.putBoolean("Beambreak", beambreak.get());

    // m_flywheelPID.setReference(periodicrpmTarget, ControlType.kVelocity);
  }

  public void runFlywheel() {
    double rpmTarget = SmartDashboard.getNumber("ConfigFlywheelRPM", 3000);
    // m_flywheelPID.setReference(rpmTarget, ControlType.kVelocity, 0);

    m_flywheelLeader.set(1.0);

    periodicrpmTarget = rpmTarget;

    System.out.println("Flywheel: Started with reference of: " + rpmTarget);
  }

  public void stopFlywheel() {
    m_flywheelLeader.stopMotor();
    periodicrpmTarget = 0;
    System.out.println("Flywheel: Stopped");
  }

  public void runFeed() {
    double feedPercent = SmartDashboard.getNumber("ConfigFeedPercent", 0.5);
    m_feederMotorController.set(feedPercent);
    System.out.println("Feed: Started with percent output of: " + feedPercent * 100 + "%");
  }

  public void stopFeed() {
    m_feederMotorController.stopMotor();
    System.out.println("Feed: Stopped");
  }

  public void stop() {
    stopFlywheel();
    stopFeed();
  }

  public boolean isGamePeice() {
    return !beambreak.get();
  }
}
