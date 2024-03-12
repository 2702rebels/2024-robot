// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.CanID;
import frc.robot.Constants;

/** Add your docs here. */
public class ShooterIOSparkMaxes implements ShooterIO {
  private CANSparkMax flywheelLeader =
      new CANSparkMax(CanID.MOTOR_SHOOTER_LEADER, MotorType.kBrushless);
  private CANSparkMax flywheelFollower =
      new CANSparkMax(CanID.MOTOR_SHOOTER_FOLLOWER, MotorType.kBrushless);
  private CANSparkMax feederMotorController =
      new CANSparkMax(CanID.MOTOR_SHOOTER_FEED, MotorType.kBrushless);
  private RelativeEncoder flywheelEncoder = flywheelLeader.getEncoder();
  private RelativeEncoder followerEncoder = flywheelFollower.getEncoder();
  private RelativeEncoder feedEncoder = feederMotorController.getEncoder();
  private SparkPIDController flywheelPID = flywheelLeader.getPIDController();
  private SparkPIDController feedPID = feederMotorController.getPIDController();

  private DigitalInput beambreak = new DigitalInput(Constants.SHOOTER_BEAMBREAK);

  public ShooterIOSparkMaxes() {
    // Configure and save settings to leader motor controller.
    flywheelLeader.restoreFactoryDefaults();
    flywheelLeader.setSmartCurrentLimit(60);
    flywheelLeader.setIdleMode(IdleMode.kCoast);
    flywheelLeader.enableVoltageCompensation(12);
    flywheelLeader.burnFlash();

    // Configure and save settings to follower motor controller.
    flywheelFollower.restoreFactoryDefaults();
    flywheelFollower.follow(flywheelLeader, true);
    flywheelFollower.setSmartCurrentLimit(60);
    flywheelFollower.setIdleMode(IdleMode.kCoast);
    flywheelPID.setOutputRange(
        0, 1.0); // Set this so it can't apply negative power to try and pull the flywheel down to
    // speed.
    flywheelFollower.burnFlash();

    // Configure and save settings to the feeder motor controller.
    feederMotorController.restoreFactoryDefaults();
    feederMotorController.setInverted(true);
    feederMotorController.setIdleMode(IdleMode.kBrake);
    feedPID.setOutputRange(0, 1.0);
    feederMotorController.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.flywheelVelocity = flywheelEncoder.getVelocity();
    inputs.flywheelPosition = flywheelEncoder.getPosition();
    inputs.flywheelAppliedVolts =
        flywheelLeader.getAppliedOutput() * flywheelLeader.getBusVoltage();
    inputs.flywheelCurrentAmps = flywheelLeader.getOutputCurrent();

    inputs.followerVelocity = followerEncoder.getVelocity();
    inputs.followerAppliedVolts =
        flywheelFollower.getAppliedOutput() * flywheelFollower.getBusVoltage();
    inputs.followerCurrentAmps = flywheelFollower.getOutputCurrent();

    inputs.feedVelocity = feedEncoder.getVelocity();
    inputs.feedPosition = feedEncoder.getPosition();
    inputs.feedAppliedVolts =
        feederMotorController.getAppliedOutput() * feederMotorController.getBusVoltage();
    inputs.feedCurrentAmps = feederMotorController.getOutputCurrent();

    inputs.feedBeamBreak = !beambreak.get();
  }

  @Override
  public void configureFlywheelPID(double kP, double kI, double kD, double kff, double staticGain) {
    flywheelPID.setP(kP);
    flywheelPID.setI(kI);
    flywheelPID.setD(kD);
    flywheelPID.setFF(kff);
  }

  @Override
  public void setFlywheelSpeed(double rpm) {
    flywheelPID.setReference(rpm, ControlType.kVelocity);
    System.out.println("Setting flywheel speed to: " + rpm);
  }

  @Override
  public void stopFlywheel() {
    flywheelLeader.stopMotor();
  }

  @Override
  public void configureFeedPID(double kP, double kI, double kD, double kFF) {
    feedPID.setP(kP);
    feedPID.setI(kI);
    feedPID.setD(kD);
    feedPID.setFF(kFF);
  }

  @Override
  public void setFeedSpeed(double rpm) {
    final var ff = 0.2265; // + rpm * 0.0022
    feedPID.setReference(rpm, ControlType.kVelocity, 0, ff);
  }

  @Override
  public void setFeedPercentOutput(double percent) {
    feederMotorController.set(percent);
  }

  @Override
  public void stopFeed() {
    feederMotorController.stopMotor();
  }

  @Override
  public void setFeedBrakeMode(boolean isBrake) {
    if (isBrake) {
      feederMotorController.setIdleMode(IdleMode.kBrake);
    } else {
      feederMotorController.setIdleMode(IdleMode.kCoast);
    }
  }

  @Override
  public void setVoltage(double volts) {
    flywheelLeader.setVoltage(volts);
  }
}
