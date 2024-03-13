// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.CanID;
import frc.robot.Constants;

/** Add your docs here. */
public class IntakeIOSparkMax implements IntakeIO {
  private CANSparkMax motor = new CANSparkMax(CanID.MOTOR_INTAKE, MotorType.kBrushless);
  private RelativeEncoder encoder = motor.getEncoder();
  private DigitalInput beambreak = new DigitalInput(Constants.SHOOTER_BEAMBREAKINTAKE);

  public IntakeIOSparkMax() {
    motor.restoreFactoryDefaults();
    motor.setSmartCurrentLimit(60);
    motor.setIdleMode(IdleMode.kCoast);
    motor.setInverted(false);
    motor.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocityRPM = encoder.getVelocity();
    inputs.appliedVolts = motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.beamBreak = !beambreak.get();
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
