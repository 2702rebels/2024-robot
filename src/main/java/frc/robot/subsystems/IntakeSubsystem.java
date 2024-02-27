// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax m_intakeMotorController =
      new CANSparkMax(IntakeConstants.MotorControllerCANId, MotorType.kBrushless);
  private RelativeEncoder m_encoder;

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    SmartDashboard.putNumber(getName().concat("/IntakePercentSpeed"), 0.7);
    SmartDashboard.putNumber(getName().concat("/OuttakePercentSpeed"), 0.7);
    SmartDashboard.putNumber(getName().concat("/MeasuredSpeed"), 0);

    m_intakeMotorController.restoreFactoryDefaults();
    m_intakeMotorController.setSmartCurrentLimit(30);
    m_intakeMotorController.setIdleMode(IdleMode.kCoast);
    m_intakeMotorController.setInverted(true);
    m_intakeMotorController.burnFlash();

    m_encoder = m_intakeMotorController.getEncoder();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(getName().concat("/MeasuredSpeed"), m_encoder.getVelocity());
  }

  public Command stop() {
    return new InstantCommand(() -> m_intakeMotorController.stopMotor());
  }

  public Command start() {
    return new InstantCommand(() -> startpercent());
  }

  public Command reverse() {
    return new InstantCommand(() -> reversePercent());
  }

  private void startpercent() {
    double speed = SmartDashboard.getNumber(getName().concat("/IntakePercentSpeed"), 0);
    m_intakeMotorController.set(speed);
  }

  private void reversePercent() {
    double speed = SmartDashboard.getNumber(getName().concat("/OuttakePercentSpeed"), 0);
    m_intakeMotorController.set(speed * -1.0);
  }
}
