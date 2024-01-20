// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax m_intakeMotorController = new CANSparkMax(IntakeConstants.MotorControllerCANId, MotorType.kBrushless);
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    SmartDashboard.putNumber(getName().concat("/CommandedSpeed"), 3000);
    SmartDashboard.putNumber(getName().concat("/MeasuredSpeed"), 0);

    m_pidController = m_intakeMotorController.getPIDController();
    m_pidController.setP(0.1);
    m_pidController.setI(0);
    m_pidController.setD(0);
    
    m_encoder = m_intakeMotorController.getEncoder();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(getName().concat("/MeasuredSpeed"), m_encoder.getVelocity());
  }

  public Command stop() {
    return new InstantCommand(()->m_intakeMotorController.stopMotor());
  }

  public Command start() {
    return new InstantCommand(()->startPID());
  }

  private void startPID() {
    double rpm = SmartDashboard.getNumber(getName().concat("/CommandedSpeed"), 0);
    m_pidController.setReference(rpm, ControlType.kVelocity);
  }
}
