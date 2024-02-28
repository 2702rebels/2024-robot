// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.SparkMaxBurnManager;

/** Add your docs here. */
public class PivotIOSparkMax implements PivotIO {
  private final CANSparkMax sparkMax;
  private final SparkAbsoluteEncoder absolutEncoder;
  private final SparkPIDController pid;

  public PivotIOSparkMax(int canID) {
    sparkMax = new CANSparkMax(canID, MotorType.kBrushless);
    absolutEncoder = sparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    pid = sparkMax.getPIDController();

    sparkMax.setCANTimeout(SparkMaxBurnManager.configCANTimeout);
    if (SparkMaxBurnManager.shouldBurn()) {
      sparkMax.restoreFactoryDefaults();
    }

    // Configure controller.
    sparkMax.setIdleMode(IdleMode.kBrake);
    sparkMax.setSmartCurrentLimit(20);
    sparkMax.setInverted(true);

    // Configure encoder.
    // absolutEncoder.setAverageDepth(2);
    absolutEncoder.setInverted(false);
    // absolutEncoder.setPositionConversionFactor(2 * Math.PI);

    // Configure PID.
    pid.setFeedbackDevice(absolutEncoder);

    sparkMax.setCANTimeout(0);
    if (SparkMaxBurnManager.shouldBurn()) {
      sparkMax.burnFlash();
    }
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.measuredPosition = Rotation2d.fromRadians(absolutEncoder.getPosition());
    inputs.measuredVelocityRadPerSec = absolutEncoder.getVelocity();
    inputs.appliedVolts = sparkMax.getBusVoltage();
    inputs.currentAmps = sparkMax.getOutputCurrent();
  }

  @Override
  public void stop() {
    sparkMax.stopMotor();
  }

  @Override
  public void configureJointPID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
  }

  @Override
  public void setJointAngle(double angleRadians) {
    pid.setReference(angleRadians, ControlType.kPosition, 0);
  }

  @Override
  public void setJointAngle(double angleRadians, double ff) {
    pid.setReference(angleRadians, ControlType.kPosition, 0, ff);
  }
}
