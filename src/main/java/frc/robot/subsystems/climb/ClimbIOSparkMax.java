// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.CanID;
import frc.robot.Constants;

/** Add your docs here. */
public class ClimbIOSparkMax implements ClimbIO {
  private static final int CLIMB_SERVO = 0;
  private CANSparkMax motor = new CANSparkMax(CanID.MOTOR_CLIMB, MotorType.kBrushless);
  private RelativeEncoder encoder = motor.getEncoder();
  private SparkPIDController pid = motor.getPIDController();
  // private AnalogOutput ratchet = new AnalogOutput(0)
  private Servo ratchet = new Servo(CLIMB_SERVO);

  private DigitalInput limitSwitchClosed = new DigitalInput(Constants.CLIMB_LIMITSWITCH);

  public ClimbIOSparkMax() {
    // Configure and save settings to leader motor controller.
    motor.restoreFactoryDefaults();
    motor.setSmartCurrentLimit(60);
    motor.setIdleMode(IdleMode.kCoast);
    motor.burnFlash();
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.appliedVolts = motor.getBusVoltage();
    inputs.limitSwitchClosed = limitSwitchClosed.get();
  }

  @Override
  public void zeroEncoder() {
    encoder.setPosition(0);
  }

  @Override
  public void setPercentOutput(double speed) {
    motor.set(speed);
  }

  @Override
  public void setPosition(double position) {
    pid.setReference(position, ControlType.kPosition);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
  }

  public void setServoPosition(double degrees) {
    ratchet.setAngle(degrees);
  }
}
