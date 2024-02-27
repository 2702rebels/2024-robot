// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CanID;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Pivot extends SubsystemBase {
  private final LoggedTunableNumber kp = new LoggedTunableNumber("Pivot/PID/P", 0.0);
  private final LoggedTunableNumber ki = new LoggedTunableNumber("Pivot/PID/I", 0.0);
  private final LoggedTunableNumber kd = new LoggedTunableNumber("Pivot/PID/D", 0.0);

  private final LoggedTunableNumber ks = new LoggedTunableNumber("Pivot/ArmFeedForward/KS", 0.0);
  private final LoggedTunableNumber kg = new LoggedTunableNumber("Pivot/ArmFeedForward/KG", 0.0);
  private final LoggedTunableNumber kv = new LoggedTunableNumber("Pivot/ArmFeedForward/KV", 0.0);
  private final LoggedTunableNumber ka = new LoggedTunableNumber("Pivot/ArmFeedForward/KA", 0.0);

  private final LoggedTunableNumber kmv =
      new LoggedTunableNumber("Pivot/ArmFeedForward/MaxVelocity", 0.0);
  private final LoggedTunableNumber kma =
      new LoggedTunableNumber("Pivot/ArmFeedForward/MaxAcceleration", 0.0);

  private final LoggedDashboardNumber requestedPos =
      new LoggedDashboardNumber("Pivot/RequestedPos");
  private final LoggedDashboardNumber measuredPos = new LoggedDashboardNumber("Pivot/MeasuredPos");

  private final LoggedDashboardNumber statePos = new LoggedDashboardNumber("Pivot/StatePosition");
  private final LoggedDashboardNumber stateVel = new LoggedDashboardNumber("Pivot/StateVelocity");

  private final CANSparkMax controller = new CANSparkMax(CanID.MOTOR_PIVOT, MotorType.kBrushless);
  private final SparkAbsoluteEncoder encoder = controller.getAbsoluteEncoder(Type.kDutyCycle);
  private final SparkPIDController pid = controller.getPIDController();
  private final ArmFeedforward armFeedforward;
  private final TrapezoidProfile trapezoidProfile;

  private TrapezoidProfile.State state = new State(0, 0);
  private TrapezoidProfile.State goal = new State(0, 0);

  private boolean enable = false;

  /** Creates a new Pivot. */
  public Pivot() {

    requestedPos.set(0);

    // controller.restoreFactoryDefaults();
    controller.setIdleMode(IdleMode.kBrake);
    controller.setSmartCurrentLimit(20);
    controller.setInverted(true);

    pid.setP(kp.get(), 0);
    pid.setI(ki.get(), 0);
    pid.setD(kd.get(), 0);

    controller.burnFlash();

    armFeedforward = new ArmFeedforward(ks.get(), kg.get(), kv.get(), ka.get());
    trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(kmv.get(), kma.get()));
  }

  @Override
  public void periodic() {
    // Update dashboard values
    measuredPos.set(encoder.getPosition());
    statePos.set(state.position);
    statePos.set(state.velocity);

    if (enable) {
      goal.position = requestedPos.get();
      state =
          trapezoidProfile.calculate(
              0.02, state, goal); // This calulate method is doing at least two new allocaitons...
      double ff =
          armFeedforward.calculate(
              state.position,
              state.velocity); // Should log these values for playback. Also the measured encoder
      // values.
      pid.setReference(state.position, ControlType.kPosition, 0, ff);

    } else {
      controller.stopMotor();
    }
  }

  public Command enableControl() {
    return Commands.startEnd(
        () -> {
          enable = true;
        },
        () -> {
          enable = false;
        },
        this);
  }

  public Command disableControl() {
    return Commands.runOnce(
        () -> {
          enable = false;
        },
        this);
  }
}
