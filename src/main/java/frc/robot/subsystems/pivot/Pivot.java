// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Pivot extends SubsystemBase {
  private static final double LOOP_PERIOD_SECS = 0.02;
  private final LoggedTunableNumber kp = new LoggedTunableNumber("Pivot/PID/P", 0.0);
  private final LoggedTunableNumber ki = new LoggedTunableNumber("Pivot/PID/I", 0.0);
  private final LoggedTunableNumber kd = new LoggedTunableNumber("Pivot/PID/D", 0.0);

  private final LoggedTunableNumber ks = new LoggedTunableNumber("Pivot/ArmFeedForward/KS", 0.0);
  private final LoggedTunableNumber kg = new LoggedTunableNumber("Pivot/ArmFeedForward/KG", 0.0);
  private final LoggedTunableNumber kv = new LoggedTunableNumber("Pivot/ArmFeedForward/KV", 0.0);
  private final LoggedTunableNumber ka = new LoggedTunableNumber("Pivot/ArmFeedForward/KA", 0.0);

  private final LoggedTunableNumber kmv =
      new LoggedTunableNumber("Pivot/TrapezoidProfile/MaxVelocity", 0.0);
  private final LoggedTunableNumber kma =
      new LoggedTunableNumber("Pivot/TrapezoidProfile/MaxAcceleration", 0.0);

  private final LoggedDashboardNumber requestedPos =
      new LoggedDashboardNumber("Pivot/RequestedPos");

  private final LoggedDashboardNumber statePos = new LoggedDashboardNumber("Pivot/StatePosition");
  private final LoggedDashboardNumber stateVel = new LoggedDashboardNumber("Pivot/StateVelocity");
  private final LoggedDashboardNumber feedforward =
      new LoggedDashboardNumber("Pivot/ArmFeedForward");

  private ArmFeedforward armFeedforward;
  private TrapezoidProfile trapezoidProfile;

  private TrapezoidProfile.State state = new State(0, 0);
  private TrapezoidProfile.State goal = new State(0, 0);

  private boolean enable = false;

  private final PivotIO pivotMotor;
  private final PivotIOInputsAutoLogged motorInputs = new PivotIOInputsAutoLogged();

  /** Creates a new Pivot. */
  public Pivot(PivotIO pivot) {
    pivotMotor = pivot;
    pivotMotor.configureJointPID(kp.get(), ki.get(), kd.get());
    requestedPos.set(0);
    armFeedforward = new ArmFeedforward(ks.get(), kg.get(), kv.get(), ka.get());
    trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(kmv.get(), kma.get()));
  }

  @Override
  public void periodic() {
    pivotMotor.updateInputs(motorInputs);
    Logger.processInputs("Pivot/Motor", motorInputs);

    final var callerId = hashCode();
    if (kp.hasChanged(callerId) || ki.hasChanged(callerId) || kd.hasChanged(callerId)) {
      pivotMotor.configureJointPID(kp.get(), ki.get(), kd.get());
    }

    if (ks.hasChanged(callerId)
        || kg.hasChanged(callerId)
        || kv.hasChanged(callerId)
        || ka.hasChanged(callerId)) {
      armFeedforward = new ArmFeedforward(ks.get(), kg.get(), kv.get(), ka.get());
    }

    if (kmv.hasChanged(callerId) || kma.hasChanged(callerId)) {
      trapezoidProfile =
          new TrapezoidProfile(new TrapezoidProfile.Constraints(kmv.get(), kma.get()));
    }

    goal.position = requestedPos.get();
    state.position = motorInputs.measuredPosition.getRadians();
    state.velocity = motorInputs.measuredVelocityRadPerSec;
    state =
        trapezoidProfile.calculate(
            LOOP_PERIOD_SECS,
            state,
            goal); // This calulate method is doing at least two new allocaitons...
    double ff = armFeedforward.calculate(state.position, state.velocity);

    // Update dashboard values
    statePos.set(state.position);
    stateVel.set(state.velocity);
    feedforward.set(ff);

    if (enable) {
      pivotMotor.setJointAngle(state.position, ff);
    } else {
      pivotMotor.stop();
    }
  }

  /** Enables the pid while command is running. Stops it when command ends. */
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
