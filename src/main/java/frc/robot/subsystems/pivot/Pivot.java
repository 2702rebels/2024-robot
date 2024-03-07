// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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

  private final LoggedTunableNumber minPivotAngle =
      new LoggedTunableNumber("Pivot/Limit/MinAngle", 20.0);
  private final LoggedTunableNumber maxPivotAngle =
      new LoggedTunableNumber("Pivot/Limit/MaxAngle", 100.0);
  private final LoggedTunableNumber onGoalAllowance =
      new LoggedTunableNumber("Pivot/OnGoalAllowance", 1.0);

  public final LoggedDashboardNumber setPointIntake =
      new LoggedDashboardNumber("Pivot/SetPoints/Intake", 80);
  public final LoggedDashboardNumber setPointAmp =
      new LoggedDashboardNumber("Pivot/SetPoints/Amp", 100);
  public final LoggedDashboardNumber setPointIdle =
      new LoggedDashboardNumber("Pivot/SetPoints/Idle", 50);

  private ArmFeedforward armFeedforward;
  private TrapezoidProfile trapezoidProfile;

  private TrapezoidProfile.State state = new State(0, 0);
  private TrapezoidProfile.State goal = new State(0, 0);

  private boolean enable = false;

  private final PivotIO pivotMotor;
  private final PivotIOInputsAutoLogged motorInputs = new PivotIOInputsAutoLogged();

  private double requestedPosInDeg = 0;

  /** Creates a new Pivot. */
  public Pivot(PivotIO pivot) {
    pivotMotor = pivot;
    pivotMotor.configureJointPID(kp.get(), ki.get(), kd.get());
    armFeedforward = new ArmFeedforward(ks.get(), kg.get(), kv.get(), ka.get());
    trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(kmv.get(), kma.get()));
  }

  @Override
  public void periodic() {
    pivotMotor.updateInputs(motorInputs);
    Logger.processInputs("Pivot/Motor", motorInputs);

    final var callerId = hashCode();
    if (kp.hasChanged(callerId) || ki.hasChanged(callerId) || kd.hasChanged(callerId)) {
      System.out.println("Pivot PID parameters have changed. Reconfiguring them.");
      pivotMotor.configureJointPID(kp.get(), ki.get(), kd.get());
    }

    if (ks.hasChanged(callerId)
        || kg.hasChanged(callerId)
        || kv.hasChanged(callerId)
        || ka.hasChanged(callerId)) {
      System.out.println("Pivot ArmFeedForward parameters have changed. Reconfiguring them.");
      armFeedforward = new ArmFeedforward(ks.get(), kg.get(), kv.get(), ka.get());
    }

    if (kmv.hasChanged(callerId) || kma.hasChanged(callerId)) {
      System.out.println("Pivot TrapezoidProfile parameters have changed. Reconfiguring them.");
      trapezoidProfile =
          new TrapezoidProfile(new TrapezoidProfile.Constraints(kmv.get(), kma.get()));
    }

    // Convert from degrees to radians.
    goal.position = requestedPosInDeg * (Math.PI / 180);
    state.position = motorInputs.measuredPositionRadians;
    state.velocity = motorInputs.measuredVelocityRadiansPerSecond;
    state =
        trapezoidProfile.calculate(
            LOOP_PERIOD_SECS,
            state,
            goal); // This calulate method is doing at least two new allocaitons...
    double ff = armFeedforward.calculate(state.position, state.velocity);

    Logger.recordOutput("Pivot/RequestedPosition", requestedPosInDeg);
    Logger.recordOutput("Pivot/StatePosition", state.position);
    Logger.recordOutput("Pivot/StateVelocity", state.velocity);
    Logger.recordOutput("Pivot/ArmFeedForward", ff);
    Logger.recordOutput("Pivot/PositionRadians", motorInputs.measuredPositionRadians);
    Logger.recordOutput(
        "Pivot/PositionAngle", motorInputs.measuredPositionRadians * (180 / Math.PI));

    if (enable) {
      pivotMotor.setJointAngle(state.position, ff);
    } else {
      pivotMotor.stop();
    }
  }

  /** Sets the requested angle for the pivot's control loop. */
  private void setRequestedAngle(double angle) {
    if (angle < minPivotAngle.get()) {
      requestedPosInDeg = minPivotAngle.get();
    } else if (angle > maxPivotAngle.get()) {
      requestedPosInDeg = maxPivotAngle.get();
    } else {
      requestedPosInDeg = angle;
    }
    System.out.println(
        "Pivot Requested angle of: " + angle + ". Setting angle to: " + requestedPosInDeg + ".");
  }

  /** Returns the pivot's current angle. */
  public double getCurrentAngle() {
    return motorInputs.measuredPositionRadians * (180 / Math.PI);
  }

  public boolean isAtGoal() {
    double angle = getCurrentAngle();
    if (angle < requestedPosInDeg + onGoalAllowance.get()
        && angle > requestedPosInDeg - onGoalAllowance.get()) {
      return true;
    }
    return false;
  }

  /** Enables the control loop while command is running. Stops it when command ends. */
  public Command enableControlLoopWhileTrue() {
    return Commands.startEnd(
        () -> {
          enable = true;
        },
        () -> {
          enable = false;
        },
        this);
  }

  /** Enables the control loop. */
  public Command enableControlLoop() {
    return Commands.runOnce(
        () -> {
          enable = true;
        },
        this);
  }

  /** Disables the control loop. */
  public Command disableControlLoop() {
    return Commands.runOnce(
        () -> {
          enable = false;
        },
        this);
  }

  /** Commands the arm to a position, ending the command when it arrives. */
  public Command moveArmToPosition(double angle) {
    return new FunctionalCommand(
        () -> {
          setRequestedAngle(angle);
        },
        () -> {},
        (interupted) -> {},
        () -> {
          return isAtGoal();
        },
        this);
  }
}
