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
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.CanID;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class PivotTrapezoidal extends TrapezoidProfileSubsystem {
  private final CANSparkMax controller = new CANSparkMax(CanID.MOTOR_PIVOT, MotorType.kBrushless);
  private final SparkAbsoluteEncoder encoder = controller.getAbsoluteEncoder(Type.kDutyCycle);
  private final SparkPIDController pid = controller.getPIDController();
  private final ArmFeedforward armFeedforward;

  private final LoggedTunableNumber kp = new LoggedTunableNumber("Pivot/PID/P", 0.0001);
  private final LoggedTunableNumber ki = new LoggedTunableNumber("Pivot/PID/I", 0.0);
  private final LoggedTunableNumber kd = new LoggedTunableNumber("Pivot/PID/D", 0.0);

  private final LoggedTunableNumber ks = new LoggedTunableNumber("Pivot/ArmFeedForward/KS", 0.0);
  private final LoggedTunableNumber kg = new LoggedTunableNumber("Pivot/ArmFeedForward/KG", 0.0);
  private final LoggedTunableNumber kv = new LoggedTunableNumber("Pivot/ArmFeedForward/KV", 0.0);
  private final LoggedTunableNumber ka = new LoggedTunableNumber("Pivot/ArmFeedForward/KA", 0.0);

  private final LoggedDashboardNumber requestedPos =
      new LoggedDashboardNumber("Pivot/RequestedPos");
  private final LoggedDashboardNumber measuredPos = new LoggedDashboardNumber("Pivot/MeasuredPos");

  /** Creates a new PivotTrapezoidal. */
  public PivotTrapezoidal() {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(0, 0),
        // The initial position of the mechanism
        0);

    requestedPos.set(0);

    controller.restoreFactoryDefaults();
    controller.setIdleMode(IdleMode.kBrake);
    controller.setSmartCurrentLimit(40);
    controller.setInverted(true);

    pid.setP(kp.get(), 0);
    pid.setI(ki.get(), 0);
    pid.setD(kd.get(), 0);

    controller.burnFlash();

    armFeedforward = new ArmFeedforward(ks.get(), kg.get(), kv.get(), ka.get());
  }

  @Override
  protected void useState(TrapezoidProfile.State state) {
    // Use the computed profile state here.
    double ff = armFeedforward.calculate(state.position, state.velocity);
    pid.setReference(state.position, ControlType.kPosition, 0, ff);
    measuredPos.set(encoder.getPosition());
  }
}
