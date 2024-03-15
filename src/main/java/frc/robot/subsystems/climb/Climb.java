// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Climb extends SubsystemBase {
  private final ClimbIO subsystemio;
  private final ClimbIOInputsAutoLogged subsystemInputs = new ClimbIOInputsAutoLogged();
  private final LoggedTunableNumber ratchetOnAngle =
      new LoggedTunableNumber("Climb/ratchetOnAngle", 0);
  private final LoggedTunableNumber ratchetOffAngle =
      new LoggedTunableNumber("Climb/ratchetOffAngle", 90);
  private final LoggedTunableNumber hookTopPosition =
      new LoggedTunableNumber("Climb/hookTopPosition", 100);
  private final LoggedTunableNumber homeSpeed = new LoggedTunableNumber("Climb/homeSpeed", 100);

  /** Creates a new Climb. */
  public Climb(ClimbIO climb) {
    subsystemio = climb;

    // Do we need to config climb PID?
    // subsystemIO.configurePID(kp.get(), ki.get(), kd.get());

    // Engage ratchet
    subsystemio.setServoPosition(ratchetOnAngle.get());
  }

  @Override
  public void periodic() {
    subsystemio.updateInputs(subsystemInputs);
  }

  // Run climb motor until the limit switch closes, then reset encoder
  public Command homeHooks() {
    return new StartEndCommand(
            // Turn on climb motor at command start
            () -> {
              // Make sure ratchet is engaged
              subsystemio.setServoPosition(ratchetOnAngle.get());
              // Run climb motor
              subsystemio.setPercentOutput(homeSpeed.get());
            },
            // Turn off climb motor at command end
            () -> {
              subsystemio.setPercentOutput(0);
            },
            this)
        .until(
            // Stop climb motor when limit switch closes
            () -> {
              if (subsystemInputs.limitSwitchClosed) {
                subsystemio.zeroEncoder();
                return (true);
              } else {
                return (false);
              }
            });
  }

  // Run climb motor until we reach the home position or the limit switch closes
  public Command retractHooks() {
    return new StartEndCommand(
            // Turn on climb motor at command start
            () -> {
              // Go to home position
              subsystemio.setPosition(0);
            },
            // Turn off climb motor at command end
            () -> {
              subsystemio.setPercentOutput(0);
            },
            this)
        .until(
            // Stop climb motor when limit switch closes
            () -> {
              return subsystemInputs.limitSwitchClosed;
            });
  }

  // Run climb motor until we reach the top position
  public Command raiseHooks() {
    return new StartEndCommand(
        // Reverse climb motor at command start
        () -> {
          // Go to top position
          subsystemio.setServoPosition(ratchetOffAngle.get()); // release ratchet
          subsystemio.setPosition(hookTopPosition.get());
        },
        // Turn off climb motor at command end
        () -> {
          subsystemio.setPercentOutput(0);
          subsystemio.setServoPosition(ratchetOnAngle.get()); // Re-engage ratchet
        },
        this);
  }

  /*
  // Engage or release ratchet by turning servo
  //   - do this as part of the other commands?
  public Command setRatchet(BooleanSupplier engage) {
    return new StartEndCommand(
            () -> {
              if (engage)
              {
                subsystemio.setServoPosition(ratchetOnAngle.get());
              }
              else
              {
                subsystemio.setServoPosition(ratchetOffAngle.get());
              }
            },
            () -> {
              // Do nothing on end
            },
            this);
  }
  */
}
