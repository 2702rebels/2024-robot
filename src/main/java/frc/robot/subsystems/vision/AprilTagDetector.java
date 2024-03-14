package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.AprilTagDetectorIO.AprilTagDetectorIOInputs;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class AprilTagDetector extends SubsystemBase {
  private static final LoggedTunableNumber timeTolerance =
      new LoggedTunableNumber("Vision/AprilTag/Time Tolerance", 50);

  private final AprilTagDetectorIO io;
  private final AprilTagDetectorIOInputs inputs = new AprilTagDetectorIOInputs();

  private final AprilTag[] tags = new AprilTag[16]; // 16 april tags locations
  private final double cutoff = 1e15; // Vision sends -3e16

  public AprilTagDetector(AprilTagDetectorIO io) {
    this.io = io;

    // initialize tags
    for (var i = 0; i < tags.length; ++i) {
      tags[i] = new AprilTag(i + 1);
    }
  }

  public void periodic() {
    this.io.updateInputs(inputs);
    Logger.processInputs("Vision/AprilTag", inputs);

    // convert from milliseconds to seconds
    final var timeToleranceSeconds = timeTolerance.get() / 1e3;

    for (var i = 0; i < inputs.frames.length; ++i) {
      final var timestamp = inputs.timestamps[i];

      // array contains consecutive 3 tuples of the following format
      // [x, y, z]
      final var frame = inputs.frames[i];
      for (var j = 0; j < tags.length; ++j) {
        final var offset = j * 3;
        // incomplete frame (should not happen)
        if (offset + 2 >= frame.length) {
          break;
        }

        final var x = frame[offset];
        if (Math.abs(x) > cutoff) {
          // if the tag disappears keep the last known position for
          // the 'tolerance' window to avoid jitterring when the tag
          // come in and out of view
          if (timestamp - tags[j].timestamp < timeToleranceSeconds) {
            continue;
          }

          // keep the last known vector but set tracked to false
          tags[j].tracked = false;
          tags[j].timestamp = timestamp;
        } else {
          tags[j].tracked = true;
          tags[j].x = x;
          tags[j].y = frame[offset + 1];
          tags[j].z = frame[offset + 2];
          tags[j].timestamp = timestamp;
        }
      }
    }

    for (var i = 0; i < tags.length; ++i) {
      final var key = "Vision/AprilTags/" + tags[i].id;
      Logger.recordOutput(key + "/Tracked", tags[i].tracked);
      Logger.recordOutput(key + "/X", tags[i].x);
      Logger.recordOutput(key + "/Y", tags[i].y);
      Logger.recordOutput(key + "/Z", tags[i].z);
    }
  }
}
