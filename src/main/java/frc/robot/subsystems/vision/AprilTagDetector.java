package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AprilTagDetector extends SubsystemBase {
  // private static final LoggedTunableNumber timeTolerance =
  //     new LoggedTunableNumber("Vision/AprilTag/Time Tolerance", 150);

  private static final double[] EmptyDoubleArray = new double[] {};
  private final DoubleArraySubscriber subscriber;

  private final AprilTag[] tags = new AprilTag[16]; // 16 april tags locations
  private final double cutoff = 1e15; // Vision sends -3e16

  public AprilTagDetector() {
    subscriber =
        NetworkTableInstance.getDefault()
            .getTable("Vision")
            .getDoubleArrayTopic("PoseArray")
            .subscribe(
                EmptyDoubleArray, PubSubOption.keepDuplicates(false), PubSubOption.sendAll(true));

    // initialize tags
    for (var i = 0; i < tags.length; ++i) {
      tags[i] = new AprilTag(i + 1);
    }

    SmartDashboard.putNumber("/Vision/Exposure", 480);
  }

  public void periodic() {
    // convert from milliseconds to seconds
    // final var timeToleranceSeconds = timeTolerance.get() / 1e3;

    final var queue = subscriber.readQueue();
    if (queue.length > 0) {
      for (var i = 0; i < queue.length; ++i) {
        final var timestamp = queue[i].timestamp / 1E6;
        final var frame = queue[i].value;

        // array contains consecutive 3 tuples of the following format
        // [x, y, z]
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
            // if (timestamp - tags[j].timestamp < timeToleranceSeconds) {
            //   continue;
            // }

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
    }

    for (var i = 0; i < tags.length; ++i) {
      final var key = "Vision/AprilTags/" + tags[i].id;
      Logger.recordOutput(key + "/Tracked", tags[i].tracked);
      Logger.recordOutput(key + "/X", tags[i].x);
      Logger.recordOutput(key + "/Y", tags[i].y);
      Logger.recordOutput(key + "/Z", tags[i].z);
    }
  }

  public AprilTag getTag(int id) {
    return tags[id - 1];
  }
}
