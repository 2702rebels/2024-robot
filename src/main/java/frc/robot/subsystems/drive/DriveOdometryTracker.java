package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/** Maintains shared queues for high-frequency odometry readings. */
public class DriveOdometryTracker {
  private static final int maxQueueCapacity = 16;

  private final Lock lock = new ReentrantLock();
  private final List<Queue<Double>> queues = new ArrayList<>();
  private final List<DoubleSupplier> signals = new ArrayList<>();
  private final Queue<Double> timestamps = new ArrayBlockingQueue<>(maxQueueCapacity);

  private static DriveOdometryTracker instance = null;

  public static DriveOdometryTracker getInstance() {
    if (instance == null) {
      instance = new DriveOdometryTracker();
    }
    return instance;
  }

  /** Registers the specified signal and returns the corresponding queue. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    final Queue<Double> queue = new ArrayBlockingQueue<>(maxQueueCapacity);
    lock.lock();
    try {
      signals.add(signal);
      queues.add(queue);
    } finally {
      lock.unlock();
    }
    return queue;
  }

  /** Runs update on all the signals. */
  public void update() {
    final var timestamp = Timer.getFPGATimestamp();
    lock.lock();
    try {
      timestamps.offer(timestamp);
      for (var i = 0; i < signals.size(); i++) {
        queues.get(i).offer(signals.get(i).getAsDouble());
      }
    } finally {
      lock.unlock();
    }
  }

  /** Reads current timestamps and runs loggable inputs updates. */
  public double[] read(Runnable updater) {
    lock.lock();
    try {
      // read timestamps and clear the queue
      var timestamps = this.timestamps.stream().mapToDouble(Double::valueOf).toArray();
      this.timestamps.clear();

      // fake sim timestamps
      if (timestamps.length == 0) {
        timestamps = new double[] {Timer.getFPGATimestamp()};
      }

      updater.run();
      return timestamps;
    } finally {
      lock.unlock();
    }
  }
}
