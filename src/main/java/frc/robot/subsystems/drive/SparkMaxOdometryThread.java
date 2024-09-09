package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.Notifier;
import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkMax that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public class SparkMaxOdometryThread {
  private List<Supplier<OptionalDouble>> signals = new ArrayList<>();
  private List<Queue<Double>> queues = new ArrayList<>();

  private List<Queue<Double>> timestampQueues = new ArrayList<>();

  // --- Create Singleton with notifier ---

  private final Notifier notifier;
  private static SparkMaxOdometryThread instance = null;

  /**
   * Returns the singleton instance of the SparkMaxOdometryThread. If the instance does not exist,
   * it is created.
   *
   * @return The singleton instance of SparkMaxOdometryThread.
   */
  public static SparkMaxOdometryThread getInstance() {
    if (instance == null) {
      instance = new SparkMaxOdometryThread();
    }
    return instance;
  }

  /**
   * Private constructor to prevent instantiation from outside the class. Initializes the Notifier
   * to periodically call the {@link #periodic} method.
   */
  private SparkMaxOdometryThread() {
    // Create a Notifier with the given callback (our periodic function).
    // Callback can be called periodically with .startPeriodic()
    notifier = new Notifier(this::periodic);
    notifier.setName("SparkMaxOdometryThread");
  }

  /**
   * Starts the Notifier to periodically call the {@link #periodic} method at the frequency defined
   * by DriveConstants.ODOMETRY_FREQUENCY.
   *
   * <p>This method only starts the Notifier if there are timestamp queues registered to keep track
   * of time.
   */
  public void start() {
    if (!timestampQueues.isEmpty()) {
      notifier.startPeriodic(1.0 / DriveConstants.ODOMETRY_FREQUENCY_HERTZ); // T = 1/f
    }
  }

  /**
   * Registers a signal supplier and creates a queue to hold the values produced by this supplier.
   *
   * @param signal The supplier that provides OptionalDouble values.
   * @return The queue where the sampled values will be stored. Read from this queue as values come
   *     in
   */
  public Queue<Double> registerSignal(Supplier<OptionalDouble> signal) {
    final Queue<Double> queue = new ArrayBlockingQueue<>(20);

    // Add signal and corresponding queue to our list,
    // values will be sampled from signal and put in queue in the periodic function
    Drive.odometryLock.lock();
    try {
      signals.add(signal);
      queues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }

    return queue;
  }

  /**
   * Creates and registers a timestamp queue, which will store the timestamps at which the signals
   * are sampled. a This queue stores the timestamp in seconds with epoch starting at FPGA clock
   * startup.
   *
   * @return The queue where timestamps will be stored. Read from this queue as values come in.
   */
  public Queue<Double> makeTimestampQueue() {
    final Queue<Double> queue = new ArrayBlockingQueue<>(20);

    // Add queue to our list of timestamp queues
    // timestamps will be added to each queue in list of timestamp queues each time periodic is
    // called,
    // this happens at the same time signals are sampled and put in queue
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /**
   * The method that is periodically called by the Notifier. It samples the signals and stores the
   * values and corresponding timestamps in the respective queues.
   *
   * <p>The method ensures that all signals are present before storing the values and timestamps. If
   * any signal is not present, no values or timestamps are stored.
   *
   * <p>This callback should be written so that it completes before the next time it's scheduled to
   * run in the notifier.
   */
  private void periodic() {
    Drive.odometryLock.lock();
    try {

      double timeStampMicroseconds = Logger.getRealTimestamp();
      double timestampSeconds = timeStampMicroseconds / 1e6;

      // Avoiding streams due to performance overhead and high frequency required :(

      // List<OptionalDouble> values = signals.stream().map(Supplier::get).toList();
      // if (values.stream().allMatch(OptionalDouble::isPresent)) {
      // for (int i = 0; i < queues.size(); i++) {
      // queues.get(i).offer(values.get(i).getAsDouble());
      // }
      // for (Queue<Double> queue : timestampQueues) {
      // queue.offer(timestamp);
      // }
      // }

      final double[] values = new double[signals.size()];
      boolean isValid = true;
      for (int i = 0; i < signals.size(); i++) {
        OptionalDouble value = signals.get(i).get();
        if (value.isPresent()) {
          values[i] = value.getAsDouble();
        } else {
          isValid = false;
          break;
        }
      }
      if (isValid) {
        for (int i = 0; i < queues.size(); i++) {
          queues.get(i).offer(values[i]);
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.get(i).offer(timestampSeconds);
        }
      }
    } finally {
      Drive.odometryLock.unlock();
    }
  }
}
