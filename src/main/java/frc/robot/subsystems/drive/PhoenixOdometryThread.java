package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>
 * This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class PhoenixOdometryThread extends Thread {
	private final Lock signalsLock = new ReentrantLock(); // Prevents conflicts when registering signals
	private BaseStatusSignal[] signals = new BaseStatusSignal[0];
	private final List<Queue<Double>> queues = new ArrayList<>();
	private final List<Queue<Double>> timestampQueues = new ArrayList<>();
	private boolean isCANFD = false;

	private static PhoenixOdometryThread instance = null;

	public static PhoenixOdometryThread getInstance() {
		if (instance == null) {
			instance = new PhoenixOdometryThread();
		}
		return instance;
	}

	private PhoenixOdometryThread() {
		setName("PhoenixOdometryThread");
		setDaemon(true);
	}

	@Override
	public synchronized void start() {
		if (!timestampQueues.isEmpty()) {
			super.start();
		}
	}

	public Queue<Double> registerSignal(ParentDevice device, StatusSignal<Double> signal) {
		Queue<Double> queue = new ArrayBlockingQueue<>(20);
		signalsLock.lock();
		Drive.odometryLock.lock();
		try {
			isCANFD = CANBus.isNetworkFD(device.getNetwork());
			BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length + 1];
			System.arraycopy(signals, 0, newSignals, 0, signals.length);
			newSignals[signals.length] = signal;
			signals = newSignals;
			queues.add(queue);
		} finally {
			signalsLock.unlock();
			Drive.odometryLock.unlock();
		}
		return queue;
	}

	public Queue<Double> makeTimestampQueue() {
		Queue<Double> queue = new ArrayBlockingQueue<>(20);
		Drive.odometryLock.lock();
		try {
			timestampQueues.add(queue);
		} finally {
			Drive.odometryLock.unlock();
		}
		return queue;
	}

	/** Wait (sleep) until we the next odometry update) */
	private void waitForAllSignals() {
		// Wait for updates from all signals
		signalsLock.lock();
		try {
			final double odometryPeriodSeconds = 1 / DriveConstants.ODOMETRY_FREQUENCY;
			if (isCANFD) {
				BaseStatusSignal.waitForAll(2 * odometryPeriodSeconds, signals);
			} else {
				// "waitForAll" does not support blocking on multiple
				// signals with a bus that is not CAN FD, regardless
				// of Pro licensing. No reasoning for this behavior
				// is provided by the documentation.
				Thread.sleep((long) odometryPeriodSeconds * 1000);
				if (signals.length > 0)
					BaseStatusSignal.refreshAll(signals);
			}
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
			e.printStackTrace();
		} finally {
			signalsLock.unlock();
		}
	}

	/**
	 * @return timestamp in seconds minus the average latency
	 */
	private double getTimestampWithoutLatency() {

		// Get time in seconds
		double timeStampMicroseconds = Logger.getRealTimestamp();
		double timestampSeconds = timeStampMicroseconds / 1e6;

		if (signals.length == 0) {
			return timestampSeconds;
		}

		double totalLatencySeconds = 0.0;
		for (BaseStatusSignal signal : signals) {
			totalLatencySeconds += signal.getTimestamp().getLatency();
		}
		double averageLatency = totalLatencySeconds / signals.length;

		return timestampSeconds - averageLatency;
	}

	/** Locks odometry, updates queues with new values and timestamps */
	private void saveDataFromSignals() {
		// Save new data to queues
		Drive.odometryLock.lock();
		try {
			double timestamp = getTimestampWithoutLatency();

			for (int i = 0; i < signals.length; i++) {
				queues.get(i).offer(signals[i].getValueAsDouble());
			}
			for (int i = 0; i < timestampQueues.size(); i++) {
				timestampQueues.get(i).offer(timestamp);
			}
		} finally {
			Drive.odometryLock.unlock();
		}
	}

	@Override
	public void run() {
		while (true) {

			waitForAllSignals();
			saveDataFromSignals();
		}
	}
}
