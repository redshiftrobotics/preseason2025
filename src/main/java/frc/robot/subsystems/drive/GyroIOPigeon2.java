package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.OptionalDouble;
import java.util.Queue;

/**
 * IO implementation for Pigeon2 IMU
 *
 * <p>https://store.ctr-electronics.com/pigeon-2/
 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;

  private final StatusSignal<Double> yaw;
  private final StatusSignal<Double> yawVelocity;

  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  /**
   * Create a new Pigeon2 IMU
   *
   * @param phoenixDrive true if drivetrain is using Phoenix, false for SparkMax
   */
  public GyroIOPigeon2(int deviceID, boolean phoenixDrive) {
    pigeon = new Pigeon2(deviceID);
    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();

    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);

    yaw.setUpdateFrequency(DriveConstants.ODOMETRY_FREQUENCY_HERTZ);
    yawVelocity.setUpdateFrequency(100.0);

    pigeon.optimizeBusUtilization();

    if (phoenixDrive) {
      yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
      yawPositionQueue =
          PhoenixOdometryThread.getInstance().registerSignal(pigeon, pigeon.getYaw());
    } else {
      yawTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
      yawPositionQueue =
          SparkMaxOdometryThread.getInstance()
              .registerSignal(
                  () ->
                      yaw.refresh().getStatus().isOK()
                          ? OptionalDouble.of(yaw.getValueAsDouble())
                          : OptionalDouble.empty());
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);

    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);

    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  @Override
  public void zeroGyro() {
    pigeon.reset();
  }
}
