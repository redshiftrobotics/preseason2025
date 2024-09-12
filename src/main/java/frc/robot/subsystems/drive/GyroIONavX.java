package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import java.util.OptionalDouble;
import java.util.Queue;

/**
 * IO implementation for AHRS
 *
 * <p>https://www.andymark.com/products/navx2-mxp-robotics-navigation-sensor
 */
public class GyroIONavX implements GyroIO {
  private static final SPI.Port SERIAL_PORT_ID = SPI.Port.kMXP;

  private final AHRS navX;

  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  /** Create a new NaxX IMU */
  public GyroIONavX() {
    navX = new AHRS(SERIAL_PORT_ID);

    yawTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(() -> OptionalDouble.of(navX.getYaw()));
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navX.isConnected();

    inputs.yawPosition = navX.getRotation2d();
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(navX.getAngleAdjustment());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);

    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  @Override
  public void zeroGyro() {
    navX.reset();
  }
}
