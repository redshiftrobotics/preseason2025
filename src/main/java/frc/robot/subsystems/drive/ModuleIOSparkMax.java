package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.MODULE_CONSTANTS;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants.ModuleConfig;
import java.util.OptionalDouble;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and CANCoder absolute encoder
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad". Update these values in DriveConstants.
 */
public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;
  private final CANcoder cancoder;

  private final RelativeEncoder driveRelativeEncoder;
  private final RelativeEncoder turnRelativeEncoder;

  private final SparkPIDController driveFeedback;
  private final SparkPIDController turnFeedback;

  private final StatusSignal<Double> turnAbsolutePosition;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final boolean isTurnMotorInverted;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(ModuleConfig config) {

    // --- Save config ---
    driveSparkMax = new CANSparkMax(config.driveID(), MotorType.kBrushless);
    turnSparkMax = new CANSparkMax(config.turnID(), MotorType.kBrushless);
    cancoder = new CANcoder(config.absoluteEncoderChannel());
    absoluteEncoderOffset = config.absoluteEncoderOffset();
    isTurnMotorInverted = config.turnMotorInverted();

    // --- Get Built In PID ---
    driveSparkMax.getPIDController();

    // --- Configure absolute encoder ---

    cancoder
        .getConfigurator()
        .apply(
            new CANcoderConfiguration()
                .withMagnetSensor(
                    new MagnetSensorConfigs()
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)));

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnAbsolutePosition.setUpdateFrequency(50);

    // --- Configure spark maxes ---

    // Prepare for changing spark max settings
    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    // Turn
    turnSparkMax.setInverted(isTurnMotorInverted);
    
    // Current
    driveSparkMax.setSmartCurrentLimit(40);
    turnSparkMax.setSmartCurrentLimit(30);

    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    // Encoders
    driveRelativeEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    driveRelativeEncoder.setPosition(0.0);
    driveRelativeEncoder.setMeasurementPeriod(10);
    driveRelativeEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    // Complete changing spark max settings
    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    // Set update period 
    double odometryPeriodSeconds = 1.0 / DriveConstants.ODOMETRY_FREQUENCY_HERTZ;
    driveSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (odometryPeriodSeconds * 1000));
    turnSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (odometryPeriodSeconds * 1000));

    // --- Queues From Odometry ---
    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();

    drivePositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () ->
                    driveSparkMax.getLastError() == REVLibError.kOk
                        ? OptionalDouble.of(driveRelativeEncoder.getPosition())
                        : OptionalDouble.empty());

    turnPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () ->
                    turnSparkMax.getLastError() == REVLibError.kOk
                        ? OptionalDouble.of(turnRelativeEncoder.getPosition())
                        : OptionalDouble.empty());

    // Write all spark max settings to flash
    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    turnAbsolutePosition.refresh();

    // --- Drive ---
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveRelativeEncoder.getPosition())
            / MODULE_CONSTANTS.driveReduction();
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveRelativeEncoder.getVelocity())
            / MODULE_CONSTANTS.driveReduction();
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveSupplyCurrentAmps = driveSparkMax.getOutputCurrent();

    // --- Turn ---
    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(
            turnRelativeEncoder.getPosition() / MODULE_CONSTANTS.turnReduction());
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / MODULE_CONSTANTS.turnReduction();
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnSupplyCurrentAmps = turnSparkMax.getOutputCurrent();

    // --- Odometry ---
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble(
                (Double value) ->
                    Units.rotationsToRadians(value) / MODULE_CONSTANTS.driveReduction())
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map(
                (Double value) ->
                    Rotation2d.fromRotations(value / MODULE_CONSTANTS.turnReduction()))
            .toArray(Rotation2d[]::new);

    // --- Clear queues after use ---
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void stop() {
    driveSparkMax.stopMotor();
    turnSparkMax.stopMotor();
  }
}
