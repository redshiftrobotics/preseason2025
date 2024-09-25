package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.MODULE_CONSTANTS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants.ModuleConfig;
import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

/**
 * Check that this will work. FULLY UNTESTED.
 *
 * <p>Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller,
 * and CANcoder. Specifically for Krakens motors with Talons, with added power and efficiency
 * benefits of Field Oriented Control (FOC)
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOKrakenFOC implements ModuleIO {

  // Hardware
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  // Controller Config
  private final TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration turnTalonConfig = new TalonFXConfiguration();
  private static final Executor configExecutor = Executors.newFixedThreadPool(8);

  // Odometry Queues
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Drive Status Signals
  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;
  private final StatusSignal<Double> driveTorqueCurrent;

  // Turn Status Signals
  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;
  private final StatusSignal<Double> turnTorqueCurrent;

  public ModuleIOKrakenFOC(ModuleConfig config) {

    // Create motor controllers
    driveTalon = new TalonFX(config.driveID());
    turnTalon = new TalonFX(config.turnID());

    // Create absolute encoder
    cancoder = new CANcoder(config.absoluteEncoderChannel());

    // Torque
    driveTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    driveTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    driveTalonConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

    turnTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    turnTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;

    // Set Current limits
    driveTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    turnTalonConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    turnTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Invert
    turnTalonConfig.MotorOutput.Inverted =
        config.turnMotorInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Break
    driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    driveTalon.getConfigurator().apply(driveTalonConfig);
    turnTalon.getConfigurator().apply(turnTalonConfig);

    // Gear Ratio
    driveTalonConfig.Feedback.SensorToMechanismRatio = MODULE_CONSTANTS.driveReduction();
    turnTalonConfig.Feedback.SensorToMechanismRatio = MODULE_CONSTANTS.turnReduction();
    turnTalonConfig.ClosedLoopGeneral.ContinuousWrap = true;

    // Cancoder config
    cancoder
        .getConfigurator()
        .apply(
            new CANcoderConfiguration()
                .withMagnetSensor(
                    new MagnetSensorConfigs()
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                        .withMagnetOffset(config.absoluteEncoderOffset().getRotations())));

    // Odometry Queues
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());

    // Drive Status Signals
    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();
    driveTorqueCurrent = driveTalon.getTorqueCurrent();

    // Turn Status Signals
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getSupplyCurrent();
    turnTorqueCurrent = turnTalon.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        DriveConstants.ODOMETRY_FREQUENCY_HERTZ, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveTorqueCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent,
        turnTorqueCurrent);

    // Reset turn position to absolute encoder position
    turnTalon.setPosition(turnAbsolutePosition.getValue(), 1.0);

    // Optimize bus utilization
    driveTalon.optimizeBusUtilization(0, 1.0);
    turnTalon.optimizeBusUtilization(0, 1.0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveTorqueCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent,
        turnTorqueCurrent);

    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveCurrent.getValueAsDouble();
    inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getAppliedUpdateFrequency();

    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnSupplyCurrentAmps = turnCurrent.getValueAsDouble();

    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble(Units::rotationsToRadians).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts).withUpdateFreqHz(0));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(new VoltageOut(volts).withUpdateFreqHz(0));
  }

  @Override
  public void setDriveVelocity(double velocityRadsPerSec, double feedForward) {
    driveTalon.setControl(
        new VelocityDutyCycle(velocityRadsPerSec).withFeedForward(feedForward).withUpdateFreqHz(0));
  }

  @Override
  public void setTurnPosition(double angleRads) {
    driveTalon.setControl(
        new PositionDutyCycle(Units.radiansToRotations(angleRads)).withUpdateFreqHz(0));
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveTalonConfig.Slot0.kP = kP;
    driveTalonConfig.Slot0.kI = kI;
    driveTalonConfig.Slot0.kD = kD;
    driveTalon.getConfigurator().apply(driveTalonConfig, 0.01);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnTalonConfig.Slot0.kP = kP;
    turnTalonConfig.Slot0.kI = kI;
    turnTalonConfig.Slot0.kD = kD;
    turnTalon.getConfigurator().apply(turnTalonConfig, 0.01);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    // Annoyingly complicated way, but we don't want to block main thread, and we can't have two
    // threads writing at the same time
    configExecutor.execute(
        () -> {
          synchronized (driveTalonConfig) {
            driveTalonConfig.MotorOutput.NeutralMode =
                enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            driveTalon.getConfigurator().apply(driveTalonConfig, 0.25);
          }
        });
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    configExecutor.execute(
        () -> {
          synchronized (turnTalonConfig) {
            turnTalonConfig.MotorOutput.NeutralMode =
                enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            turnTalon.getConfigurator().apply(turnTalonConfig, 0.25);
          }
        });
  }

  @Override
  public void stop() {
    driveTalon.setControl(new NeutralOut().withUpdateFreqHz(0));
    turnTalon.setControl(new NeutralOut().withUpdateFreqHz(0));
  }
}
