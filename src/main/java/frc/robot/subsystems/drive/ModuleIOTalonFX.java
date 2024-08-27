package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.MODULE_CONSTANTS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants.ModuleConfig;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
    private final TalonFX driveTalon;
    private final TalonFX turnTalon;
    private final CANcoder cancoder;

    private final Queue<Double> timestampQueue;

    private final StatusSignal<Double> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveCurrent;

    private final StatusSignal<Double> turnAbsolutePosition;
    private final StatusSignal<Double> turnPosition;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<Double> turnVelocity;
    private final StatusSignal<Double> turnAppliedVolts;
    private final StatusSignal<Double> turnCurrent;

    private final boolean isTurnMotorInverted;
    private final Rotation2d absoluteEncoderOffset;

    public ModuleIOTalonFX(ModuleConfig config) {
        driveTalon = new TalonFX(config.driveID());
        turnTalon = new TalonFX(config.turnID());
        cancoder = new CANcoder(config.absoluteEncoderChannel());
        absoluteEncoderOffset = config.absoluteEncoderOffset();
        isTurnMotorInverted = config.turnMotorInverted();

        var driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveTalon.getConfigurator().apply(driveConfig);
        setDriveBrakeMode(true);

        var turnConfig = new TalonFXConfiguration();
        turnConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnTalon.getConfigurator().apply(turnConfig);
        setTurnBrakeMode(true);

        cancoder.getConfigurator().apply(new CANcoderConfiguration());

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        drivePosition = driveTalon.getPosition();
        drivePositionQueue =
                PhoenixOdometryThread.getInstance()
                        .registerSignal(driveTalon, driveTalon.getPosition());
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getSupplyCurrent();

        turnAbsolutePosition = cancoder.getAbsolutePosition();
        turnPosition = turnTalon.getPosition();
        turnPositionQueue =
                PhoenixOdometryThread.getInstance()
                        .registerSignal(turnTalon, turnTalon.getPosition());
        turnVelocity = turnTalon.getVelocity();
        turnAppliedVolts = turnTalon.getMotorVoltage();
        turnCurrent = turnTalon.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                DriveConstants.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                driveVelocity,
                driveAppliedVolts,
                driveCurrent,
                turnAbsolutePosition,
                turnVelocity,
                turnAppliedVolts,
                turnCurrent);
        driveTalon.optimizeBusUtilization();
        turnTalon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveCurrent,
                turnAbsolutePosition,
                turnPosition,
                turnVelocity,
                turnAppliedVolts,
                turnCurrent);

        inputs.drivePositionRad =
                Units.rotationsToRadians(drivePosition.getValueAsDouble())
                        / MODULE_CONSTANTS.driveReduction();
        inputs.driveVelocityRadPerSec =
                Units.rotationsToRadians(driveVelocity.getValueAsDouble())
                        / MODULE_CONSTANTS.driveReduction();
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

        inputs.turnAbsolutePosition =
                Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
                        .minus(absoluteEncoderOffset);
        inputs.turnPosition =
                Rotation2d.fromRotations(
                        turnPosition.getValueAsDouble() / MODULE_CONSTANTS.turnReduction());
        inputs.turnVelocityRadPerSec =
                Units.rotationsToRadians(turnVelocity.getValueAsDouble())
                        / MODULE_CONSTANTS.turnReduction();
        inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
        inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};

        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
                drivePositionQueue.stream()
                        .mapToDouble(
                                (Double value) ->
                                        Units.rotationsToRadians(value)
                                                / MODULE_CONSTANTS.driveReduction())
                        .toArray();
        inputs.odometryTurnPositions =
                turnPositionQueue.stream()
                        .map(
                                (Double value) ->
                                        Rotation2d.fromRotations(
                                                value / MODULE_CONSTANTS.turnReduction()))
                        .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveTalon.setControl(new VoltageOut(volts));
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnTalon.setControl(new VoltageOut(volts));
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        var config = new MotorOutputConfigs();
        config.Inverted = InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveTalon.getConfigurator().apply(config);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        var config = new MotorOutputConfigs();
        config.Inverted =
                isTurnMotorInverted
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        turnTalon.getConfigurator().apply(config);
    }
}
