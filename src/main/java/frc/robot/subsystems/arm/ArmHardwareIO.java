package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.ARM_CONFIG;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;

public class ArmHardwareIO implements ArmIO {
  private final CANSparkMax left;
  private final CANSparkMax right;

  private PIDController pid;

  private final CANcoder cancoder;
  private final Supplier<Double> absolutePosition;
  private final Supplier<Double> relativePosition;
  private final Supplier<Double> velocity;

  public ArmHardwareIO() {

    // --- Create Spark Maxes---
    left = new CANSparkMax(ARM_CONFIG.leftID(), MotorType.kBrushless);
    right = new CANSparkMax(ARM_CONFIG.rightID(), MotorType.kBrushless);

    // --- Create Can Coder ---
    cancoder = new CANcoder(ARM_CONFIG.encoderId());

    MagnetSensorConfigs magnetSensorConfig = new MagnetSensorConfigs();
    magnetSensorConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    magnetSensorConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    magnetSensorConfig.MagnetOffset = ArmConstants.ARM_ENCODER_OFFSET.getRotations();
    cancoder.getConfigurator().apply(magnetSensorConfig);

    absolutePosition = cancoder.getAbsolutePosition().asSupplier();
    relativePosition = cancoder.getPosition().asSupplier();
    velocity = cancoder.getVelocity().asSupplier();

    // --- Configure Spark Maxes ---

    // Defaults
    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();

    // Invert
    left.setInverted(ARM_CONFIG.leftInverted());
    right.setInverted(ARM_CONFIG.rightInverted());

    // Enable brake
    left.setIdleMode(IdleMode.kBrake);
    right.setIdleMode(IdleMode.kBrake);

    // Voltage
    left.enableVoltageCompensation(12.0);
    left.setSmartCurrentLimit(30);

    right.enableVoltageCompensation(12.0);
    right.setSmartCurrentLimit(30);

    // Save config
    left.burnFlash();
    right.burnFlash();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    inputs.absoluteEncoderPositionRads = Units.rotationsToRadians(absolutePosition.get());
    inputs.relativeEncoderPositionRads = Units.rotationsToRadians(relativePosition.get());
    inputs.velocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocity.get());

    inputs.positionRads = inputs.absoluteEncoderPositionRads;

    inputs.appliedVolts =
        new double[] {
          left.getAppliedOutput() * left.getBusVoltage(),
          right.getAppliedOutput() * right.getBusVoltage()
        };
    inputs.supplyCurrentAmps = new double[] {left.getOutputCurrent(), right.getOutputCurrent()};
  }

  @Override
  public void periodic() {
    double speed = pid.calculate(absolutePosition.get());

    left.set(speed);
    right.set(speed);
  }

  @Override
  public void runSetpoint(double positionRad, double feedForward) {
    pid.setSetpoint(Units.radiansToRotations(positionRad));
  }

  @Override
  public void runVolts(double volts) {
    left.setVoltage(volts);
  }

  @Override
  public void stop() {
    left.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    left.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    right.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid = new PIDController(kP, kI, kD);
  }
}
