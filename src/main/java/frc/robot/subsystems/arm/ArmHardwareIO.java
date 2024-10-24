package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.ARM_CONFIG;
import static frc.robot.subsystems.examples.flywheel.FlywheelConstants.GEAR_RATIO;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;

public class ArmHardwareIO implements ArmIO {
  private final CANSparkMax leader;
  private final CANSparkMax follower;

  private final RelativeEncoder encoder;
  private final SparkPIDController pid;

  private final CANcoder cancoder;
  private final Supplier<Double> absolutePosition;
  private final Supplier<Double> relativePosition;

  public ArmHardwareIO() {

    // --- Create Hardware ---
    leader = new CANSparkMax(ARM_CONFIG.leaderID(), MotorType.kBrushless);
    follower = new CANSparkMax(ARM_CONFIG.followerID(), MotorType.kBrushless);

    cancoder = new CANcoder(ARM_CONFIG.encoderId());

    // --- Set up CANCoder ---
    CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
    armEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    armEncoderConfig.MagnetSensor.MagnetOffset = ArmConstants.ARM_ENCODER_OFFSET.getRotations();
    cancoder.getConfigurator().apply(armEncoderConfig, 1.0);

    absolutePosition = cancoder.getAbsolutePosition().asSupplier();
    relativePosition = cancoder.getPosition().asSupplier();

    // --- Set up leader controller ---
    encoder = leader.getEncoder();
    pid = leader.getPIDController();

    // --- Configure Spark Maxes ---

    // Defaults
    leader.restoreFactoryDefaults();
    follower.restoreFactoryDefaults();

    // Conversion Factor
    encoder.setPositionConversionFactor(1 / GEAR_RATIO);
    encoder.setVelocityConversionFactor(1 / GEAR_RATIO);

    pid.setPositionPIDWrappingMinInput(0);
    pid.setPositionPIDWrappingMaxInput(1);
    pid.setPositionPIDWrappingEnabled(true);

    leader.setSoftLimit(SoftLimitDirection.kForward, 0);
    leader.setSoftLimit(SoftLimitDirection.kForward, 1);

    // Set follower to copy leader
    leader.setInverted(ARM_CONFIG.leaderInverted());
    follower.follow(leader, ARM_CONFIG.followerInverted());

    // Enable brake
    leader.setIdleMode(IdleMode.kBrake);
    follower.setIdleMode(IdleMode.kBrake);

    // Voltage
    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(30);

    // Save config
    leader.burnFlash();
    follower.burnFlash();

    // --- Match encoder positions ---
    encoder.setPosition(absolutePosition.get());
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.positionRads = Units.rotationsToRadians(encoder.getPosition());
    inputs.velocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());

    inputs.absoluteEncoderPositionRads = Units.rotationsToRadians(absolutePosition.get());
    inputs.relativeEncoderPositionRads = Units.rotationsToRadians(relativePosition.get());

    SmartDashboard.putNumber("Position", Units.radiansToDegrees(inputs.positionRads));
    SmartDashboard.putNumber("Velocity", Units.radiansToDegrees(inputs.velocityRadsPerSec));

    SmartDashboard.putNumber(
        "AbsPosition", Units.radiansToDegrees(inputs.absoluteEncoderPositionRads));
    SmartDashboard.putNumber(
        "RelPosition", Units.radiansToDegrees(inputs.relativeEncoderPositionRads));

    SmartDashboard.putNumber("leader out", leader.getAppliedOutput());
    SmartDashboard.putNumber("follower out", leader.getAppliedOutput());

    inputs.appliedVolts =
        new double[] {
          leader.getAppliedOutput() * leader.getBusVoltage(),
          follower.getAppliedOutput() * follower.getBusVoltage()
        };
    inputs.supplyCurrentAmps =
        new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
  }

  @Override
  public void runSetpoint(double positionRad, double feedForward) {
    pid.setReference(
        Units.radiansToRotations(positionRad),
        ControlType.kPosition,
        0,
        feedForward,
        ArbFFUnits.kPercentOut);
  }

  @Override
  public void runVolts(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    leader.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    follower.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }
}
