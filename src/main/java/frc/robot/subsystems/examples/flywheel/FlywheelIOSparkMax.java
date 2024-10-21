package frc.robot.subsystems.examples.flywheel;

import static frc.robot.subsystems.examples.flywheel.FlywheelConstants.FLYWHEEL_CONFIG;
import static frc.robot.subsystems.examples.flywheel.FlywheelConstants.GEAR_RATIO;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class FlywheelIOSparkMax implements FlywheelIO {
  private final CANSparkMax leader;
  private final CANSparkMax follower;
  private final RelativeEncoder encoder;
  private final SparkPIDController pid;

  public FlywheelIOSparkMax() {

    // --- Save config ---
    leader = new CANSparkMax(FLYWHEEL_CONFIG.followerID(), MotorType.kBrushless);
    follower = new CANSparkMax(FLYWHEEL_CONFIG.leaderID(), MotorType.kBrushless);

    // --- Set up leader controller ---
    encoder = leader.getEncoder();
    pid = leader.getPIDController();

    // --- Configure Hardware ---

    // Defaults
    leader.restoreFactoryDefaults();
    follower.restoreFactoryDefaults();

    // Set follower to copy leader
    leader.setInverted(FLYWHEEL_CONFIG.leaderInverted());
    follower.follow(leader, FLYWHEEL_CONFIG.followerInverted());

    // Disable brake
    leader.setIdleMode(IdleMode.kCoast);
    follower.setIdleMode(IdleMode.kCoast);

    // Voltage
    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(30);

    // Save config
    leader.burnFlash();
    follower.burnFlash();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts =
        new double[] {
          leader.getAppliedOutput() * leader.getBusVoltage(),
          follower.getAppliedOutput() * follower.getBusVoltage()
        };
    inputs.supplyCurrentAmps =
        new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }
}
