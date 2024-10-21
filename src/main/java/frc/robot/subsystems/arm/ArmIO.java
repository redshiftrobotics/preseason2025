package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double positionRads = 0.0;
    public double absoluteEncoderPositionRads = 0.0;
    public double relativeEncoderPositionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Run to setpoint angle in radians */
  public default void runSetpoint(double positionRad, double feedForward) {}

  /** Run motors at volts */
  default void runVolts(double volts) {}

  /** Stops motors */
  default void stop() {}

  /** Enable or disable brake mode on the motor. */
  public default void setBrakeMode(boolean enable) {}

  /** Set velocity PID constants. */
  public default void configurePID(double Kp, double Ki, double Kd) {}
}
