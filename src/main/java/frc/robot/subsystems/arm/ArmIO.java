package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {

    public double rotation = 0;
    public double rotationPerSecond = 0;

    public boolean isAtDesiredPosition = false;

    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Update motor powers */
  public default void periodic() {}

  public default void setSetpoint(double degrees) {}
}
