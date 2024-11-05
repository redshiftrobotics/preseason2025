package frc.robot.subsystems.reservoir;

import org.littletonrobotics.junction.AutoLog;

public interface ReservoirIO {
  @AutoLog
  public static class ReservoirIOInputs {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ReservoirIOInputs inputs) {}
}
