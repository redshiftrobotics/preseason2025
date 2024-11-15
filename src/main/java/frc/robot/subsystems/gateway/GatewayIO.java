package frc.robot.subsystems.gateway;

import org.littletonrobotics.junction.AutoLog;

public interface GatewayIO {
  @AutoLog
  public static class GatewayIOInput {
    boolean filling;
    float psi;
  }

  public default void updateInputs(GatewayIOInput inputs) {}

  public default void beginFilling() {}

  public default void stopFilling() {}

  public default void fireCannon(byte cannonId) {}
}
