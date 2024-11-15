package frc.robot.subsystems.gateway;

import org.littletonrobotics.junction.AutoLog;

public interface GatewayIO {
    @AutoLog
    public static class GatewayIOInput {
        boolean filling;
        float psi;
    }

    public default void updateInputs(GatewayIOInput inputs) {}

    public void beginFilling();
    public void stopFilling();
    public void fireCannon(byte cannonId);
}
