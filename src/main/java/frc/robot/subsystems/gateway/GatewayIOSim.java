package frc.robot.subsystems.gateway;

public class GatewayIOSim implements GatewayIO {
  private GatewayIOInput input;

  @Override
  public void updateInputs(GatewayIOInput input) {
    periodic();

    this.input = input;
  }

  public void periodic() {
    if (input.filling) input.psi++;
  }

  @Override
  public void beginFilling() {
    input.filling = true;
  }

  @Override
  public void stopFilling() {
    input.filling = false;
  }

  @Override
  public void fireCannon(byte cannonId) {
    input.psi = 0;
  }
}
