package frc.robot.subsystems.gateway;

import frc.robot.hardwareWrappers.RelaySolenoid;
import frc.robot.hardwareWrappers.Transducer;

public class GatewayIOHardware implements GatewayIO {
  private GatewayIOInput input;
  private RelaySolenoid solenoid;
  private Transducer transducer;

  public GatewayIOHardware(RelaySolenoid solenoid) {
    this.solenoid = solenoid;
  }

  @Override
  public void updateInputs(GatewayIOInput inputs) {
    input.psi = (float) transducer.getTankPSI();
  }

  @Override
  public void beginFilling() {
    input.filling = true;
    solenoid.open();
  }

  @Override
  public void stopFilling() {
    input.filling = false;
    solenoid.close();
  }
}
