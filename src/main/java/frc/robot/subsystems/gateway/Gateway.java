package frc.robot.subsystems.gateway;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gateway extends SubsystemBase {
  private GatewayIO IO;
  private GatewayIOInputAutoLogged input;
  private BangBangController controller;
  
  public Gateway(GatewayIO gatewayIO) {
    IO = gatewayIO;
  }
  
  @Override
  public void periodic() {
    if (controller.calculate(input.psi, getTargetPsi()) == 1) {
      IO.beginFilling();
    } else {
      IO.stopFilling();
    }

    IO.updateInputs(input);
  }

  public float getTargetPsi() {
    return input.targetPsi;
  }

  public void setTargetPsi(int targetPsi) {
    input.targetPsi = targetPsi;
  }

  public void stopFilling() {
    input.targetPsi = input.psi;
    IO.stopFilling();
  }

  public void fireCannon(byte cannonId) {
    IO.fireCannon(cannonId);
  }
}
