package frc.robot.hardwareWrappers;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;

public class Compressor {

  private final PneumaticHub hub;
  private final Solenoid compressor;

  public Compressor() {
    hub = new PneumaticHub(1);
    compressor = hub.makeSolenoid(8);
  }

  public void startCompressor() {
    compressor.set(true);
  }

  public void stopCompressor() {
    compressor.set(false);
  }
}
