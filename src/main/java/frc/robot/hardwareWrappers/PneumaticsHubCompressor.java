package frc.robot.hardwareWrappers;

/** Compressor for compressor plugged into REV Pneumatic Hub with pressure sensor is allowed port */
public class PneumaticsHubCompressor {

  private final CustomPneumaticHub hub;

  /**
   * Create compressor
   */
  public PneumaticsHubCompressor() {
    hub = new CustomPneumaticHub();
  }

  public void setTargetPressure(int minPressure, int maxPressure) {
    hub.enableCompressorAnalog(minPressure, maxPressure);
  }

  public void disableCompressor() {
    hub.disableCompressor();
  }

  public boolean isCompressorEnabled() {
    return hub.getCompressor();
  }
}
