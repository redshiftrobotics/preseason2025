package frc.robot.hardwareWrappers;

import edu.wpi.first.wpilibj.Relay;

public class RelaySolenoid {
  private final Relay relay;

  // Relays
  // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/index.html#relays

  /**
   * Create solenoid
   *
   * @param relayChannel Relay channel for the solenoid (Relay 0-3 are on-board)
   */
  public RelaySolenoid(int relayChannel) {
    relay = new Relay(relayChannel);
  }

  /** Open the solenoid */
  public void open() {
    relay.set(Relay.Value.kForward);
  }

  /** Close the solenoid */
  public void close() {
    relay.set(Relay.Value.kOff);
  }
}
