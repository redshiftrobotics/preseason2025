package frc.robot.hardwareWrappers;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import java.time.Duration;

public class PressureSwitch {
  private final DigitalInput input;
  private final DigitalGlitchFilter filter;

  // Digital Inputs (DIO)

  // Software:
  // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/digital-inputs-software.html

  // Hardware:
  // https://docs.wpilib.org/en/stable/docs/hardware/sensors/digital-inputs-hardware.html

  /**
   * Create pressure switch
   *
   * @param digitalIOChannel DIO channel for the pressure switch (DIO 0-9 are on-board)
   */
  public PressureSwitch(int digitalIOChannel) {
    input = new DigitalInput(digitalIOChannel);
    filter = new DigitalGlitchFilter();

    filter.setPeriodNanoSeconds(Duration.ofMillis(5).toNanos());
    filter.add(input);
  }

  /** Get whether pressure switch is */
  public boolean isSwitched() {
    // Gets the value of the digital input. Returns true if the circuit is open.
    // When the signal pin is floating (not connected to any circuit), they consistently remain in a
    // high state
    return input.get();
  }
}
