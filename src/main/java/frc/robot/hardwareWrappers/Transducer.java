package frc.robot.hardwareWrappers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class Transducer {
  private final AnalogInput input;

  // Analog Inputs (Analog in)

  // Software:
  // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/analog-inputs-software.html

  // Hardware:
  // https://docs.wpilib.org/en/stable/docs/hardware/sensors/analog-inputs-hardware.html#analog-inputs-hardware

  /**
   * Create traducer
   *
   * @param analogInChannel Analog input channel for the transducer (Analog 0-3 are on-board)
   */
  public Transducer(int analogInChannel) {
    input = new AnalogInput(analogInChannel);
  }

  /**
   * Get PSI reading from analog input voltage
   *
   * @return PSI reading from transducer
   */
  public double getTankPSI() {
    return MathUtil.interpolate(0, 150, MathUtil.inverseInterpolate(.5, 4.5, input.getVoltage()));
  }

  /**
   * Get PSI reading from analog input voltage. WITH EXTRA SCALING
   *
   * @return PSI reading from transducer
   */
  public double getTankPSI2() {
    double volts =
        (input.getVoltage() / RobotController.getVoltage5V()) * 5; // TODO, is this needed?

    return MathUtil.interpolate(0, 150, MathUtil.inverseInterpolate(.5, 4.5, volts));
  }
}
