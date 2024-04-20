package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

public class JoystickValue implements DoubleSupplier {
  private static final double DEADBAND = 0.1;

  private final DoubleSupplier supplier;
  private final double coefficient;

  private final DoubleUnaryOperator modifier;

  public JoystickValue(DoubleSupplier valueSupplier, double coefficient) {
    this.supplier = valueSupplier;
    this.coefficient = coefficient;
    this.modifier = (x) -> x;
  }

  public JoystickValue(DoubleSupplier valueSupplier, double coefficient, double rateLimit) {
    this.supplier = valueSupplier;
    this.coefficient = coefficient;
    this.modifier = new SlewRateLimiter(rateLimit)::calculate;
  }

  @Override
  public double getAsDouble() {
    return modifier.applyAsDouble(
        coefficient * MathUtil.applyDeadband(supplier.getAsDouble(), DEADBAND));
  }
}
