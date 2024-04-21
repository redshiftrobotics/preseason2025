package frc.robot.utility;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

public class OverrideSwitch implements BooleanSupplier {
  private boolean value;

  public OverrideSwitch(Trigger trigger, boolean defaultValue) {
    value = defaultValue;
    trigger.onTrue(Commands.runOnce(this::toggle));
  }

  public OverrideSwitch(Trigger trigger) {
    this(trigger, false);
  }

  public void toggle() {
    value = !value;
  }

  public void toggleOn() {
    value = true;
  }

  public void toggleOff() {
    value = false;
  }

  public boolean get() {
    return value;
  }

  @Override
  public boolean getAsBoolean() {
    return get();
  }
}
