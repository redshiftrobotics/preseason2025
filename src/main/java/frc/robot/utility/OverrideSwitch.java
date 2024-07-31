package frc.robot.utility;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

/** Class for switching on and off features, implements BooleanSupplier */
public class OverrideSwitch implements BooleanSupplier {
  private final Subsystem requirement;

  private final boolean defaultValue;
  private boolean value;

  /**
   * Creates new Override switch
   *
   * @param trigger Trigger to toggle state
   * @param defaultValue Starting state
   * @param requirement Will clear command on this subsystem on state change
   */
  public OverrideSwitch(Trigger trigger, boolean defaultValue, Subsystem requirement) {
    this.defaultValue = value = defaultValue;

    this.requirement = requirement;

    trigger.onTrue(Commands.runOnce(this::toggle));
  }

  public OverrideSwitch(Trigger trigger, boolean defaultValue) {
    this(trigger, defaultValue, null);
  }

  public OverrideSwitch(Trigger trigger) {
    this(trigger, false);
  }

  private void set(boolean value) {
    this.value = value;

    if (requirement != null && requirement.getCurrentCommand() != null) {
      requirement.getCurrentCommand().cancel();
    }
  }

  public void toggle() {
    set(!value);
  }

  public void reset() {
    set(defaultValue);
  }

  public void toggleOn() {
    set(true);
  }

  public void toggleOff() {
    set(false);
  }

  public boolean get() {
    return value;
  }

  @Override
  public boolean getAsBoolean() {
    return get();
  }
}
