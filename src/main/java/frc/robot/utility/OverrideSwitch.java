package frc.robot.utility;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

/** Class for switching on and off features, implements BooleanSupplier */
public class OverrideSwitch implements BooleanSupplier {
    private final Subsystem requirement;

    private final boolean defaultValue;
    private boolean value;

    private final String name;

	/**
	 * Creates new Override switch
	 *
     * @param trigger      trigger to toggle state
     * @param defaultValue starting state
     * @param requirement  will clear command on this subsystem on state change
	 */
	public OverrideSwitch(Trigger trigger, boolean defaultValue, Subsystem requirement) {
		this(trigger, defaultValue, requirement, null);
	}

    /**
	 * Creates new Override switch
     *
	 * @param trigger      trigger to toggle state
	 * @param defaultValue starting state
	 * @param requirement  will clear command on this subsystem on state change
	 * @param name		   name for smart dashboard
     */
    public OverrideSwitch(Trigger trigger, boolean defaultValue, Subsystem requirement, String name) {
        this.defaultValue = defaultValue;

        this.requirement = requirement;
		this.name = name;

        trigger.onTrue(Commands.runOnce(this::toggle));

		this.reset();
    }

    public OverrideSwitch(Trigger trigger, boolean defaultValue) {
        this(trigger, defaultValue, null);
    }

    public OverrideSwitch(Trigger trigger) {
        this(trigger, false);
    }

    private void set(boolean value) {
        this.value = value;

		if (name != null) {
			SmartDashboard.putBoolean(name, value);
		}

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
