package frc.robot.utility;

public class LoggedGroup {
	private final String root;

	public LoggedGroup(String root) {
		this.root = root;
	}

	public LoggedTunableNumber buildTunable(String name, double defaultValue) {
		return new LoggedTunableNumber(root + "/" + name, defaultValue);
	}

	public LoggedGroup subgroup(String name) {
		return new LoggedGroup(root + "/" + name);
	}
}