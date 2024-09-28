package frc.robot.utility.logging;

public class LoggedTunableNumberGroup {
  private final String root;

  public LoggedTunableNumberGroup(String root) {
    this.root = root;
  }

  public LoggedTunableNumber build(String name, double defaultValue) {
    return new LoggedTunableNumber(root + "/" + name, defaultValue);
  }

  public LoggedTunableNumberGroup subgroup(String name) {
    return new LoggedTunableNumberGroup(root + "/" + name);
  }
}
