package frc.robot.utility;

/** Store gains for simple motor feed forward controller */
public record FeedForwardConstants(double kS, double kV, double kA) {
  public FeedForwardConstants(double ks, double kv) {
    this(ks, kv, 0);
  }
}
