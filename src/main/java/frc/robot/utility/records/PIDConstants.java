package frc.robot.utility.records;

/** Store gains in PID controller */
public record PIDConstants(double kP, double kI, double kD) {

  /** Proportional Controller */
  public PIDConstants(double kP) {
    this(kP, 0, 0);
  }

  /** Proportional Derivative Controller */
  public PIDConstants(double kP, double kD) {
    this(kP, 0, kD);
  }
}
