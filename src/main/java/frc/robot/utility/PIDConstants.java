package frc.robot.utility;

public record PIDConstants(double kP, double kI, double kD) {
  public PIDConstants(double kP) {
    this(kP, 0, 0);
  }

  public PIDConstants(double kP, double kD) {
    this(kP, 0, kD);
  }
}
