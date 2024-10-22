package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ArmConstants {

  public record ArmConfig(
      int leaderID,
      int followerID,
      int encoderId,
      boolean leaderInverted,
      Boolean followerInverted) {}

  public static final ArmConfig ARM_CONFIG =
      switch (Constants.getRobot()) {
        case OLD_DEV_BOT -> new ArmConfig(19, 5, 6, true, false);
        default -> new ArmConfig(0, 0, 0, false, false);
      };

  public static final Rotation2d ARM_MIN_ANGLE = Rotation2d.fromDegrees(-5);
  public static final Rotation2d ARM_MAX_ANGLE = Rotation2d.fromDegrees(180);

  public static final Rotation2d ARM_ENCODER_OFFSET =
      switch (Constants.getRobot()) {
        default -> new Rotation2d();
      };

  public static final double GEAR_RATIO = (5.0 / 1.0) * (5.0 / 1.0) * (4.0 / 1.0) * (3.0 / 1.0);

  public static final double ARM_LENGTH_METERS = Units.inchesToMeters(24);
  public static final Translation2d ARM_ORIGIN = new Translation2d(-0.238, 0.298);

  public record PID(double Kp, double Ki, double Kd) {}

  public static final PID PID_CONFIG =
      switch (Constants.getRobot()) {
        default -> new PID(80.0, 0, 0);
      };

  public record ArmFeedForward(double Ks, double Kg, double Kv, double Ka) {}

  public static final ArmFeedForward FEED_FORWARD_CONFIG =
      switch (Constants.getRobot()) {
        default -> new ArmFeedForward(0, 0, 0, 0);
      };

  public static TrapezoidProfile.Constraints PROFILED_CONSTRAINTS =
      new TrapezoidProfile.Constraints(2 * Math.PI, 15);
}
