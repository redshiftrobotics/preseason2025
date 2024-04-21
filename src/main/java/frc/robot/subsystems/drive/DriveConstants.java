package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/**
 * Constants for drivetrain/chassis. All constants should be in meters and radians (m/s, m/s^2,
 * rad/s, rad/s^2) Switch expressions must cover all cases.
 */
public class DriveConstants {

  public record DriveConfig(
      double wheelRadius,
      double trackWidthX,
      double trackWidthY,
      double bumperWidthX,
      double bumperWidthY,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration) {
    public double driveBaseRadius() {
      return Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
    }
  }

  public static final DriveConfig driveConfig =
      switch (Constants.getRobot()) {
        case SIM_BOT, COMP_BOT -> new DriveConfig(
            Units.inchesToMeters(2), // wheel radius
            Units.inchesToMeters(25), // track width x/y
            Units.inchesToMeters(25),
            Units.inchesToMeters(33), // bumper width x/y
            Units.inchesToMeters(33),
            Units.feetToMeters(15.0), // max linear accel and velocity
            Units.feetToMeters(75.0),
            10, // max angular accel and velocity
            6.0);
        case DEV_BOT -> new DriveConfig(
            Units.inchesToMeters(2.01834634),
            Units.inchesToMeters(20.75),
            Units.inchesToMeters(20.75),
            Units.inchesToMeters(37),
            Units.inchesToMeters(33),
            Units.feetToMeters(12.16),
            Units.feetToMeters(21.32),
            7.93,
            29.89);
      };

  public record HeadingControllerConstants(
      double kP, double kD, double maxVelocity, double maxAcceleration) {}

  // Swerve Heading Control
  public static final HeadingControllerConstants headingControllerConstants =
      switch (Constants.getRobot()) {
        default -> new HeadingControllerConstants(5.0, 0.0, 8.0, 20.0);
      };

  public record ModuleLimits(
      double maxDriveVelocity, double maxDriveAcceleration, double maxSteeringVelocity) {}

  public static final ModuleLimits moduleLimitsFree =
      new ModuleLimits(
          driveConfig.maxLinearVelocity(),
          driveConfig.maxLinearAcceleration(),
          Units.degreesToRadians(1080.0));

  public static final double ODOMETRY_FREQUENCY =
      switch (Constants.getRobot()) {
        case SIM_BOT -> 50.0;
        case DEV_BOT -> 100.0;
        case COMP_BOT -> 250.0;
      };
}
