package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.utility.FeedForwardConstants;
import frc.robot.utility.PIDConstants;
import frc.robot.utility.swerve254util.ModuleLimits;

/**
 * Constants for drivetrain/chassis. All constants should be in meters and radians (m/s, m/s^2,
 * rad/s, rad/s^2). Switch expressions must cover all cases.
 */
public class DriveConstants {

  public static final boolean USE_254_SWERVE_SETPOINT = true;

  // --- Drive Config ---

  public record DriveConfig(
      double wheelRadius,
      Translation2d trackCornerToCorner,
      Translation2d bumperCornerToCorner,
      double maxLinearVelocity,
      double maxLinearAcceleration) {
    public double driveBaseRadius() {
      return trackCornerToCorner.getNorm() * 0.5;
    }

    public double maxAngularVelocity() {
      return maxLinearVelocity / driveBaseRadius();
    }

    public double maxAngularAcceleration() {
      return maxLinearAcceleration / driveBaseRadius();
    }
  }

  public static final DriveConfig DRIVE_CONFIG =
      switch (Constants.getRobot()) {
        case COMP_BOT -> new DriveConfig(
            Units.inchesToMeters(2),
            new Translation2d(0.885, 0.885),
            new Translation2d(0.9612, 0.9612),
            5.05968,
            14.5);
        case DEV_BOT -> new DriveConfig(
            Units.inchesToMeters(2),
            new Translation2d(0.885, 0.885),
            new Translation2d(0.9612, 0.9612),
            5.05968,
            14.5);
        case OLD_DEV_BOT -> new DriveConfig(
            Units.inchesToMeters(2),
            new Translation2d(0.885, 0.885),
            new Translation2d(0.9612, 0.9612),
            3.81,
            14.5);
        case SIM_BOT -> new DriveConfig(
            Units.inchesToMeters(2),
            new Translation2d(0.885, 0.885),
            new Translation2d(0.9612, 0.9612),
            5.05968,
            14.5);
      };

  // --- Module Config ---

  public record ModuleConfig(
      int driveID,
      int turnID,
      int absoluteEncoderChannel,
      Rotation2d absoluteEncoderOffset,
      boolean turnMotorInverted) {}

  public static final ModuleConfig FRONT_LEFT_MODULE_CONFIG;
  public static final ModuleConfig FRONT_RIGHT_MODULE_CONFIG;
  public static final ModuleConfig BACK_LEFT_MODULE_CONFIG;
  public static final ModuleConfig BACK_RIGHT_MODULE_CONFIG;

  static {
    switch (Constants.getRobot()) {
      case SIM_BOT:
        FRONT_LEFT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, new Rotation2d(), false);
        FRONT_RIGHT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, new Rotation2d(), false);
        BACK_LEFT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, new Rotation2d(), false);
        BACK_RIGHT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, new Rotation2d(), false);
        break;

      case DEV_BOT:
        FRONT_LEFT_MODULE_CONFIG =
            new ModuleConfig(2, 3, 3, Rotation2d.fromRotations(0.631591796875), false);
        FRONT_RIGHT_MODULE_CONFIG =
            new ModuleConfig(14, 17, 4, Rotation2d.fromRotations(-0.77758789062), false);
        BACK_LEFT_MODULE_CONFIG =
            new ModuleConfig(8, 9, 2, Rotation2d.fromRotations(-0.641357421875), false);
        BACK_RIGHT_MODULE_CONFIG =
            new ModuleConfig(10, 11, 1, Rotation2d.fromRotations(0.453857421875), false);
        break;

      case OLD_DEV_BOT:
        FRONT_LEFT_MODULE_CONFIG =
            new ModuleConfig(2, 3, 3, Rotation2d.fromRotations(0.631591796875), false);
        FRONT_RIGHT_MODULE_CONFIG =
            new ModuleConfig(14, 17, 4, Rotation2d.fromRotations(-0.77758789062), false);
        BACK_LEFT_MODULE_CONFIG =
            new ModuleConfig(8, 9, 2, Rotation2d.fromRotations(-0.641357421875), false);
        BACK_RIGHT_MODULE_CONFIG =
            new ModuleConfig(10, 11, 1, Rotation2d.fromRotations(0.453857421875), false);
        break;

      default:
      case COMP_BOT:
        FRONT_LEFT_MODULE_CONFIG =
            new ModuleConfig(2, 3, 3, Rotation2d.fromRotations(0.631591796875), false);
        FRONT_RIGHT_MODULE_CONFIG =
            new ModuleConfig(14, 17, 4, Rotation2d.fromRotations(-0.77587890), false);
        BACK_LEFT_MODULE_CONFIG =
            new ModuleConfig(8, 9, 2, Rotation2d.fromRotations(-0.641357421875), false);
        BACK_RIGHT_MODULE_CONFIG =
            new ModuleConfig(10, 11, 1, Rotation2d.fromRotations(0.453857421), false);
        break;
    }
  }

  // --- Module Constants ---

  public record ModuleConstants(
      FeedForwardConstants driveFeedforward,
      PIDConstants driveFeedback,
      PIDConstants turnFeedback,
      double driveReduction,
      double turnReduction) {}

  public static final ModuleConstants MODULE_CONSTANTS =
      switch (Constants.getRobot()) {
        case OLD_DEV_BOT -> new ModuleConstants(
            new FeedForwardConstants(0.1, 3.12, 0.40),
            new PIDConstants(0.1, 0.0, 0.0),
            new PIDConstants(10.0, 0.0, 0.0),
            Mk4Reductions.L1.reduction,
            Mk4Reductions.TURN.reduction);
        default -> new ModuleConstants(
            new FeedForwardConstants(0.1, 2.35, 0.53),
            new PIDConstants(0.1, 0.0, 0.0),
            new PIDConstants(10.0, 0.0, 0.0),
            Mk4iReductions.L3.reduction,
            Mk4iReductions.TURN.reduction);
      };

  // --- Module Limit Config ---

  public static final ModuleLimits MODULE_LIMITS_FREE =
      new ModuleLimits(
          DRIVE_CONFIG.maxLinearVelocity(),
          DRIVE_CONFIG.maxLinearAcceleration(),
          Units.degreesToRadians(1080.0));

  public static final ModuleLimits MODULE_LIMITS_FLYWHEEL_SPIN_UP =
      new ModuleLimits(
          DRIVE_CONFIG.maxLinearVelocity(),
          DRIVE_CONFIG.maxLinearAcceleration() / 2.0,
          Units.degreesToRadians(1080.0));

  // --- Odometry Frequency ---

  public static final double ODOMETRY_FREQUENCY_HERTZ =
      switch (Constants.getRobot()) {
        case SIM_BOT -> 50.0;
        default -> 100.0;
      };

  // --- Heading Controller Config ---

  public record HeadingControllerConstants(double Kp, double Kd) {}

  public static final HeadingControllerConstants HEADING_CONTROLLER_CONSTANTS =
      switch (Constants.getRobot()) {
        default -> new HeadingControllerConstants(5.0, 0.0);
      };

  // --- Module reductions ---

  // https://www.swervedrivespecialties.com/products/mk4i-swerve-module
  private enum Mk4iReductions {
    L1((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)),
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    TURN((150.0 / 7.0));

    final double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }

  // https://www.swervedrivespecialties.com/products/mk4-swerve-module
  @SuppressWarnings("unused")
  private enum Mk4Reductions {
    L1((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)),
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    L4((48.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    TURN((12.8 / 1.0));

    final double reduction;

    Mk4Reductions(double reduction) {
      this.reduction = reduction;
    }
  }
}
