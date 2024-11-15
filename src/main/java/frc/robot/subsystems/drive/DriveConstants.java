package frc.robot.subsystems.drive;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/**
 * Constants for drivetrain/chassis. All constants should be in meters and radians (m/s, m/s^2,
 * rad/s, rad/s^2). Switch expressions must cover all cases.
 */
public class DriveConstants {
  private DriveConstants() {}

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
        case CANNON_BOT -> new DriveConfig(
            Units.inchesToMeters(2),
            new Translation2d(0.885, 0.885),
            new Translation2d(0.9612, 0.9612),
            5.05968,
            14.5);
        case SIM_BOT -> new DriveConfig(
            Units.inchesToMeters(2),
            new Translation2d(0.885, 0.885),
            new Translation2d(0.9612, 0.9612),
            5.05968,
            14.5);
      };

  public static PathConstraints PATH_CONSTRAINS =
      new PathConstraints(3.0, 3.0, 3 * Math.PI, 4 * Math.PI); // Currently a bit arbitrary

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

      case CANNON_BOT:
      default:
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

  public static final Translation2d FRONT_LEFT_MODULE_DISTANCE_FROM_CENTER;
  public static final Translation2d FRONT_RIGHT_MODULE_DISTANCE_FROM_CENTER;
  public static final Translation2d BACK_LEFT_MODULE_DISTANCE_FROM_CENTER;
  public static final Translation2d BACK_RIGHT_MODULE_DISTANCE_FROM_CENTER;

  static {
    switch (Constants.getRobot()) {
        // case OLD_DEV_BOT: // Exception, old dev bot is no longer a square
        //   double frontLeftToRight = Units.inchesToMeters(0);
        //   double backLeftToRight = Units.inchesToMeters(0);
        //   double leftFrontToBack = Units.inchesToMeters(0);
        //   double rightFrontToBack = Units.inchesToMeters(0);
        //   FRONT_LEFT_MODULE_DISTANCE_FROM_CENTER =
        //       new Translation2d(leftFrontToBack / 2.0, frontLeftToRight / 2.0);
        //   FRONT_RIGHT_MODULE_DISTANCE_FROM_CENTER =
        //       new Translation2d(rightFrontToBack / 2.0, -frontLeftToRight / 2.0);
        //   BACK_LEFT_MODULE_DISTANCE_FROM_CENTER =
        //       new Translation2d(-leftFrontToBack / 2.0, backLeftToRight / 2.0);
        //   BACK_RIGHT_MODULE_DISTANCE_FROM_CENTER =
        //       new Translation2d(-rightFrontToBack / 2.0, -backLeftToRight / 2.0);
        //   break;
      default:
        double trackCenterX = DRIVE_CONFIG.trackCornerToCorner().getX() / 2;
        double trackCenterY = DRIVE_CONFIG.trackCornerToCorner().getY() / 2;
        FRONT_LEFT_MODULE_DISTANCE_FROM_CENTER = new Translation2d(trackCenterX, trackCenterY);
        FRONT_RIGHT_MODULE_DISTANCE_FROM_CENTER = new Translation2d(trackCenterX, -trackCenterY);
        BACK_LEFT_MODULE_DISTANCE_FROM_CENTER = new Translation2d(-trackCenterX, trackCenterY);
        BACK_RIGHT_MODULE_DISTANCE_FROM_CENTER = new Translation2d(-trackCenterX, -trackCenterY);
        break;
    }
  }

  // --- Gyro Config ---

  public static final int GYRO_CAN_ID =
      switch (Constants.getRobot()) {
        case CANNON_BOT -> 40;
        default -> 0;
      };

  // --- Module Constants ---

  public record ModuleConstants(
      FeedForward driveFeedforward,
      PID driveFeedback,
      PID turnFeedback,
      double driveReduction,
      double turnReduction) {}

  public static final ModuleConstants MODULE_CONSTANTS =
      switch (Constants.getRobot()) {
        case SIM_BOT -> new ModuleConstants(
            new FeedForward(0.1, 3.12, 0.40),
            new PID(0.1, 0.0, 0.0),
            new PID(10.0, 0.0, 0.0),
            Mk4iReductions.L3.reduction,
            Mk4iReductions.TURN.reduction);
        case CANNON_BOT -> new ModuleConstants(
            new FeedForward(0.1, 2.35, 0.53),
            new PID(0.000006, 0.0, 0.0),
            new PID(10.0, 0.0, 0.0),
            Mk4iReductions.L3.reduction,
            Mk4iReductions.TURN.reduction);
      };

  // --- Odometry Frequency ---

  public static final double ODOMETRY_FREQUENCY_HERTZ =
      switch (Constants.getRobot()) {
        case SIM_BOT -> 50.0;
        default -> 100.0;
      };

  // --- Heading Controller Config ---

  public record HeadingControllerConstants(double Kp, double Kd, double toleranceDegrees) {}

  public static final HeadingControllerConstants HEADING_CONTROLLER_CONSTANTS =
      switch (Constants.getRobot()) {
        default -> new HeadingControllerConstants(5.0, 0.0, 1);
      };

  // --- Control ---

  public record PID(double Kp, double Ki, double Kd) {}

  public record FeedForward(double Ks, double Kv, double Ka) {}

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

  @SuppressWarnings("unused")
  // https://www.swervedrivespecialties.com/products/mk4-swerve-module
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
