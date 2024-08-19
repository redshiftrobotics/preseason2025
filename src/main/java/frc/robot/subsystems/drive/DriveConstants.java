package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/**
 * Constants for drivetrain/chassis. All constants should be in meters and radians (m/s, m/s^2,
 * rad/s, rad/s^2) Switch expressions must cover all cases.
 *
 * <p>
 * in the future find builder pattern like way to make records, I think there is a library?
 */
public class DriveConstants {

	// --- Drive Config ---

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

	public static final DriveConfig DRIVE_CONFIG = switch (Constants.getRobot()) {
		case SIM_BOT, COMP_BOT -> new DriveConfig(
				Units.inchesToMeters(2),
				0.54,
				0.54,
				0.61,
				0.61,
				6,
				25,
				10,
				6);
		case DEV_BOT -> new DriveConfig(
				Units.inchesToMeters(2),
				0.54,
				0.54,
				0.61,
				0.61,
				4.57,
				22.86,
				10,
				6);
	};

	// --- Module Config ---

	public record ModuleConfig(
			int driveID,
			int turnID,
			int absoluteEncoderChannel,
			Rotation2d absoluteEncoderOffset,
			boolean turnMotorInverted) {
	}

	public static final ModuleConfig FRONT_LEFT_MODULE_CONFIG;
	public static final ModuleConfig FRONT_RIGHT_MODULE_CONFIG;
	public static final ModuleConfig BACK_LEFT_MODULE_CONFIG;
	public static final ModuleConfig BACK_RIGHT_MODULE_CONFIG;

	static {
		switch (Constants.getRobot()) {
			case SIM_BOT: // SIM ONLY
				FRONT_LEFT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, new Rotation2d(), false);
				FRONT_RIGHT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, new Rotation2d(), false);
				BACK_LEFT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, new Rotation2d(), false);
				BACK_RIGHT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, new Rotation2d(), false);
				break;

			case DEV_BOT: // DEV ONLY
				FRONT_LEFT_MODULE_CONFIG = new ModuleConfig(41, 40, 1, new Rotation2d(-0.136474609), false);
				FRONT_RIGHT_MODULE_CONFIG = new ModuleConfig(4, 5, 2, new Rotation2d(-0.3828125), false);
				BACK_LEFT_MODULE_CONFIG = new ModuleConfig(2, 3, 4, new Rotation2d(-0.017333984375), false);
				BACK_RIGHT_MODULE_CONFIG = new ModuleConfig(42, 6, 3, new Rotation2d(0.45385742175), false);
				break;

			default:
			case COMP_BOT: // COMP ONLY - REAL USED BOT
				FRONT_LEFT_MODULE_CONFIG = new ModuleConfig(2, 3, 3, new Rotation2d(0.631591796875), false);
				FRONT_RIGHT_MODULE_CONFIG = new ModuleConfig(14, 17, 4, new Rotation2d(-0.77587890), false);
				BACK_LEFT_MODULE_CONFIG = new ModuleConfig(8, 9, 2, new Rotation2d(-0.641357421875), false);
				BACK_RIGHT_MODULE_CONFIG = new ModuleConfig(10, 11, 1, new Rotation2d(0.453857421), false);
				break;
		}
	}

	// --- Module Constants ---

	public record ModuleConstants(
			double feedForwardKs,
			double feedForwardKv,
			double driveKp,
			double driveKd,
			double turnKp,
			double turnKd,
			double driveReduction,
			double turnReduction) {
	}

	public static final ModuleConstants MODULE_CONSTANTS = switch (Constants.getRobot()) {
		case DEV_BOT -> new ModuleConstants(
				0.1,
				0.13,
				0.1,
				0.0,
				10.0,
				0.0,
				Mk4Reductions.L1.reduction,
				Mk4Reductions.TURN.reduction);
		case SIM_BOT -> new ModuleConstants(
				0.014,
				0.134,
				0.1,
				0.0,
				10.0,
				0.0,
				Mk4iReductions.L3.reduction,
				Mk4iReductions.TURN.reduction);
		case COMP_BOT -> new ModuleConstants(
				5.0,
				0.0,
				35.0,
				0.0,
				4000.0,
				50.0,
				Mk4iReductions.L3.reduction,
				Mk4iReductions.TURN.reduction);
	};

	// --- Module Limit Config ---

	public record ModuleLimits(
			double maxDriveVelocity, double maxDriveAcceleration, double maxSteeringVelocity) {
	}

	public static final ModuleLimits MODULE_LIMITS_FREE = new ModuleLimits(
			DRIVE_CONFIG.maxLinearVelocity(),
			DRIVE_CONFIG.maxLinearAcceleration(),
			Units.degreesToRadians(1080.0));

	public static final ModuleLimits moduleLimitsWhenFlywheelSpinUp = new ModuleLimits(
			DRIVE_CONFIG.maxLinearVelocity(),
			DRIVE_CONFIG.maxLinearAcceleration() / 2.0,
			Units.degreesToRadians(1080.0));

	// --- Odometry Frequency ---

	public static final double ODOMETRY_FREQUENCY = switch (Constants.getRobot()) {
		case SIM_BOT -> 50.0;
		case DEV_BOT -> 100.0;
		case COMP_BOT -> 250.0;
	};

	// --- Heading Controller Config ---

	public record HeadingControllerConstants(
			double kP, double kD, double maxVelocity, double maxAcceleration) {
	}

	public static final HeadingControllerConstants HEADING_CONTROLLER_CONSTANTS = switch (Constants.getRobot()) {
		default -> new HeadingControllerConstants(5.0, 0.0, 8.0, 20.0);
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
	private enum Mk4Reductions {
		L1((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)),
		L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
		L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
		L4((48.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0)),
		TURN(12.8 / 1.0);

		final double reduction;

		Mk4Reductions(double reduction) {
			this.reduction = reduction;
		}
	}
}
