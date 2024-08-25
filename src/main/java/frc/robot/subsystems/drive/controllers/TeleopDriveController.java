package frc.robot.subsystems.drive.controllers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.LoggedTunableNumber;

/** Controller for converting joystick values to drive components */
public class TeleopDriveController {

	private static final LoggedTunableNumber stickDeadband = new LoggedTunableNumber(
			"Drive/TeleopDriveController/Deadband", 0.2);

	private static final LoggedTunableNumber stickDirectionDeadband = new LoggedTunableNumber(
			"Drive/TeleopDriveController/AngleDeadband", 0.5);

	private final Drive drive;

	private final DoubleSupplier xSupplier, ySupplier, xAngleSupplier, yAngleSupplier;

	private static SpeedLevel speedLevel = SpeedLevel.DEFAULT;

	public static enum SpeedLevel {
		PRECISE(0.25, 0.1),
		DEFAULT(0.90, 0.60),
		BOOST(1, 0.75),
		MAX_BOOST(1, 1);

		public final double translationCoefficient, rotationCoefficient;

		private SpeedLevel(double translationCoefficient, double rotationCoefficient) {
			this.translationCoefficient = translationCoefficient;
			this.rotationCoefficient = rotationCoefficient;
		}
	}

	/**
	 * Creates a new TeleopDriveController object
	 *
	 * @param drive drivetrain of robot
	 * @param xSupplier supplier of translational x (forward+)
	 * @param ySupplier supplier of translational y (left+)
	 * @param xAngleSupplier supplier of rotational x (angle)
	 * @param yAngleSupplier supplier of rotational y (angle, ccw_ omega)
	 */
	public TeleopDriveController(
			Drive drive,
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			DoubleSupplier xAngleSupplier,
			DoubleSupplier yAngleSupplier) {
		this.drive = drive;
		this.xSupplier = xSupplier;
		this.ySupplier = ySupplier;
		this.xAngleSupplier = xAngleSupplier;
		this.yAngleSupplier = yAngleSupplier;
	}

	/**
	 * Get desired chassis translation from controller (x and y supplier)
	 *
	 * @return translation x and y in meters per second
	 */
	public Translation2d getTranslationMetersPerSecond() {
		return TeleopDriveController.getTranslationMetersPerSecond(xSupplier.getAsDouble(), ySupplier.getAsDouble(),
				drive.getMaxLinearSpeedMetersPerSec() * TeleopDriveController.speedLevel.translationCoefficient);
	}

	/**
	 * Get desired chassis rotation from controller (y angle supplier)
	 *
	 * @return rotation in unit per second
	 */
	public Rotation2d getOmegaRadiansPerSecond() {
		return TeleopDriveController.getOmegaRadiansPerSecond(yAngleSupplier.getAsDouble(),
				drive.getMaxAngularSpeedRadPerSec() * TeleopDriveController.speedLevel.translationCoefficient);
	}

	/**
	 * Get desired chassis heading from controller (x and y angle supplier)
	 *
	 * @return heading angle
	 */
	public Rotation2d getHeadingDirection() {
		return TeleopDriveController.getHeadingDirection(xAngleSupplier.getAsDouble(), yAngleSupplier.getAsDouble());
	}

	/** Get translational and rotational speed coefficients level */
	public static SpeedLevel getSpeedLevel() {
		return speedLevel;
	}

	/** Set translational and rotational speed coefficients level */
	public static void setSpeedLevel(SpeedLevel speedLevel) {
		TeleopDriveController.speedLevel = speedLevel;
	}

	/** Increase translational and rotational speed coefficients up one level */
	public void increaseSpeedLevel() {
		TeleopDriveController.speedLevel = SpeedLevel.values()[Math.min(speedLevel.ordinal() + 1, SpeedLevel.values().length - 1)];
	}

	/** Decrease translational and rotational speed coefficients down one level */
	public void decreaseSpeedLevel() {
		TeleopDriveController.speedLevel = SpeedLevel.values()[Math.max(speedLevel.ordinal() - 1, 0)];
	}

	private static Translation2d getTranslationMetersPerSecond(double xInput, double yInput,
			double maxTranslationSpeedMetersPerSecond) {

		Translation2d translation = new Translation2d(xInput, yInput);

		// get length of linear velocity vector, and apply deadband to it for noise reduction
		double magnitude = MathUtil.applyDeadband(translation.getNorm(), stickDeadband.get());

		// squaring the magnitude of the vector makes for quicker ramp up and slower fine control,
		// magnitude should always be positive
		double magnitudeSquared = Math.copySign(Math.pow(magnitude, 2), 1);

		// get a vector with the same angle as the base linear velocity vector but with the magnitude squared
		Translation2d squaredLinearVelocity = new Pose2d(new Translation2d(), translation.getAngle())
				.transformBy(new Transform2d(magnitudeSquared, 0.0, new Rotation2d()))
				.getTranslation();

		// return final value
		return squaredLinearVelocity.times(maxTranslationSpeedMetersPerSecond);
	}

	private static Rotation2d getOmegaRadiansPerSecond(double omegaInput, double maxAngularSpeedRadPerSec) {

		// get rotation speed, and apply deadband
		double omega = MathUtil.applyDeadband(omegaInput, stickDeadband.get());

		// square the omega value for quicker ramp up and slower fine control
		// make sure to copy the sign over for direction
		double omegaSquared = Math.copySign(Math.pow(omega, 2), omega);

		// return final value
		return new Rotation2d(omegaSquared * maxAngularSpeedRadPerSec);
	}

	private static Rotation2d getHeadingDirection(double xInput, double yInput) {
		// Get desired angle as a vector
		final Translation2d desiredAngle = new Translation2d(xInput, yInput);

		// If the vector length is longer then our deadband update the heading controller
		if (desiredAngle.getNorm() > stickDirectionDeadband.get()) {
			return desiredAngle.getAngle();
		}

		return null;
	}
}
