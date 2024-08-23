package frc.robot.commands.teleop.input;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utility.LoggedTunableNumber;

/** Utils for converting joystick values to drive components */
public class DriverInputUtil {

	private static final LoggedTunableNumber stickDeadband = new LoggedTunableNumber(
			"TeleopDrive/Deadband", 0.2);

	private static final LoggedTunableNumber stickDirectionDeadband = new LoggedTunableNumber(
			"TeleopDrive/AngleDeadband", 0.5);

	public static Translation2d getTranslationMetersPerSecond(double xInput, double yInput, double maxTranslationSpeedMetersPerSecond) {

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

	public static Rotation2d getOmegaRadiansPerSecond(double omegaInput, double maxAngularSpeedRadPerSec) {

		// get rotation speed, and apply deadband
		double omega = MathUtil.applyDeadband(omegaInput, stickDeadband.get());

		// square the omega value for quicker ramp up and slower fine control
		// make sure to copy the sign over for direction
		double omegaSquared = Math.copySign(Math.pow(omega, 2), omega);

		// return final value
		return new Rotation2d(omegaSquared * maxAngularSpeedRadPerSec);
	}

	public static Rotation2d getHeadingDirection(double xInput, double yInput) {
		// Get desired angle as a vector
		final Translation2d desiredAngle = new Translation2d(xInput, yInput);

		// If the vector length is longer then our deadband update the heading controller
		if (desiredAngle.getNorm() > stickDirectionDeadband.get()) {
			return desiredAngle.getAngle();
		}

		return null;
	}
}
