package frc.robot.commands.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.LoggedTunableNumber;
import java.util.function.Supplier;
import java.util.function.BooleanSupplier;

/** Drive control command for driving robot with x, y and omega control */
public class TeleopDrive extends Command {

	private static final LoggedTunableNumber controllerDeadband = new LoggedTunableNumber(
			"TeleopDrive/Deadband", 0.2);

	private static final LoggedTunableNumber maxTranslationCoefficient = new LoggedTunableNumber(
			"TeleopDrive/maxTranslationCoefficient", 1);

	private static final LoggedTunableNumber maxOmegaCoefficient = new LoggedTunableNumber(
			"TeleopDrive/maxOmegaCoefficient", 0.65);

	private final Drive drive;
	private final Supplier<Translation2d> translationStick, rotationStick;
	private final BooleanSupplier fieldRelativeSupplier;

	/**
	 * Creates a new TeleopAngleDrive Command. Meant to be default command for drivetrain.
	 *
	 * Sticks are in Joystick Space, convert them to 2d Robot Space
	 *
	 * @see https://docs.wpilib.org/en/stable/_images/joystick-3d.svg
	 * @see	https://docs.wpilib.org/en/stable/_images/robot-2d.svg
	 *
	 * @param drive                 drivetrain of robot
	 * @param movementStick         stick x and y supplier
	 * @param angleStick            stick x and y supplier
	 * @param fieldRelativeSupplier supply true for field relative, false for robot relative
	 */
	public TeleopDrive(
			Drive drive,
			Supplier<Translation2d> movementStick,
			Supplier<Translation2d> angleStick,
			BooleanSupplier fieldRelativeSupplier) {

		this.drive = drive;

		this.translationStick = movementStick;
		this.rotationStick = angleStick;

		this.fieldRelativeSupplier = fieldRelativeSupplier;

		addRequirements(drive);
	}

	public Drive getDrive() {
		return drive;
	}

	@Override
	public void initialize() {
		drive.stopUsingForwardArrangement();
	}

	@Override
	public void execute() {
		Translation2d translation = getTranslationMetersPerSecond(translationStick.get());
		double omega = getOmegaRadiansPerSecond(rotationStick.get());
		drive.setRobotSpeeds(
			new ChassisSpeeds(
				translation.getX() * maxTranslationCoefficient.get(),
				translation.getY() * maxTranslationCoefficient.get(),
				omega * maxOmegaCoefficient.get()),
			fieldRelativeSupplier.getAsBoolean()
		);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		drive.stop();
	}

	/**
	 * Get translation (x and y) in meters per second.
	 *
	 * https://docs.wpilib.org/en/stable/_images/robot-2d.svg
	 *
	 * @param input Input from movement joysticks. X and Y both [-1, 1]
	 * @return translation of robot
	 */
	public Translation2d getTranslationMetersPerSecond(Translation2d input) {

		// get length of linear velocity vector, and apply deadband to it for noise reduction
		double magnitude = MathUtil.applyDeadband(input.getNorm(), controllerDeadband.get());

		// squaring the magnitude of the vector makes for quicker ramp up and slower fine control,
		// magnitude should always be positive
		double magnitudeSquared = Math.copySign(Math.pow(magnitude, 2), 1);

		// get a vector with the same angle as the base linear velocity vector but with the magnitude squared
		Translation2d squaredLinearVelocity = new Pose2d(new Translation2d(), input.getAngle())
				.transformBy(new Transform2d(magnitudeSquared, 0.0, new Rotation2d()))
				.getTranslation();

		// return final value
		return squaredLinearVelocity.times(drive.getMaxLinearSpeedMetersPerSec());
	}

	/**
	 * Get omega (rotation speed) in radians per second.
	 *
	 * @param input Input from movement joysticks. X and Y both [-1, 1]
	 * @return omega of robot
	 */
	public double getOmegaRadiansPerSecond(Translation2d input) {

		// get rotation speed, and apply deadband
		double omega = MathUtil.applyDeadband(input.getY(), controllerDeadband.get());

		// square the omega value for quicker ramp up and slower fine control
		// make sure to copy the sign over for direction
		double omegaSquared = Math.copySign(Math.pow(omega, 2), omega);

		// return final value
		return omegaSquared * drive.getMaxAngularSpeedRadPerSec();
	}
}
