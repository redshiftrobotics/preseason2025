package frc.robot.commands.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.HeadingController;
import frc.robot.utility.AllianceFlipUtil;
import frc.robot.utility.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Drive control command for driving robot with x, y control and absolute angle control */
public class TeleopAngleDrive extends Command {
	private static final LoggedTunableNumber controllerDeadband = new LoggedTunableNumber(
			"TeleopDrive/Angle/Deadband", 0.2);
	private static final LoggedTunableNumber controllerAngleDeadband = new LoggedTunableNumber(
			"TeleopDrive/Angle/AngleDeadband", 0.75);

	private final Drive drive;
	private final DoubleSupplier xSupplier, ySupplier, xAngleSupplier, yAngleSupplier;
	private final BooleanSupplier fieldRelativeSupplier;
	private final HeadingController headingController;

	/**
	 * Creates a new TeleopAngleDrive Command. Meant to be default command for drivetrain.
	 *
	 * @param drive                 drivetrain of robot
	 * @param xSupplier             double from controller joystick for X (forward) velocity
	 * @param ySupplier             double from controller joystick for Y (left) velocity
	 * @param xAngleSupplier        double from controller joystick for base (used in arc tangent)
	 * @param yAngleSupplier        double from controller joystick for perpendicular (used in arc tangent)
	 * @param fieldRelativeSupplier supply true for field relative, false for robot relative
	 */
	public TeleopAngleDrive(
			Drive drive,
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			DoubleSupplier xAngleSupplier,
			DoubleSupplier yAngleSupplier,
			BooleanSupplier fieldRelativeSupplier) {

		headingController = new HeadingController(drive);

		this.drive = drive;
		this.xSupplier = xSupplier;
		this.ySupplier = ySupplier;
		this.xAngleSupplier = xAngleSupplier;
		this.yAngleSupplier = yAngleSupplier;
		this.fieldRelativeSupplier = fieldRelativeSupplier;

		addRequirements(drive);
	}

	@Override
	public void initialize() {
		headingController.reset();
		// set starting setpoint to current rotation to prevent spinning at start
		headingController.setGoal(drive.getPose().getRotation());
		drive.stopUsingForwardArrangement();
	}

	@Override
	public void execute() {
		drive.setRobotSpeeds(getDesiredSpeeds(), fieldRelativeSupplier.getAsBoolean());
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	public ChassisSpeeds getDesiredSpeeds() {
		// Get raw linear velocity vector
		final Translation2d linearVelocity = new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

		// get length of linear velocity vector, and apply deadband to it for noise reduction
		final double magnitude = MathUtil.applyDeadband(linearVelocity.getNorm(), controllerDeadband.get());
		// Squaring the magnitude of the vector makes for better ramp up and better fine control
		final double magnitudeSquared = magnitude * magnitude;

		// get a vector with the same angle as the base linear velocity vector but with the magnitude
		// squared
		final Translation2d squaredLinearVelocity = new Pose2d(new Translation2d(), linearVelocity.getAngle())
				.transformBy(new Transform2d(magnitudeSquared, 0.0, new Rotation2d()))
				.getTranslation();

		// Get desired angle as a vector
		final Translation2d desiredAngle = new Translation2d(xAngleSupplier.getAsDouble(),
				yAngleSupplier.getAsDouble());

		// If the vector length is longer then our deadband update the heading controller
		if (desiredAngle.getNorm() > controllerAngleDeadband.get()) {
			headingController.setGoal(AllianceFlipUtil.apply(desiredAngle.getAngle()));
		}

		// convert percentage speeds to actual meters per sec speeds
		return new ChassisSpeeds(
				squaredLinearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
				squaredLinearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
				headingController.calculate());
	}
}
