package frc.robot.commands.teleop;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.teleop.input.DriverInput;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.HeadingController;

/** Drive control command for driving robot with x and y with heading locked */
public class TeleopLockedHeading extends Command {
	private final Drive drive;
	private final DriverInput input;
	private final Supplier<Rotation2d> headingSupplier;
	private final HeadingController headingController;

	/**
	 * Creates a new TeleopDrive Command. Meant to be default command for drivetrain.
	 *
	 * @param drive           drivetrain of robot
	 * @param input           inputs for drive
	 * @param headingSupplier supplier for angle
	 */
	public TeleopLockedHeading(Drive drive, DriverInput input, Supplier<Rotation2d> headingSupplier) {

		this.drive = drive;
		this.input = input;
		this.headingSupplier = headingSupplier;

		this.headingController = new HeadingController(drive);

		addRequirements(drive);
	}

	@Override
	public void initialize() {
		headingController.reset();
	}

	@Override
	public void execute() {

		Translation2d translation = input.getTranslationMetersPerSecond();

		double rotationRadians = headingController.calculate(headingSupplier.get());

		boolean fieldRelative = input.getFieldRelative();

		ChassisSpeeds speeds = new ChassisSpeeds(
				translation.getX(),
				translation.getY(),
				rotationRadians);

		drive.setRobotSpeeds(speeds, fieldRelative);
	}

	@Override
	public boolean isFinished() {
		return Math.abs(input.getOmegaRadiansPerSecond().getRadians()) > 1E-3;
	}

	@Override
	public void end(boolean interrupted) {
		drive.stop();
	}

	@Override
	public String getName() {
		return String.format("%s[%d]", getClass().getSimpleName(), (int) headingController.getGoal().getDegrees());
	}
}
