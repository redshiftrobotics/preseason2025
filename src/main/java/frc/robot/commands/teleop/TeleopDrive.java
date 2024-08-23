package frc.robot.commands.teleop;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.teleop.input.DriverInput;
import frc.robot.subsystems.drive.Drive;

/** Drive control command for driving robot with x, y and omega control */
public class TeleopDrive extends Command {
	private final Drive drive;
	private final DriverInput input;

	/**
	 * Creates a new TeleopDrive Command. Meant to be default command for drivetrain.
	 *
	 * @param drive drivetrain of robot
	 * @param input inputs for drive
	 */
	public TeleopDrive(Drive drive, DriverInput input) {

		this.drive = drive;
		this.input = input;

		addRequirements(drive);
	}

	@Override
	public void initialize() {
		drive.stopUsingForwardArrangement();
	}

	@Override
	public void execute() {

		Translation2d translation = input.getTranslationMetersPerSecond();

		Rotation2d rotation = input.getOmegaRadiansPerSecond();

		boolean fieldRelative = input.getFieldRelative();

		ChassisSpeeds speeds = new ChassisSpeeds(
				translation.getX(),
				translation.getY(),
				rotation.getRadians());

		drive.setRobotSpeeds(speeds, fieldRelative);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public String getName() {
		return String.format("%s[%s]", getClass().getSimpleName(), input.getSpeedLevel());
	}
}
