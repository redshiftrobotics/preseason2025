package frc.robot.commands.teleop;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/** Drives at field relative speed, should be attached to button */
public class DriveAtSpeeds extends Command {

	private final Drive drive;
	private final ChassisSpeeds driveSpeeds;

	/**
	 * Creates a new DriveAtAngle Command
	 *
	 * @param drive  drivetrain of robot
	 * @param speeds field relative speeds to drive
	 */
	public DriveAtSpeeds(Drive drive, ChassisSpeeds speeds) {
		this.drive = drive;

		driveSpeeds = speeds;

		addRequirements(drive);
	}

	@Override
	public void initialize() {
		drive.setRobotSpeeds(driveSpeeds);
	}

	@Override
	public void end(boolean interrupted) {
		drive.setRobotSpeeds(new ChassisSpeeds());
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
