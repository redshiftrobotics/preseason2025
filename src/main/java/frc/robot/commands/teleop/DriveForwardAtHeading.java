package frc.robot.commands.teleop;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.HeadingController;
import frc.robot.utility.LoggedTunableNumber;

/** First rotates to specified angle, then starts slowing driving in that direction */
public class DriveForwardAtHeading extends Command {

	private static final LoggedTunableNumber driveSpeed = new LoggedTunableNumber("TeleopDrive/POV/driveSpeed", 1);

	private static final LoggedTunableNumber delaySecondsTillForward = new LoggedTunableNumber(
			"TeleopDrive/POV/delaySecondsTillForward", 0.5);

	private final Drive drive;
	private final HeadingController headingController;
	private final Timer timer;

	/**
	 * Creates new DriveForwardAtAngle Command
	 *
	 * @param drive   drivetrain of robot
	 * @param heading field relative rotation to drive to aim at before driving forward
	 */
	public DriveForwardAtHeading(Drive drive, Rotation2d heading) {
		this.drive = drive;

		headingController = new HeadingController(drive);
		headingController.setGoal(heading);

		timer = new Timer();

		addRequirements(drive);
	}

	@Override
	public void initialize() {
		headingController.reset();
		timer.restart();
	}

	@Override
	public void execute() {
		drive.setRobotSpeeds(getDesiredSpeeds(), false);
	}

	private ChassisSpeeds getDesiredSpeeds() {
		return new ChassisSpeeds(
				headingController.atGoal() && timer.hasElapsed(delaySecondsTillForward.get())
						? driveSpeed.get()
						: 0,
				0,
				headingController.calculate());
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
