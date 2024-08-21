package frc.robot.commands.teleop;

import java.util.Map;
import java.util.stream.Collectors;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/** Drives at robot relative speed, should be attached to button */
public class RelativeDrive extends Command {

	private final Drive drive;
	private final ChassisSpeeds robotSpeeds;

	/**
	 * Creates a new DriveAtAngle Command
	 *
	 * @param drive       drivetrain of robot
	 * @param robotSpeeds robot relative speeds to drive
	 */
	public RelativeDrive(Drive drive, ChassisSpeeds robotSpeeds) {
		this.drive = drive;

		this.robotSpeeds = robotSpeeds;

		addRequirements(drive);
	}

	@Override
	public void initialize() {
		drive.setRobotSpeeds(robotSpeeds);
	}

	@Override
	public void end(boolean interrupted) {
		drive.setRobotSpeeds(new ChassisSpeeds());
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public String getName() {
		return String.format("%s[%s]", this.getClass().getSimpleName(), Map.of(
				"x", robotSpeeds.vxMetersPerSecond,
				"y", robotSpeeds.vyMetersPerSecond,
				"omega", robotSpeeds.omegaRadiansPerSecond).entrySet().stream()
				.filter((entry) -> Math.abs(entry.getValue()) > 0.01)
				.map((entry) -> (entry.getValue() > 0 ? "+" : "-") + entry.getKey())
				.collect(Collectors.joining(", ")));
	}
}
