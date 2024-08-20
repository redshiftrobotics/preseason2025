package frc.robot.commands.teleop;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.HeadingController;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Drive control command for driving robot with x, y and omega control */
public class TeleopDrive extends Command {
	private final Drive drive;

	// Modes

	private static enum Mode {
		DEFAULT,
		HEADING_CONTROLLED,
		HEADING_CONTROLLED_PASSIVE,
		DRIVE_ROBOT_RELATIVE_SPEEDS,
	}

	private Mode mode = Mode.DEFAULT;

	// Default
	private final DoubleSupplier xSupplier, ySupplier, omegaSupplier;
	private final BooleanSupplier fieldRelativeSupplier;

	// Heading controlled
	private final HeadingController headingController;

	// drive robot speeds
	private ChassisSpeeds robotRelativeSpeeds;

	/**
	 * Creates a new TeleopAngleDrive Command. Meant to be default command for drivetrain.
	 *
	 * @param drive                 drivetrain of robot
	 * @param xSupplier             double from controller joystick for X (forward) velocity
	 * @param ySupplier             double from controller joystick for Y (left) velocity
	 * @param omegaSupplier         double from controller joystick for omega (rotation) velocity
	 * @param fieldRelativeSupplier supply true for field relative, false for robot relative
	 */
	public TeleopDrive(
			Drive drive,
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			DoubleSupplier omegaSupplier,
			BooleanSupplier fieldRelativeSupplier) {

		this.drive = drive;

		this.xSupplier = xSupplier;
		this.ySupplier = ySupplier;
		this.omegaSupplier = omegaSupplier;
		this.fieldRelativeSupplier = fieldRelativeSupplier;

		this.headingController = new HeadingController(drive);

		addRequirements(drive);
	}

	// --- HEADING CONTROLLED ---

	public Command setHeadingCommand(Rotation2d heading) {
		return Commands.startEnd(() -> setHeading(heading), this::clearHeading).until(headingController::atGoal);
	}

	private void setHeading(Rotation2d heading) {
		headingController.reset();
		headingController.setGoal(heading);
		mode = Mode.HEADING_CONTROLLED;
	}

	private void clearHeading() {
		mode = Mode.HEADING_CONTROLLED_PASSIVE;
	}

	// --- DRIVE ROBOT RELATIVE SPEEDS ---

	public Command setRobotRelativeSpeedsCommands(ChassisSpeeds speeds) {
		return Commands.startEnd(() -> setRobotRelativeSpeeds(speeds), this::clearRobotRelativeSpeeds);
	}

	private void setRobotRelativeSpeeds(ChassisSpeeds speeds) {
		robotRelativeSpeeds = speeds;
		mode = Mode.DRIVE_ROBOT_RELATIVE_SPEEDS;
	}

	private void clearRobotRelativeSpeeds() {
		robotRelativeSpeeds = null;
		drive.stop();
		mode = Mode.DEFAULT;
	}

	// --- Command ---

	@Override
	public void initialize() {
		drive.stopUsingForwardArrangement();
	}

	@Override
	public void execute() {

		Translation2d translation = DriverInput.getTranslationMetersPerSecond(
				xSupplier.getAsDouble(),
				ySupplier.getAsDouble(),
				drive.getMaxLinearSpeedMetersPerSec());

		Rotation2d rotation = DriverInput.getOmegaRadiansPerSecond(
				omegaSupplier.getAsDouble(),
				drive.getMaxAngularSpeedRadPerSec());

		boolean fieldRelative = fieldRelativeSupplier.getAsBoolean();

		ChassisSpeeds speeds = new ChassisSpeeds(
				translation.getX(),
				translation.getY(),
				rotation.getRadians());

		if (mode == Mode.DRIVE_ROBOT_RELATIVE_SPEEDS) {
			speeds = robotRelativeSpeeds;
			fieldRelative = false;
		} else if (mode == Mode.HEADING_CONTROLLED || mode == Mode.HEADING_CONTROLLED_PASSIVE) {
			if (mode == Mode.HEADING_CONTROLLED_PASSIVE && Math.abs(speeds.omegaRadiansPerSecond) > 1E-3) {
				mode = Mode.DEFAULT;
			}
			else {
				speeds.omegaRadiansPerSecond = headingController.calculate();
			}
		}

		SmartDashboard.putString("Mode", mode.toString());
		drive.setRobotSpeeds(speeds, fieldRelative);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
