package frc.robot.subsystems.drive.controllers;

import static frc.robot.subsystems.drive.DriveConstants.HEADING_CONTROLLER_CONSTANTS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveConstants.ModuleLimits;
import frc.robot.utility.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Controller for rotating robot to goal heading using ProfiledPIDController */
public class HeadingController {

	// Tunable parameters
	private static final LoggedTunableNumber kP = new LoggedTunableNumber("HeadingController/kP",
			HEADING_CONTROLLER_CONSTANTS.kP());
	private static final LoggedTunableNumber kD = new LoggedTunableNumber("HeadingController/kD",
			HEADING_CONTROLLER_CONSTANTS.kD());

	private static final LoggedTunableNumber maxVelocityCoefficient = new LoggedTunableNumber(
			"HeadingController/MaxVelocityCoefficient", 0.8);
	private static final LoggedTunableNumber maxAccelerationCoefficient = new LoggedTunableNumber(
			"HeadingController/MaxAccelerationCoefficient", 0.8);
	private static final LoggedTunableNumber toleranceDegrees = new LoggedTunableNumber(
			"HeadingController/ToleranceDegrees", 2.0);

	private final Drive drive;

	private final ProfiledPIDController headingControllerRadians;
	private double output;

	/**
	 * Creates a new HeadingController object
	 *
	 * @param drive drivetrain of robot
	 */
	public HeadingController(Drive drive) {
		this.drive = drive;

		headingControllerRadians = new ProfiledPIDController(
				kP.get(),
				0,
				kD.get(),
				new TrapezoidProfile.Constraints(0.0, 0.0),
				Constants.LOOP_PERIOD_SECONDS);
		headingControllerRadians.enableContinuousInput(-Math.PI, Math.PI);
		headingControllerRadians.setTolerance(Units.degreesToRadians(toleranceDegrees.get()));

		reset();
	}

	public void reset() {
		headingControllerRadians.reset(
				drive.getPose().getRotation().getRadians(), drive.getRobotSpeeds().omegaRadiansPerSecond);
	}

	public void setGoal(Rotation2d heading) {
		headingControllerRadians.setGoal(heading.getRadians());
	}

	// public double getGoal(Rotation2d heading) {
		// headingControllerRadians.getGoal().position;
	// }

	public double calculate(Rotation2d goalHeading) {
		setGoal(goalHeading);
		return calculate();
	}

	/**
	 * @return omega radians per second to turn to goal heading
	 */
	public double calculate() {

		// Update profiled PID controller
		if (Constants.TUNING_MODE) {
			headingControllerRadians.setPID(kP.get(), 0, kD.get());
			headingControllerRadians.setTolerance(Units.degreesToRadians(toleranceDegrees.get()));
		}

		ModuleLimits moduleLimits = RobotState.getInstance().getModuleLimits();

		// Update constraints for profiled PID controller
		double maxAngularAcceleration = moduleLimits.maxDriveAcceleration()
				/ DriveConstants.DRIVE_CONFIG.driveBaseRadius()
				* maxAccelerationCoefficient.get();

		double maxAngularVelocity = moduleLimits.maxDriveAcceleration()
				/ DriveConstants.DRIVE_CONFIG.driveBaseRadius()
				* maxVelocityCoefficient.get();

		headingControllerRadians.setConstraints(
				new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration));

		// Calculate output
		output = headingControllerRadians.calculate(drive.getPose().getRotation().getRadians());

		Logger.recordOutput(
				"Drive/HeadingController/HeadingError", headingControllerRadians.getPositionError());
		Logger.recordOutput(
				"Drive/HeadingController/Output", output);

		return output;
	}

	/**
	 * @return true if within tolerance of aiming at goal
	 */
	@AutoLogOutput(key = "Drive/HeadingController/AtGoal")
	public boolean atGoal() {
		return headingControllerRadians.atGoal();
	}
}
