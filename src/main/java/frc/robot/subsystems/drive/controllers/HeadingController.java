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
import org.littletonrobotics.junction.Logger;

/** Controller for rotating robot to goal heading using ProfiledPIDController */
public class HeadingController {

	// Tunable parameters
	private static final LoggedTunableNumber Kp = new LoggedTunableNumber("HeadingController/kP",
			HEADING_CONTROLLER_CONSTANTS.Kp());
	private static final LoggedTunableNumber Kd = new LoggedTunableNumber("HeadingController/kD",
			HEADING_CONTROLLER_CONSTANTS.Kd());

	private static final LoggedTunableNumber maxVelocityCoefficient = new LoggedTunableNumber(
			"HeadingController/MaxVelocityCoefficient", 1);
	private static final LoggedTunableNumber maxAccelerationCoefficient = new LoggedTunableNumber(
			"HeadingController/MaxAccelerationCoefficient", 1);

	private static final LoggedTunableNumber toleranceDegrees = new LoggedTunableNumber(
			"HeadingController/ToleranceDegrees", 1);

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
				Kp.get(),
				0,
				Kd.get(),
				new TrapezoidProfile.Constraints(0.0, 0.0),
				Constants.LOOP_PERIOD_SECONDS);
		headingControllerRadians.enableContinuousInput(-Math.PI, Math.PI);
		headingControllerRadians.setTolerance(Units.degreesToRadians(toleranceDegrees.get()));
	}

	/**
	 * Creates a new HeadingController object with specified hading goal
	 *
	 * @param drive drivetrain of robot
	 * @param goal  target heading
	 */
	public HeadingController(Drive drive, Rotation2d heading) {
		this(drive);
		setGoal(heading);
	}

	public void reset() {
		headingControllerRadians.reset(
				drive.getPose().getRotation().getRadians(), drive.getRobotSpeeds().omegaRadiansPerSecond);
	}

	public void setGoal(Rotation2d heading) {
		headingControllerRadians.setGoal(heading.getRadians());
	}

	public Rotation2d getGoal() {
		return new Rotation2d(headingControllerRadians.getGoal().position);
	}

	public double calculate(Rotation2d goalHeading) {
		setGoal(goalHeading);
		return calculate();
	}

	/**
	 * @return omega radians per second to turn to goal heading
	 */
	public double calculate() {

		// Update profiled PID controller
		LoggedTunableNumber.ifChanged(hashCode(), () -> {
			headingControllerRadians.setPID(Kp.get(), 0, Kd.get());
		}, Kp, Kd);

		LoggedTunableNumber.ifChanged(hashCode(), () -> {
			headingControllerRadians.setTolerance(Units.degreesToRadians(toleranceDegrees.get()));
		}, toleranceDegrees);

		ModuleLimits moduleLimits = RobotState.getInstance().getModuleLimits();

		// Update constraints for profiled PID controller
		double maxAngularAcceleration = moduleLimits.maxDriveAcceleration()
				/ DriveConstants.DRIVE_CONFIG.driveBaseRadius()
				* maxAccelerationCoefficient.get();

		double maxAngularVelocity = moduleLimits.maxDriveVelocity()
				/ DriveConstants.DRIVE_CONFIG.driveBaseRadius()
				* maxVelocityCoefficient.get();

		headingControllerRadians.setConstraints(
				new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration));

		// Calculate output
		double measurement = drive.getPose().getRotation().getRadians();
		output = headingControllerRadians.calculate(measurement);

		Logger.recordOutput(
				"Drive/HeadingController/Output", output);
		Logger.recordOutput(
				"Drive/HeadingController/HeadingError", headingControllerRadians.getPositionError());
		Logger.recordOutput(
			"Drive/HeadingController/AtGoal", headingControllerRadians.atSetpoint());
		Logger.recordOutput(
			"Drive/HeadingController/AtGoal", headingControllerRadians.atGoal());

		return output;
	}

	/** Get if within tolerance of aiming at goal */
	public boolean atGoal() {
		return headingControllerRadians.atGoal();
	}
}
