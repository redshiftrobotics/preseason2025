package frc.robot.subsystems.drive.controllers;

import static frc.robot.subsystems.drive.DriveConstants.HEADING_CONTROLLER_CONSTANTS;

import edu.wpi.first.math.controller.ProfiledPIDController;
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
			"Drive/HeadingController/MaxVelocityCoefficient", 1);
	private static final LoggedTunableNumber maxAccelerationCoefficient = new LoggedTunableNumber(
			"Drive/HeadingController/MaxAccelerationCoefficient", 1);

	private static final LoggedTunableNumber toleranceDegrees = new LoggedTunableNumber(
			"Drive/HeadingController/ToleranceDegrees", 1);

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

	/** Reset last position and rotation to prepare for new use */
	public void reset() {
		headingControllerRadians.reset(
				drive.getPose().getRotation().getRadians(), drive.getRobotSpeeds().omegaRadiansPerSecond);
	}

	/**
	 * Set goal heading in radians. Calculate will now give values to get to this heading.
	 * 
	 * @param headingRadians desired heading of chassis in radians
	 */
	public void setGoal(double headingRadians) {
		Logger.recordOutput(
			"Drive/HeadingController/Goal", headingRadians);

		headingControllerRadians.setGoal(headingRadians);
	}

	/**
	 * Get goal heading in radians.
	 * 
	 * @return desired heading of chassis
	 */
	public double getGoal() {
		return headingControllerRadians.getGoal().position;
	}

	/**
	 * Get speed chassis needs to rotation at to reach heading goal
	 * 
	 * @param goalHeadingRadians  desired heading of chassis in radians
	 * @return rotation speed to reach heading goal, omega radians per second
	 */
	public double calculate(double goalHeadingRadians) {
		setGoal(goalHeadingRadians);
		return calculate();
	}

	/**
	 * Get speed chassis needs to rotation at to reach heading goal
	 * 
	 * @return rotation speed to reach heading goal, omega radians per second
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
			"Drive/HeadingController/AtGoal", headingControllerRadians.atGoal());

		return output;
	}

	/**
	 * Get if the chassis heading is our goal heading
	 * 
	 * @return true if the absolute value of the position error is less than tolerance
	 */
	public boolean atGoal() {
		return headingControllerRadians.atGoal();
	}
}
